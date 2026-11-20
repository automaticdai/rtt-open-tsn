#include "tsn/qbv.h"
#include <string.h>

static void qbv_timer_cb(void *arg);

static void qbv_emit_gate_metric(const tsn_qbv_port_t *qbv, tsn_time_ps_t now_ps) {
    if (!qbv->hal || !qbv->hal->emit_metric) {
        return;
    }
    tsn_metric_event_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = TSN_METRIC_QBV_GATE_CHANGE;
    evt.timestamp_ps = now_ps;
    evt.port_id = qbv->port_id;
    evt.value = (int64_t)qbv->oper_gate_states;
    qbv->hal->emit_metric(qbv->hal->ctx, &evt);
}

static void qbv_schedule_next(tsn_qbv_port_t *qbv, tsn_time_ps_t target_ps) {
    if (!qbv->hal || !qbv->hal->schedule_timer) {
        return;
    }
    if (qbv->timer_id != 0 && qbv->hal->cancel_timer) {
        qbv->hal->cancel_timer(qbv->hal->ctx, qbv->timer_id);
        qbv->timer_id = 0;
    }
    qbv->next_change_ps = target_ps;
    qbv->timer_id = qbv->hal->schedule_timer(qbv->hal->ctx, target_ps, qbv_timer_cb, qbv);
}

static void qbv_apply_entry(tsn_qbv_port_t *qbv, uint16_t idx, tsn_time_ps_t now_ps) {
    const tsn_qbv_gcl_entry_t *e = &qbv->oper.entries[idx];
    qbv->current_entry_idx = idx;
    qbv->oper_gate_states = e->gate_mask;

    if (e->op == TSN_QBV_OP_SET_AND_HOLD_MAC) {
        qbv->mac_held = true;
    } else if (e->op == TSN_QBV_OP_SET_AND_RELEASE_MAC) {
        qbv->mac_held = false;
    }

    qbv_emit_gate_metric(qbv, now_ps);

    tsn_time_ps_t next_ps = now_ps + e->duration_ps;
    /* Cap at cycle boundary if sum of entries exceeds cycle_time_ps */
    tsn_time_ps_t cycle_end_ps = qbv->cycle_start_ps + qbv->oper.cycle_time_ps;
    if (qbv->oper.cycle_time_ps > 0 && next_ps > cycle_end_ps) {
        next_ps = cycle_end_ps;
    }
    /* Cap at pending Admin ConfigChangeTime if earlier */
    if (qbv->config_pending && qbv->config_change_time_ps > now_ps &&
        qbv->config_change_time_ps < next_ps) {
        next_ps = qbv->config_change_time_ps;
    }

    qbv_schedule_next(qbv, next_ps);

    if (qbv->on_gate_change) {
        qbv->on_gate_change(qbv->cb_arg, qbv->port_id, qbv->oper_gate_states, qbv->mac_held);
    }
}

static void qbv_timer_cb(void *arg) {
    tsn_qbv_port_t *qbv = (tsn_qbv_port_t *)arg;
    if (!qbv || !qbv->enabled) {
        return;
    }
    qbv->timer_id = 0;
    tsn_time_ps_t now_ps = qbv->hal && qbv->hal->get_time_ps
                               ? qbv->hal->get_time_ps(qbv->hal->ctx)
                               : qbv->next_change_ps;

    /* Atomic Admin -> Oper swap at ConfigChangeTime */
    if (qbv->config_pending && now_ps >= qbv->config_change_time_ps) {
        qbv->oper = qbv->admin;
        qbv->oper_valid = true;
        qbv->config_pending = false;
        qbv->cycle_start_ps = now_ps;
        if (qbv->oper.entry_count > 0) {
            qbv_apply_entry(qbv, 0, now_ps);
        } else {
            qbv->oper_gate_states = qbv->oper.initial_gate_states;
            if (qbv->on_gate_change) {
                qbv->on_gate_change(qbv->cb_arg, qbv->port_id, qbv->oper_gate_states, qbv->mac_held);
            }
        }
        return;
    }

    if (!qbv->oper_valid || qbv->oper.entry_count == 0) {
        return;
    }

    uint16_t next_idx = (uint16_t)(qbv->current_entry_idx + 1u);
    tsn_time_ps_t cycle_end_ps = qbv->cycle_start_ps + qbv->oper.cycle_time_ps;

    if (next_idx >= qbv->oper.entry_count ||
        (qbv->oper.cycle_time_ps > 0 && now_ps >= cycle_end_ps)) {
        next_idx = 0;
        qbv->cycle_start_ps = now_ps;
    }

    qbv_apply_entry(qbv, next_idx, now_ps);
}

void tsn_qbv_init(tsn_qbv_port_t *qbv, uint8_t port_id, const tsn_hal_t *hal,
                  tsn_qbv_gate_change_cb_t cb, void *cb_arg) {
    if (!qbv) {
        return;
    }
    memset(qbv, 0, sizeof(*qbv));
    qbv->port_id = port_id;
    qbv->enabled = false;
    qbv->guard_band_enabled = true;
    qbv->hal = hal;
    qbv->on_gate_change = cb;
    qbv->cb_arg = cb_arg;
    qbv->oper_gate_states = 0xFFu; /* All gates open when disabled */
}

tsn_status_t tsn_qbv_set_admin_config(tsn_qbv_port_t *qbv, const tsn_qbv_config_t *admin,
                                      bool guard_band_enabled) {
    if (!qbv || !admin || admin->entry_count > TSN_QBV_MAX_GCL_ENTRIES) {
        return TSN_ERR_INVALID_ARG;
    }
    if (admin->entry_count > 0 && admin->cycle_time_ps <= 0) {
        return TSN_ERR_INVALID_ARG;
    }

    qbv->admin = *admin;
    qbv->guard_band_enabled = guard_band_enabled;
    qbv->enabled = true;

    tsn_time_ps_t now_ps = qbv->hal && qbv->hal->get_time_ps
                               ? qbv->hal->get_time_ps(qbv->hal->ctx)
                               : 0;

    tsn_time_ps_t change_time = admin->base_time_ps;
    if (change_time < now_ps && admin->cycle_time_ps > 0) {
        int64_t n = (now_ps - change_time + admin->cycle_time_ps - 1) / admin->cycle_time_ps;
        change_time += n * admin->cycle_time_ps;
    }

    qbv->config_pending = true;
    qbv->config_change_time_ps = change_time;

    if (!qbv->oper_valid) {
        qbv->oper_gate_states = admin->initial_gate_states ? admin->initial_gate_states : 0xFFu;
        if (change_time <= now_ps) {
            qbv_timer_cb(qbv);
        } else {
            qbv_schedule_next(qbv, change_time);
        }
    } else if (change_time < qbv->next_change_ps) {
        qbv_schedule_next(qbv, change_time);
    }

    return TSN_OK;
}

bool tsn_qbv_is_gate_open(const tsn_qbv_port_t *qbv, uint8_t tc) {
    if (!qbv || !qbv->enabled || tc >= TSN_NUM_TRAFFIC_CLASSES) {
        return true;
    }
    return ((qbv->oper_gate_states >> tc) & 1u) != 0u;
}

bool tsn_qbv_can_transmit(const tsn_qbv_port_t *qbv, uint8_t tc,
                          tsn_time_ps_t now_ps, tsn_time_ps_t tx_dur_ps) {
    if (!qbv || !qbv->enabled || !qbv->oper_valid || tc >= TSN_NUM_TRAFFIC_CLASSES) {
        return true;
    }
    if (!tsn_qbv_is_gate_open(qbv, tc)) {
        return false;
    }
    if (!qbv->guard_band_enabled) {
        return true;
    }
    if (now_ps + tx_dur_ps <= qbv->next_change_ps) {
        return true;
    }

    /* Check whether the next GCL entry keeps this TC's gate open */
    if (qbv->oper.entry_count > 0) {
        uint16_t next_idx = (uint16_t)((qbv->current_entry_idx + 1u) % qbv->oper.entry_count);
        uint8_t next_mask = qbv->oper.entries[next_idx].gate_mask;
        if (((next_mask >> tc) & 1u) != 0u) {
            /* Gate stays open across the immediate boundary; allow if fits in combined window */
            tsn_time_ps_t combined_end = qbv->next_change_ps + qbv->oper.entries[next_idx].duration_ps;
            if (now_ps + tx_dur_ps <= combined_end) {
                return true;
            }
        }
    }
    return false;
}
