#include "tsn/qav.h"
#include <math.h>
#include <string.h>

void tsn_qav_init(tsn_qav_queue_t *q) {
    if (!q) {
        return;
    }
    memset(q, 0, sizeof(*q));
    q->gate_open = true;
}

tsn_status_t tsn_qav_config(tsn_qav_queue_t *q, double idle_slope_bps,
                            uint64_t port_rate_bps, double hi_credit_bits,
                            double lo_credit_bits) {
    if (!q || idle_slope_bps <= 0.0 || port_rate_bps == 0 ||
        idle_slope_bps >= (double)port_rate_bps) {
        return TSN_ERR_INVALID_ARG;
    }
    q->enabled = true;
    q->idle_slope_bps = idle_slope_bps;
    q->send_slope_bps = idle_slope_bps - (double)port_rate_bps;
    q->hi_credit_bits = hi_credit_bits > 0.0 ? hi_credit_bits : 1e9;
    q->lo_credit_bits = lo_credit_bits < 0.0 ? lo_credit_bits : -1e9;
    q->credit_bits = 0.0;
    q->last_update_ps = 0;
    q->gate_open = true;
    return TSN_OK;
}

void tsn_qav_update(tsn_qav_queue_t *q, tsn_time_ps_t now_ps) {
    if (!q || !q->enabled) {
        return;
    }
    if (now_ps <= q->last_update_ps) {
        q->last_update_ps = now_ps;
        return;
    }

    double dt_sec = (double)(now_ps - q->last_update_ps) * 1e-12;
    q->last_update_ps = now_ps;

    if (q->transmitting) {
        /* Credit decreases at sendSlope while transmitting */
        q->credit_bits += q->send_slope_bps * dt_sec;
        if (q->credit_bits < q->lo_credit_bits) {
            q->credit_bits = q->lo_credit_bits;
        }
    } else if (!q->gate_open) {
        /* IEEE 802.1Q-2018 §8.6.8.2: credit freezes when Qbv gate is closed */
        return;
    } else if (q->has_queued_frame) {
        /* Frame waiting while gate is open: credit increases at idleSlope */
        q->credit_bits += q->idle_slope_bps * dt_sec;
        if (q->credit_bits > q->hi_credit_bits) {
            q->credit_bits = q->hi_credit_bits;
        }
    } else {
        /* Queue empty & gate open: negative credit recovers to 0; positive resets to 0 */
        if (q->credit_bits < 0.0) {
            q->credit_bits += q->idle_slope_bps * dt_sec;
            if (q->credit_bits > 0.0) {
                q->credit_bits = 0.0;
            }
        } else if (q->credit_bits > 0.0) {
            q->credit_bits = 0.0;
        }
    }
}

void tsn_qav_on_tx_start(tsn_qav_queue_t *q, tsn_time_ps_t now_ps) {
    if (!q || !q->enabled) {
        return;
    }
    tsn_qav_update(q, now_ps);
    q->transmitting = true;
}

void tsn_qav_on_tx_end(tsn_qav_queue_t *q, tsn_time_ps_t now_ps, bool still_has_frames) {
    if (!q || !q->enabled) {
        return;
    }
    tsn_qav_update(q, now_ps);
    q->transmitting = false;
    q->has_queued_frame = still_has_frames;
    if (!still_has_frames && q->credit_bits > 0.0) {
        q->credit_bits = 0.0;
    }
}

void tsn_qav_on_enqueue(tsn_qav_queue_t *q, tsn_time_ps_t now_ps) {
    if (!q || !q->enabled) {
        return;
    }
    tsn_qav_update(q, now_ps);
    q->has_queued_frame = true;
}

void tsn_qav_on_gate_change(tsn_qav_queue_t *q, tsn_time_ps_t now_ps, bool gate_open) {
    if (!q || !q->enabled) {
        return;
    }
    tsn_qav_update(q, now_ps);
    q->gate_open = gate_open;
}

bool tsn_qav_can_transmit(tsn_qav_queue_t *q, tsn_time_ps_t now_ps) {
    if (!q || !q->enabled) {
        return true;
    }
    tsn_qav_update(q, now_ps);
    return q->credit_bits >= -1e-9;
}

tsn_time_ps_t tsn_qav_time_until_ready_ps(tsn_qav_queue_t *q, tsn_time_ps_t now_ps) {
    if (!q || !q->enabled) {
        return 0;
    }
    tsn_qav_update(q, now_ps);
    if (q->credit_bits >= -1e-9 || !q->gate_open || !q->has_queued_frame || q->transmitting) {
        return 0;
    }
    double secs = (-q->credit_bits) / q->idle_slope_bps;
    tsn_time_ps_t ps = (tsn_time_ps_t)ceil(secs * 1e12);
    return ps > 0 ? ps : 1;
}
