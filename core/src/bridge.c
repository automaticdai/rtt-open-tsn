#include "tsn/bridge.h"
#include <string.h>

static void emit_metric(const tsn_bridge_t *br, tsn_metric_type_t type,
                        uint8_t port_id, uint8_t tc, const tsn_frame_t *frame,
                        int64_t value) {
    if (!br || !br->hal.emit_metric) {
        return;
    }
    tsn_metric_event_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = type;
    evt.timestamp_ps = br->hal.get_time_ps ? br->hal.get_time_ps(br->hal.ctx) : 0;
    evt.port_id = port_id;
    evt.tc = tc;
#ifdef TSN_SIM
    if (frame) {
        evt.stream_id = frame->stream_id;
        evt.frame_uid = frame->frame_uid;
    }
#else
    (void)frame;
#endif
    evt.value = value;
    br->hal.emit_metric(br->hal.ctx, &evt);
}

static void bridge_try_preempt(tsn_bridge_t *br, uint8_t egress_port, tsn_time_ps_t now_ps) {
    if (!br || egress_port >= br->num_ports) {
        return;
    }
    tsn_bridge_port_t *port = &br->ports[egress_port];
    if (!port->tx_busy || !port->qbu.enabled || !br->hal.preempt_tx) {
        return;
    }
    uint16_t cut_bytes = 0;
    if (tsn_qbu_can_preempt(&port->qbu, now_ps, port->link_rate_bps, &cut_bytes)) {
        if (br->hal.preempt_tx(br->hal.ctx, egress_port, cut_bytes) == TSN_OK) {
            tsn_qbu_mark_preempted(&port->qbu, cut_bytes);
        }
    }
}

static void bridge_qav_wakeup_cb(void *arg) {
    tsn_qav_wakeup_ctx_t *w = (tsn_qav_wakeup_ctx_t *)arg;
    if (!w || !w->br) {
        return;
    }
    w->br->ports[w->port_id].qav[w->tc].wakeup_timer_id = 0;
    tsn_bridge_try_tx(w->br, w->port_id);
}

static void bridge_on_qbv_gate_change(void *arg, uint8_t port_id,
                                      uint8_t new_gate_mask, bool mac_held) {
    tsn_bridge_t *br = (tsn_bridge_t *)arg;
    if (!br || port_id >= br->num_ports) {
        return;
    }
    tsn_time_ps_t now_ps = br->hal.get_time_ps ? br->hal.get_time_ps(br->hal.ctx) : 0;
    tsn_bridge_port_t *port = &br->ports[port_id];
    port->qbu.hold_active = mac_held;

    bool express_ready = false;
    for (uint8_t tc = 0; tc < TSN_NUM_TRAFFIC_CLASSES; ++tc) {
        bool open = ((new_gate_mask >> tc) & 1u) != 0u;
        tsn_qav_on_gate_change(&port->qav[tc], now_ps, open);
        if (open && !tsn_qbu_is_preemptable(&port->qbu, tc) &&
            !tsn_queue_is_empty(&port->queues[tc])) {
            express_ready = true;
        }
    }

    if (port->tx_busy && port->qbu.enabled && (mac_held || express_ready)) {
        bridge_try_preempt(br, port_id, now_ps);
    }

    tsn_bridge_try_tx(br, port_id);
}

static tsn_status_t gptp_bridge_tx_adapter(void *ctx, uint8_t port_id,
                                           const tsn_frame_t *frame) {
    tsn_bridge_t *br = (tsn_bridge_t *)ctx;
    if (!br || !frame || port_id >= br->num_ports) {
        return TSN_ERR_INVALID_ARG;
    }
    tsn_status_t st = tsn_queue_enqueue(&br->ports[port_id].gptp_queue, frame);
    if (st == TSN_OK) {
        tsn_bridge_try_tx(br, port_id);
    }
    return st;
}

tsn_status_t tsn_bridge_init(tsn_bridge_t *br, uint8_t num_ports, const tsn_hal_t *hal) {
    if (!br || !hal || num_ports == 0 || num_ports > TSN_MAX_PORTS) {
        return TSN_ERR_INVALID_ARG;
    }
    memset(br, 0, sizeof(*br));
    br->num_ports = num_ports;
    br->hal = *hal;
    tsn_frer_init(&br->frer);

    for (uint8_t p = 0; p < num_ports; ++p) {
        tsn_bridge_port_t *port = &br->ports[p];
        port->port_id = p;
        port->enabled = true;
        port->tx_busy = false;
        port->active_tx_tc = -1;
        port->link_rate_bps = 1000000000ULL;
        tsn_queue_init(&port->gptp_queue, 16u);
        tsn_qbv_init(&port->qbv, p, &br->hal, bridge_on_qbv_gate_change, br);
        tsn_qbu_init(&port->qbu);
        for (uint8_t tc = 0; tc < TSN_NUM_TRAFFIC_CLASSES; ++tc) {
            tsn_queue_init(&port->queues[tc], TSN_QUEUE_CAPACITY);
            tsn_qav_init(&port->qav[tc]);
            port->pcp_to_tc[tc] = tc;
        }
    }
    return TSN_OK;
}

tsn_status_t tsn_bridge_config_port(tsn_bridge_t *br, uint8_t port_id,
                                    uint64_t link_rate_bps, uint16_t queue_capacity) {
    if (!br || port_id >= br->num_ports) {
        return TSN_ERR_INVALID_ARG;
    }
    tsn_bridge_port_t *port = &br->ports[port_id];
    port->link_rate_bps = link_rate_bps;
    for (uint8_t tc = 0; tc < TSN_NUM_TRAFFIC_CLASSES; ++tc) {
        tsn_queue_init(&port->queues[tc], queue_capacity);
    }
    return TSN_OK;
}

tsn_status_t tsn_bridge_add_fdb(tsn_bridge_t *br, const tsn_mac_t *dst_mac,
                                uint16_t vid, uint32_t port_mask) {
    if (!br || !dst_mac) {
        return TSN_ERR_INVALID_ARG;
    }
    for (uint16_t i = 0; i < br->fdb_count; ++i) {
        if (br->fdb[i].valid && br->fdb[i].vid == vid &&
            tsn_mac_equal(&br->fdb[i].dst_mac, dst_mac)) {
            br->fdb[i].port_mask = port_mask;
            return TSN_OK;
        }
    }
    if (br->fdb_count >= TSN_MAX_FDB_ENTRIES) {
        return TSN_ERR_FULL;
    }
    tsn_fdb_entry_t *e = &br->fdb[br->fdb_count++];
    e->dst_mac = *dst_mac;
    e->vid = vid;
    e->port_mask = port_mask;
    e->valid = true;
    return TSN_OK;
}

tsn_status_t tsn_bridge_config_qbv(tsn_bridge_t *br, uint8_t port_id,
                                   const tsn_qbv_config_t *admin, bool guard_band_enabled) {
    if (!br || port_id >= br->num_ports) {
        return TSN_ERR_INVALID_ARG;
    }
    return tsn_qbv_set_admin_config(&br->ports[port_id].qbv, admin, guard_band_enabled);
}

tsn_status_t tsn_bridge_config_qav(tsn_bridge_t *br, uint8_t port_id, uint8_t tc,
                                   double idle_slope_bps, double hi_credit_bits,
                                   double lo_credit_bits) {
    if (!br || port_id >= br->num_ports || tc >= TSN_NUM_TRAFFIC_CLASSES) {
        return TSN_ERR_INVALID_ARG;
    }
    tsn_bridge_port_t *port = &br->ports[port_id];
    tsn_status_t st = tsn_qav_config(&port->qav[tc], idle_slope_bps,
                                     port->link_rate_bps, hi_credit_bits, lo_credit_bits);
    if (st == TSN_OK) {
        tsn_time_ps_t now_ps = br->hal.get_time_ps ? br->hal.get_time_ps(br->hal.ctx) : 0;
        tsn_qav_on_gate_change(&port->qav[tc], now_ps, tsn_qbv_is_gate_open(&port->qbv, tc));
    }
    return st;
}

tsn_status_t tsn_bridge_config_qbu(tsn_bridge_t *br, uint8_t port_id,
                                   uint8_t preemptable_mask, uint8_t add_frag_size) {
    if (!br || port_id >= br->num_ports) {
        return TSN_ERR_INVALID_ARG;
    }
    return tsn_qbu_config(&br->ports[port_id].qbu, preemptable_mask, add_frag_size);
}

tsn_status_t tsn_bridge_add_frer_gen(tsn_bridge_t *br, const tsn_mac_t *dst_mac,
                                     uint16_t vid, uint32_t split_port_mask) {
    if (!br) {
        return TSN_ERR_INVALID_ARG;
    }
    return tsn_frer_add_gen_rule(&br->frer, dst_mac, vid, split_port_mask);
}

tsn_status_t tsn_bridge_add_frer_recov(tsn_bridge_t *br, const tsn_mac_t *dst_mac,
                                       uint16_t vid, tsn_frer_algo_t algo,
                                       uint8_t history_length, bool strip_rtag,
                                       tsn_time_ps_t reset_timeout_ps) {
    if (!br) {
        return TSN_ERR_INVALID_ARG;
    }
    return tsn_frer_add_recov_rule(&br->frer, dst_mac, vid, algo, history_length,
                                   strip_rtag, reset_timeout_ps);
}

static tsn_time_ps_t gptp_hal_get_time(void *ctx) {
    tsn_bridge_t *br = (tsn_bridge_t *)ctx;
    return br->hal.get_time_ps(br->hal.ctx);
}
static void gptp_hal_adjust_clock(void *ctx, int64_t offset_ps, double rate_ppb) {
    tsn_bridge_t *br = (tsn_bridge_t *)ctx;
    if (br->hal.adjust_clock) br->hal.adjust_clock(br->hal.ctx, offset_ps, rate_ppb);
}
static uint64_t gptp_hal_schedule_timer(void *ctx, tsn_time_ps_t target,
                                        tsn_timer_cb_t cb, void *arg) {
    tsn_bridge_t *br = (tsn_bridge_t *)ctx;
    return br->hal.schedule_timer(br->hal.ctx, target, cb, arg);
}
static void gptp_hal_cancel_timer(void *ctx, uint64_t id) {
    tsn_bridge_t *br = (tsn_bridge_t *)ctx;
    if (br->hal.cancel_timer) br->hal.cancel_timer(br->hal.ctx, id);
}
static void gptp_hal_emit_metric(void *ctx, const tsn_metric_event_t *evt) {
    tsn_bridge_t *br = (tsn_bridge_t *)ctx;
    if (br->hal.emit_metric) br->hal.emit_metric(br->hal.ctx, evt);
}

void tsn_bridge_init_gptp(tsn_bridge_t *br, const tsn_gptp_config_t *cfg,
                          const tsn_mac_t *bridge_mac) {
    if (!br || !cfg) {
        return;
    }
    br->gptp_hal.ctx = br;
    br->gptp_hal.get_time_ps = gptp_hal_get_time;
    br->gptp_hal.adjust_clock = gptp_hal_adjust_clock;
    br->gptp_hal.schedule_timer = gptp_hal_schedule_timer;
    br->gptp_hal.cancel_timer = gptp_hal_cancel_timer;
    br->gptp_hal.tx_frame = gptp_bridge_tx_adapter;
    br->gptp_hal.emit_metric = gptp_hal_emit_metric;
    tsn_gptp_init(&br->gptp, cfg, &br->gptp_hal, bridge_mac);
}

void tsn_bridge_on_tx_timestamp(tsn_bridge_t *br, uint8_t egress_port,
                                const tsn_frame_t *frame, tsn_time_ps_t tx_ts_ps) {
    if (!br || !frame) {
        return;
    }
    if (frame->ethertype == TSN_ETHERTYPE_GPTP && br->gptp.enabled) {
        tsn_gptp_on_tx_timestamp(&br->gptp, egress_port, frame, tx_ts_ps);
    }
}

static uint32_t lookup_egress_mask(const tsn_bridge_t *br, uint8_t ingress_port,
                                   const tsn_frame_t *frame) {
    uint16_t frame_vid = frame->has_vlan ? frame->vid : 0u;

    for (uint16_t i = 0; i < br->fdb_count; ++i) {
        const tsn_fdb_entry_t *e = &br->fdb[i];
        if (e->valid && e->vid == frame_vid &&
            tsn_mac_equal(&e->dst_mac, &frame->dst_mac)) {
            return e->port_mask & ~(1u << ingress_port);
        }
    }
    for (uint16_t i = 0; i < br->fdb_count; ++i) {
        const tsn_fdb_entry_t *e = &br->fdb[i];
        if (e->valid && e->vid == 0u &&
            tsn_mac_equal(&e->dst_mac, &frame->dst_mac)) {
            return e->port_mask & ~(1u << ingress_port);
        }
    }
    uint32_t all_ports = (br->num_ports >= 32u) ? 0xFFFFFFFFu : ((1u << br->num_ports) - 1u);
    return all_ports & ~(1u << ingress_port);
}

tsn_status_t tsn_bridge_rx_frame(tsn_bridge_t *br, uint8_t ingress_port,
                                 const tsn_frame_t *frame) {
    if (!br || !frame || ingress_port >= br->num_ports) {
        return TSN_ERR_INVALID_ARG;
    }

    /* 1. IEEE 802.3br Ingress Reassembly (Express SMD-E passes through immediately) */
    tsn_frame_t reassembled;
    if (!tsn_qbu_rx_mpacket(&br->ports[ingress_port].qbu, frame, &reassembled)) {
        return TSN_OK;
    }

    /* 2. Intercept IEEE 802.1AS gPTP frames before L2 FDB forwarding */
    if (reassembled.ethertype == TSN_ETHERTYPE_GPTP) {
        if (br->gptp.enabled) {
            tsn_gptp_rx_frame(&br->gptp, ingress_port, &reassembled);
        }
        return TSN_OK;
    }

    tsn_time_ps_t now_ps = br->hal.get_time_ps ? br->hal.get_time_ps(br->hal.ctx) : 0;

    /* 3. IEEE 802.1CB FRER Sequence Recovery (eliminate duplicates before forwarding) */
    if (!tsn_frer_process_recovery(&br->frer, &reassembled, now_ps)) {
        return TSN_OK;
    }

    /* 4. IEEE 802.1CB FRER Sequence Generation & Stream Splitting (or normal FDB lookup) */
    uint32_t egress_mask = 0;
    if (tsn_frer_process_generation(&br->frer, &reassembled, &egress_mask)) {
        egress_mask &= ~(1u << ingress_port);
    } else {
        egress_mask = lookup_egress_mask(br, ingress_port, &reassembled);
    }
    uint8_t pcp = reassembled.has_vlan ? (reassembled.pcp & 0x07u) : 0u;

    tsn_frame_t fwd_frame = reassembled;
#ifdef TSN_SIM
    fwd_frame.hop_count++;
#endif

    for (uint8_t p = 0; p < br->num_ports; ++p) {
        if ((egress_mask & (1u << p)) == 0u) {
            continue;
        }
        tsn_bridge_port_t *port = &br->ports[p];
        if (!port->enabled) {
            continue;
        }
        uint8_t tc = port->pcp_to_tc[pcp];
        tsn_status_t st = tsn_queue_enqueue(&port->queues[tc], &fwd_frame);
        if (st == TSN_OK) {
            tsn_qav_on_enqueue(&port->qav[tc], now_ps);
            emit_metric(br, TSN_METRIC_FRAME_ENQUEUE, p, tc, &fwd_frame,
                        (int64_t)tsn_queue_size(&port->queues[tc]));

            /* Dynamic 802.1Qbu preemption: if an Express frame arrives while a pMAC frame
             * is transmitting on the wire, preempt it immediately if eligible. */
            if (port->tx_busy && port->qbu.enabled && !tsn_qbu_is_preemptable(&port->qbu, tc) &&
                tsn_qbv_is_gate_open(&port->qbv, tc) &&
                tsn_qav_can_transmit(&port->qav[tc], now_ps)) {
                bridge_try_preempt(br, p, now_ps);
            }

            tsn_bridge_try_tx(br, p);
        } else {
            emit_metric(br, TSN_METRIC_FRAME_DROP, p, tc, &fwd_frame, 1);
        }
    }
    return TSN_OK;
}

static void schedule_qav_wakeup(tsn_bridge_t *br, uint8_t egress_port, uint8_t tc,
                                tsn_time_ps_t now_ps) {
    tsn_bridge_port_t *port = &br->ports[egress_port];
    tsn_time_ps_t wait_ps = tsn_qav_time_until_ready_ps(&port->qav[tc], now_ps);
    if (wait_ps > 0 && br->hal.schedule_timer) {
        if (port->qav[tc].wakeup_timer_id != 0 && br->hal.cancel_timer) {
            br->hal.cancel_timer(br->hal.ctx, port->qav[tc].wakeup_timer_id);
        }
        tsn_qav_wakeup_ctx_t *w = &port->qav_wakeup[tc];
        w->br = br;
        w->port_id = egress_port;
        w->tc = tc;
        port->qav[tc].wakeup_timer_id = br->hal.schedule_timer(
            br->hal.ctx, now_ps + wait_ps, bridge_qav_wakeup_cb, w);
    }
}

static bool launch_frame_tx(tsn_bridge_t *br, uint8_t egress_port, uint8_t tc,
                            bool is_resuming, tsn_time_ps_t now_ps, tsn_frame_t *frame) {
    tsn_bridge_port_t *port = &br->ports[egress_port];
    tsn_qbu_prepare_tx_start(&port->qbu, tc, is_resuming, now_ps, frame);

    port->tx_busy = true;
    port->active_tx_tc = (int8_t)tc;
    tsn_qav_on_tx_start(&port->qav[tc], now_ps);
    emit_metric(br, TSN_METRIC_FRAME_TX_START, egress_port, tc,
                frame, (int64_t)frame->length_bytes);

    if (br->hal.tx_frame) {
        tsn_status_t st = br->hal.tx_frame(br->hal.ctx, egress_port, frame);
        if (st != TSN_OK) {
            port->tx_busy = false;
            port->active_tx_tc = -1;
            tsn_qbu_on_tx_complete(&port->qbu);
            tsn_qav_on_tx_end(&port->qav[tc], now_ps, !tsn_queue_is_empty(&port->queues[tc]));
            return false;
        }
    }
    return true;
}

void tsn_bridge_try_tx(tsn_bridge_t *br, uint8_t egress_port) {
    if (!br || egress_port >= br->num_ports) {
        return;
    }
    tsn_bridge_port_t *port = &br->ports[egress_port];
    if (!port->enabled || port->tx_busy) {
        return;
    }

    /* 1. Highest priority: 802.1AS gPTP control queue */
    if (!tsn_queue_is_empty(&port->gptp_queue)) {
        tsn_frame_t frame;
        if (tsn_queue_dequeue(&port->gptp_queue, &frame) == TSN_OK) {
            frame.smd_type = TSN_SMD_E;
            port->tx_busy = true;
            port->active_tx_tc = -1;
            if (br->hal.tx_frame) {
                if (br->hal.tx_frame(br->hal.ctx, egress_port, &frame) != TSN_OK) {
                    port->tx_busy = false;
                }
            }
        }
        return;
    }

    tsn_time_ps_t now_ps = br->hal.get_time_ps ? br->hal.get_time_ps(br->hal.ctx) : 0;

    /* 2. Express TCs first (highest TC down to lowest TC) */
    for (int tc = (int)TSN_NUM_TRAFFIC_CLASSES - 1; tc >= 0; --tc) {
        if (tsn_qbu_is_preemptable(&port->qbu, (uint8_t)tc)) {
            continue;
        }
        const tsn_frame_t *peek = tsn_queue_peek(&port->queues[tc]);
        if (!peek) {
            continue;
        }
        tsn_time_ps_t tx_dur_ps =
            tsn_frame_tx_time_ps(peek->length_bytes, port->link_rate_bps, true);
        if (!tsn_qbv_can_transmit(&port->qbv, (uint8_t)tc, now_ps, tx_dur_ps)) {
            continue;
        }
        if (!tsn_qav_can_transmit(&port->qav[tc], now_ps)) {
            schedule_qav_wakeup(br, egress_port, (uint8_t)tc, now_ps);
            continue;
        }
        tsn_frame_t frame;
        if (tsn_queue_dequeue(&port->queues[tc], &frame) == TSN_OK) {
            launch_frame_tx(br, egress_port, (uint8_t)tc, false, now_ps, &frame);
        }
        return;
    }

    /* 3. Preemptable TCs (blocked while Qbv SetAndHoldMAC is active) */
    if (!port->qbu.enabled || port->qbu.hold_active) {
        return;
    }

    /* 3a. Resume a previously preempted pMAC frame before starting a new pMAC frame */
    if (port->qbu.tx.has_suspended) {
        uint8_t stc = port->qbu.tx.suspended_tc;
        uint16_t rem = (uint16_t)(port->qbu.tx.suspended_frame.length_bytes -
                                  port->qbu.tx.suspended_bytes_sent);
        uint16_t guard_bytes = tsn_qbu_min_frag_bytes(&port->qbu) + TSN_QBU_MCRC_BYTES;
        if (rem < guard_bytes) {
            guard_bytes = rem;
        }
        tsn_time_ps_t guard_dur_ps =
            tsn_frame_tx_time_ps(guard_bytes, port->link_rate_bps, true);
        if (tsn_qbv_can_transmit(&port->qbv, stc, now_ps, guard_dur_ps)) {
            if (tsn_qav_can_transmit(&port->qav[stc], now_ps)) {
                tsn_frame_t frag;
                tsn_frame_init(&frag);
                launch_frame_tx(br, egress_port, stc, true, now_ps, &frag);
                return;
            } else {
                schedule_qav_wakeup(br, egress_port, stc, now_ps);
            }
        }
    }

    /* 3b. Start a new preemptable frame */
    for (int tc = (int)TSN_NUM_TRAFFIC_CLASSES - 1; tc >= 0; --tc) {
        if (!tsn_qbu_is_preemptable(&port->qbu, (uint8_t)tc)) {
            continue;
        }
        const tsn_frame_t *peek = tsn_queue_peek(&port->queues[tc]);
        if (!peek) {
            continue;
        }
        /* Preemptable guard band is reduced to min_frag + mCRC (if frame is larger) */
        uint16_t guard_bytes = tsn_qbu_min_frag_bytes(&port->qbu) + TSN_QBU_MCRC_BYTES;
        if (peek->length_bytes < guard_bytes) {
            guard_bytes = peek->length_bytes;
        }
        tsn_time_ps_t guard_dur_ps =
            tsn_frame_tx_time_ps(guard_bytes, port->link_rate_bps, true);
        if (!tsn_qbv_can_transmit(&port->qbv, (uint8_t)tc, now_ps, guard_dur_ps)) {
            continue;
        }
        if (!tsn_qav_can_transmit(&port->qav[tc], now_ps)) {
            schedule_qav_wakeup(br, egress_port, (uint8_t)tc, now_ps);
            continue;
        }
        tsn_frame_t frame;
        if (tsn_queue_dequeue(&port->queues[tc], &frame) == TSN_OK) {
            launch_frame_tx(br, egress_port, (uint8_t)tc, false, now_ps, &frame);
        }
        return;
    }
}

void tsn_bridge_on_tx_complete(tsn_bridge_t *br, uint8_t egress_port) {
    if (!br || egress_port >= br->num_ports) {
        return;
    }
    tsn_bridge_port_t *port = &br->ports[egress_port];
    tsn_time_ps_t now_ps = br->hal.get_time_ps ? br->hal.get_time_ps(br->hal.ctx) : 0;

    if (port->active_tx_tc >= 0 && port->active_tx_tc < (int8_t)TSN_NUM_TRAFFIC_CLASSES) {
        uint8_t tc = (uint8_t)port->active_tx_tc;
        bool has_more = !tsn_queue_is_empty(&port->queues[tc]) ||
                        (port->qbu.tx.has_suspended && port->qbu.tx.suspended_tc == tc);
        tsn_qav_on_tx_end(&port->qav[tc], now_ps, has_more);
    }
    tsn_qbu_on_tx_complete(&port->qbu);
    port->tx_busy = false;
    port->active_tx_tc = -1;
    tsn_bridge_try_tx(br, egress_port);
}
