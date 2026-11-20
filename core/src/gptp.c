#include "tsn/gptp.h"
#include <math.h>
#include <string.h>

static const tsn_mac_t GPTP_MCAST_MAC = {{0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E}};

static void gptp_sync_timer_cb(void *arg);
static void gptp_pdelay_timer_cb(void *arg);

static void make_gptp_frame(const tsn_gptp_node_t *gptp, uint8_t port_id,
                            tsn_gptp_msg_type_t type, uint16_t seq_id,
                            tsn_frame_t *out) {
    tsn_frame_init(out);
    out->dst_mac = GPTP_MCAST_MAC;
    out->src_mac = gptp->node_mac;
    out->ethertype = TSN_ETHERTYPE_GPTP;
    out->has_vlan = false;
    out->pcp = 7;
    out->length_bytes = 64u; /* Minimum Ethernet frame */
    out->gptp.msg_type = type;
    out->gptp.seq_id = seq_id;
    out->gptp.src_port_id = port_id;
}

static void gptp_send_frame(const tsn_gptp_node_t *gptp, uint8_t port_id,
                            const tsn_frame_t *frame) {
    if (gptp && gptp->hal && gptp->hal->tx_frame) {
        gptp->hal->tx_frame(gptp->hal->ctx, port_id, frame);
    }
}

static void gptp_schedule_sync_timer(tsn_gptp_node_t *gptp) {
    if (!gptp->enabled || !gptp->cfg.is_gm || gptp->cfg.sync_interval_ps <= 0) {
        return;
    }
    tsn_time_ps_t now_ps = gptp->hal->get_time_ps(gptp->hal->ctx);
    gptp->sync_timer_id = gptp->hal->schedule_timer(
        gptp->hal->ctx, now_ps + gptp->cfg.sync_interval_ps, gptp_sync_timer_cb, gptp);
}

static void gptp_schedule_pdelay_timer(tsn_gptp_node_t *gptp) {
    if (!gptp->enabled || gptp->cfg.pdelay_interval_ps <= 0) {
        return;
    }
    tsn_time_ps_t now_ps = gptp->hal->get_time_ps(gptp->hal->ctx);
    gptp->pdelay_timer_id = gptp->hal->schedule_timer(
        gptp->hal->ctx, now_ps + gptp->cfg.pdelay_interval_ps, gptp_pdelay_timer_cb, gptp);
}

static void gptp_send_gm_sync(tsn_gptp_node_t *gptp) {
    uint16_t seq = ++gptp->gm_sync_seq;
    for (uint8_t p = 0; p < gptp->cfg.num_ports; ++p) {
        if (gptp->ports[p].role != TSN_GPTP_ROLE_MASTER) {
            continue;
        }
        tsn_frame_t sync_f;
        make_gptp_frame(gptp, p, TSN_GPTP_SYNC, seq, &sync_f);
        sync_f.gptp.precise_origin_ts_ps = gptp->hal->get_time_ps(gptp->hal->ctx);
        sync_f.gptp.correction_field_ps = 0;
        gptp_send_frame(gptp, p, &sync_f);
    }
}

static void gptp_send_pdelay_reqs(tsn_gptp_node_t *gptp) {
    for (uint8_t p = 0; p < gptp->cfg.num_ports; ++p) {
        tsn_gptp_port_t *port = &gptp->ports[p];
        if (port->role == TSN_GPTP_ROLE_DISABLED) {
            continue;
        }
        uint16_t seq = ++port->pdelay_seq_id;
        tsn_frame_t req;
        make_gptp_frame(gptp, p, TSN_GPTP_PDELAY_REQ, seq, &req);
        gptp_send_frame(gptp, p, &req);
    }
}

static void gptp_sync_timer_cb(void *arg) {
    tsn_gptp_node_t *gptp = (tsn_gptp_node_t *)arg;
    gptp->sync_timer_id = 0;
    gptp_send_gm_sync(gptp);
    gptp_schedule_sync_timer(gptp);
}

static void gptp_pdelay_timer_cb(void *arg) {
    tsn_gptp_node_t *gptp = (tsn_gptp_node_t *)arg;
    gptp->pdelay_timer_id = 0;
    gptp_send_pdelay_reqs(gptp);
    gptp_schedule_pdelay_timer(gptp);
}

void tsn_gptp_init(tsn_gptp_node_t *gptp, const tsn_gptp_config_t *cfg,
                   const tsn_hal_t *hal, const tsn_mac_t *node_mac) {
    if (!gptp || !cfg || !hal) {
        return;
    }
    memset(gptp, 0, sizeof(*gptp));
    gptp->enabled = true;
    gptp->cfg = *cfg;
    gptp->hal = hal;
    if (node_mac) {
        gptp->node_mac = *node_mac;
    }
    if (gptp->cfg.servo_kp <= 0.0) {
        gptp->cfg.servo_kp = 0.7;
    }
    if (gptp->cfg.servo_ki <= 0.0) {
        gptp->cfg.servo_ki = 0.3;
    }
    for (uint8_t p = 0; p < TSN_MAX_PORTS; ++p) {
        gptp->ports[p].port_id = p;
        gptp->ports[p].role = TSN_GPTP_ROLE_DISABLED;
        gptp->ports[p].neighbor_rate_ratio = 1.0;
    }
}

void tsn_gptp_set_port_role(tsn_gptp_node_t *gptp, uint8_t port_id,
                            tsn_gptp_port_role_t role) {
    if (!gptp || port_id >= TSN_MAX_PORTS) {
        return;
    }
    gptp->ports[port_id].role = role;
}

void tsn_gptp_start(tsn_gptp_node_t *gptp) {
    if (!gptp || !gptp->enabled || !gptp->hal) {
        return;
    }
    /* Trigger initial Pdelay shortly after t=0 so link delays are measured before first Sync */
    tsn_time_ps_t now_ps = gptp->hal->get_time_ps(gptp->hal->ctx);
    gptp->pdelay_timer_id = gptp->hal->schedule_timer(
        gptp->hal->ctx, now_ps + 10 * TSN_PS_PER_US, gptp_pdelay_timer_cb, gptp);

    if (gptp->cfg.is_gm) {
        gptp->sync_timer_id = gptp->hal->schedule_timer(
            gptp->hal->ctx, now_ps + 50 * TSN_PS_PER_US, gptp_sync_timer_cb, gptp);
    }
}

void tsn_gptp_on_tx_timestamp(tsn_gptp_node_t *gptp, uint8_t port_id,
                              const tsn_frame_t *frame, tsn_time_ps_t tx_ts_ps) {
    if (!gptp || !gptp->enabled || !frame || port_id >= gptp->cfg.num_ports) {
        return;
    }
    tsn_gptp_port_t *port = &gptp->ports[port_id];

    if (frame->gptp.msg_type == TSN_GPTP_PDELAY_REQ) {
        port->t1_req_tx_ps = tx_ts_ps;
    } else if (frame->gptp.msg_type == TSN_GPTP_PDELAY_RESP) {
        tsn_frame_t fup;
        make_gptp_frame(gptp, port_id, TSN_GPTP_PDELAY_RESP_FUP, frame->gptp.seq_id, &fup);
        fup.gptp.response_origin_ts_ps = tx_ts_ps;
        gptp_send_frame(gptp, port_id, &fup);
    } else if (frame->gptp.msg_type == TSN_GPTP_SYNC) {
        tsn_frame_t fup;
        make_gptp_frame(gptp, port_id, TSN_GPTP_FOLLOW_UP, frame->gptp.seq_id, &fup);
        if (gptp->cfg.is_gm) {
            fup.gptp.precise_origin_ts_ps = tx_ts_ps;
            fup.gptp.correction_field_ps = 0;
        } else {
            /* Bridge forwarding: add residence time (tx_ts - rx_ts) to correctionField */
            fup.gptp.precise_origin_ts_ps = frame->gptp.precise_origin_ts_ps;
            int64_t residence_ps = tx_ts_ps - frame->gptp.request_receipt_ts_ps;
            fup.gptp.correction_field_ps = frame->gptp.correction_field_ps + residence_ps;
        }
        gptp_send_frame(gptp, port_id, &fup);
    }
}

static void gptp_run_servo(tsn_gptp_node_t *gptp, int64_t offset_err_ps) {
    gptp->last_offset_err_ps = offset_err_ps;
    gptp->sync_rx_count++;

    if (!gptp->servo_initialized) {
        /* First sync: step local clock directly to GM time */
        gptp->servo_initialized = true;
        if (gptp->hal->adjust_clock) {
            gptp->hal->adjust_clock(gptp->hal->ctx, offset_err_ps, gptp->servo_rate_ppb);
        }
    } else {
        /* PI frequency + fractional phase servo */
        double dt_sec = (double)gptp->cfg.sync_interval_ps * 1e-12;
        if (dt_sec <= 0.0) {
            dt_sec = 1e-3;
        }
        /* Convert phase error over sync_interval into equivalent rate error in ppb */
        double err_ppb = ((double)offset_err_ps / (double)gptp->cfg.sync_interval_ps) * 1e9;
        gptp->servo_integral_ppb += gptp->cfg.servo_ki * err_ppb;
        double rate_cmd_ppb = gptp->servo_integral_ppb + gptp->cfg.servo_kp * err_ppb;

        /* Clamp servo rate to +/- 500 ppm */
        if (rate_cmd_ppb > 500000.0) rate_cmd_ppb = 500000.0;
        if (rate_cmd_ppb < -500000.0) rate_cmd_ppb = -500000.0;
        gptp->servo_rate_ppb = rate_cmd_ppb;

        /* Apply gentle phase nudge + updated frequency rate */
        int64_t phase_nudge_ps = (int64_t)llround(0.5 * (double)offset_err_ps);
        if (gptp->hal->adjust_clock) {
            gptp->hal->adjust_clock(gptp->hal->ctx, phase_nudge_ps, gptp->servo_rate_ppb);
        }
    }

    if (gptp->hal->emit_metric) {
        tsn_metric_event_t evt;
        memset(&evt, 0, sizeof(evt));
        evt.type = TSN_METRIC_GPTP_SYNC;
        evt.timestamp_ps = gptp->hal->get_time_ps(gptp->hal->ctx);
        evt.value = offset_err_ps;
        gptp->hal->emit_metric(gptp->hal->ctx, &evt);
    }
}

void tsn_gptp_rx_frame(tsn_gptp_node_t *gptp, uint8_t ingress_port,
                       const tsn_frame_t *frame) {
    if (!gptp || !gptp->enabled || !frame || ingress_port >= gptp->cfg.num_ports) {
        return;
    }
    tsn_gptp_port_t *port = &gptp->ports[ingress_port];
    if (port->role == TSN_GPTP_ROLE_DISABLED) {
        return;
    }

    tsn_time_ps_t rx_ts = frame->hw_rx_ts_ps;

    switch (frame->gptp.msg_type) {
        case TSN_GPTP_PDELAY_REQ: {
            tsn_frame_t resp;
            make_gptp_frame(gptp, ingress_port, TSN_GPTP_PDELAY_RESP,
                            frame->gptp.seq_id, &resp);
            resp.gptp.request_receipt_ts_ps = rx_ts;
            gptp_send_frame(gptp, ingress_port, &resp);
            break;
        }
        case TSN_GPTP_PDELAY_RESP: {
            if (frame->gptp.seq_id == port->pdelay_seq_id) {
                port->t2_req_rx_ps = frame->gptp.request_receipt_ts_ps;
                port->t4_resp_rx_ps = rx_ts;
            }
            break;
        }
        case TSN_GPTP_PDELAY_RESP_FUP: {
            if (frame->gptp.seq_id == port->pdelay_seq_id) {
                port->t3_resp_tx_ps = frame->gptp.response_origin_ts_ps;
                if (port->has_prev_pdelay) {
                    int64_t dt_peer = port->t3_resp_tx_ps - port->prev_t3_ps;
                    int64_t dt_local = port->t4_resp_rx_ps - port->prev_t4_ps;
                    if (dt_local > 0 && dt_peer > 0) {
                        port->neighbor_rate_ratio = (double)dt_peer / (double)dt_local;
                    }
                }
                port->prev_t3_ps = port->t3_resp_tx_ps;
                port->prev_t4_ps = port->t4_resp_rx_ps;
                port->has_prev_pdelay = true;

                /* IEEE 802.1AS Eq. 11-1: meanLinkDelay = ((t4 - t1)*r - (t3 - t2)) / 2 */
                double rtt_scaled = (double)(port->t4_resp_rx_ps - port->t1_req_tx_ps) *
                                    port->neighbor_rate_ratio;
                double turnaround = (double)(port->t3_resp_tx_ps - port->t2_req_rx_ps);
                int64_t delay_ps = (int64_t)llround((rtt_scaled - turnaround) * 0.5);
                if (delay_ps >= 0) {
                    port->mean_link_delay_ps = delay_ps;
                    port->pdelay_valid = true;
                }
            }
            break;
        }
        case TSN_GPTP_SYNC: {
            if (port->role == TSN_GPTP_ROLE_SLAVE) {
                port->last_sync_seq = frame->gptp.seq_id;
                port->last_sync_rx_ps = rx_ts;
                port->sync_pending = true;
            }
            break;
        }
        case TSN_GPTP_FOLLOW_UP: {
            if (port->role != TSN_GPTP_ROLE_SLAVE || !port->sync_pending ||
                frame->gptp.seq_id != port->last_sync_seq) {
                break;
            }
            port->sync_pending = false;

            int64_t link_delay = port->pdelay_valid ? port->mean_link_delay_ps : 0;
            tsn_time_ps_t gm_time_at_rx =
                frame->gptp.precise_origin_ts_ps + frame->gptp.correction_field_ps + link_delay;
            int64_t offset_err_ps = gm_time_at_rx - port->last_sync_rx_ps;

            /* Forward Sync to all MASTER ports before stepping clock, passing original rx_ts
             * so residence time is computed in the same local timebase (or after step). */
            tsn_time_ps_t rx_ts_for_fwd = port->last_sync_rx_ps;
            int64_t corr_for_fwd = frame->gptp.correction_field_ps + link_delay;

            gptp_run_servo(gptp, offset_err_ps);

            /* Adjust rx_ts_for_fwd if the first sync stepped the clock */
            if (gptp->sync_rx_count == 1) {
                rx_ts_for_fwd += offset_err_ps;
            } else {
                rx_ts_for_fwd += (int64_t)llround(0.5 * (double)offset_err_ps);
            }

            for (uint8_t p = 0; p < gptp->cfg.num_ports; ++p) {
                if (gptp->ports[p].role != TSN_GPTP_ROLE_MASTER) {
                    continue;
                }
                tsn_frame_t fwd_sync;
                make_gptp_frame(gptp, p, TSN_GPTP_SYNC, frame->gptp.seq_id, &fwd_sync);
                fwd_sync.gptp.precise_origin_ts_ps = frame->gptp.precise_origin_ts_ps;
                fwd_sync.gptp.correction_field_ps = corr_for_fwd;
                fwd_sync.gptp.request_receipt_ts_ps = rx_ts_for_fwd;
                gptp_send_frame(gptp, p, &fwd_sync);
            }
            break;
        }
        default:
            break;
    }
}
