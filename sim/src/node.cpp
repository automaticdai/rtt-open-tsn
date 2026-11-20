#include "sim/node.hpp"
#include "sim/telemetry.hpp"
#include <stdexcept>

namespace sim {

uint64_t EndStation::global_frame_uid_ = 1;

SimNode::SimNode(std::string name, uint32_t node_id, SimKernel& kernel,
                 LocalClock clock, Telemetry* telemetry)
    : name_(std::move(name)),
      node_id_(node_id),
      kernel_(kernel),
      clock_(clock),
      telemetry_(telemetry) {}

void SimNode::attach_link(uint8_t port_id, Link* link) {
    if (port_id >= ports_.size()) {
        ports_.resize(static_cast<size_t>(port_id) + 1u, nullptr);
    }
    ports_[port_id] = link;
}

Link* SimNode::get_link(uint8_t port_id) const {
    if (port_id >= ports_.size()) {
        return nullptr;
    }
    return ports_[port_id];
}

tsn_hal_t SimNode::make_hal() {
    tsn_hal_t hal{};
    hal.ctx = this;
    hal.get_time_ps = &SimNode::hal_get_time_ps;
    hal.adjust_clock = &SimNode::hal_adjust_clock;
    hal.schedule_timer = &SimNode::hal_schedule_timer;
    hal.cancel_timer = &SimNode::hal_cancel_timer;
    hal.tx_frame = &SimNode::hal_tx_frame;
    hal.preempt_tx = &SimNode::hal_preempt_tx;
    hal.emit_metric = &SimNode::hal_emit_metric;
    return hal;
}

tsn_time_ps_t SimNode::hal_get_time_ps(void* ctx) {
    auto* self = static_cast<SimNode*>(ctx);
    return self->clock_.sim_to_local(self->kernel_.now_ps());
}

void SimNode::hal_adjust_clock(void* ctx, int64_t offset_ps, double rate_ppb) {
    auto* self = static_cast<SimNode*>(ctx);
    self->clock_.adjust(self->kernel_.now_ps(), offset_ps, rate_ppb);
}

uint64_t SimNode::hal_schedule_timer(void* ctx, tsn_time_ps_t target_local_ps,
                                     tsn_timer_cb_t cb, void* arg) {
    auto* self = static_cast<SimNode*>(ctx);
    tsn_time_ps_t target_sim_ps = self->clock_.local_to_sim(target_local_ps);
    return self->kernel_.schedule_at(target_sim_ps, [cb, arg]() {
        if (cb) cb(arg);
    });
}

void SimNode::hal_cancel_timer(void* ctx, uint64_t timer_id) {
    auto* self = static_cast<SimNode*>(ctx);
    self->kernel_.cancel(timer_id);
}

tsn_status_t SimNode::hal_tx_frame(void* ctx, uint8_t port_id, const tsn_frame_t* frame) {
    auto* self = static_cast<SimNode*>(ctx);
    Link* link = self->get_link(port_id);
    if (!link || !frame) {
        return TSN_ERR_INVALID_ARG;
    }
    return link->transmit(self, port_id, *frame, [self, port_id]() {
        self->on_tx_complete(port_id);
    });
}

tsn_status_t SimNode::hal_preempt_tx(void* ctx, uint8_t port_id, uint16_t cut_bytes) {
    auto* self = static_cast<SimNode*>(ctx);
    Link* link = self->get_link(port_id);
    if (!link) {
        return TSN_ERR_INVALID_ARG;
    }
    return link->preempt_tx(self, port_id, cut_bytes);
}

void SimNode::hal_emit_metric(void* ctx, const tsn_metric_event_t* evt) {
    auto* self = static_cast<SimNode*>(ctx);
    if (!self || !self->telemetry_ || !evt) {
        return;
    }
    if (evt->type == TSN_METRIC_GPTP_SYNC) {
        self->telemetry_->record_gptp_sync(self->node_id_, evt->value);
    } else if (evt->type == TSN_METRIC_QBV_GATE_CHANGE) {
        self->telemetry_->record_qbv_gate_change(
            self->node_id_, evt->port_id, self->kernel_.now_ps(),
            static_cast<uint8_t>(evt->value & 0xFF));
    }
}

/* ------------------------------------------------------------------------- */
/* EndStation                                                                */
/* ------------------------------------------------------------------------- */

EndStation::EndStation(std::string name, uint32_t node_id, tsn_mac_t mac,
                       SimKernel& kernel, LocalClock clock, Telemetry* telemetry)
    : SimNode(std::move(name), node_id, kernel, clock, telemetry), mac_(mac) {
    tsn_queue_init(&gptp_queue_, 16u);
    tsn_qbu_init(&rx_qbu_);
    for (uint8_t tc = 0; tc < TSN_NUM_TRAFFIC_CLASSES; ++tc) {
        tsn_queue_init(&tx_queues_[tc], TSN_QUEUE_CAPACITY);
    }
}

tsn_status_t EndStation::es_gptp_tx_adapter(void* ctx, uint8_t port_id,
                                            const tsn_frame_t* frame) {
    (void)port_id;
    auto* self = static_cast<EndStation*>(ctx);
    tsn_status_t st = tsn_queue_enqueue(&self->gptp_queue_, frame);
    if (st == TSN_OK) {
        self->try_tx();
    }
    return st;
}

void EndStation::init_gptp(const tsn_gptp_config_t& cfg, tsn_gptp_port_role_t port0_role) {
    gptp_hal_ = make_hal();
    gptp_hal_.tx_frame = &EndStation::es_gptp_tx_adapter;
    tsn_gptp_init(&gptp_, &cfg, &gptp_hal_, &mac_);
    tsn_gptp_set_port_role(&gptp_, 0, port0_role);
}

void EndStation::start_gptp() {
    if (gptp_.enabled) {
        tsn_gptp_start(&gptp_);
    }
}

void EndStation::add_talker_stream(const StreamSpec& spec) {
    talker_streams_.push_back(spec);
    stream_seq_.push_back(0);
}

void EndStation::start_talkers(tsn_time_ps_t sim_end_ps) {
    for (size_t i = 0; i < talker_streams_.size(); ++i) {
        tsn_time_ps_t first_sim_ps = clock_.local_to_sim(talker_streams_[i].start_time_ps);
        if (first_sim_ps < sim_end_ps) {
            kernel_.schedule_at(first_sim_ps, [this, i, sim_end_ps]() {
                schedule_next_burst(i, sim_end_ps);
            });
        }
    }
}

void EndStation::start_single_talker(const StreamSpec& spec, tsn_time_ps_t sim_end_ps) {
    size_t idx = talker_streams_.size();
    talker_streams_.push_back(spec);
    stream_seq_.push_back(0);
    tsn_time_ps_t first_sim_ps = clock_.local_to_sim(spec.start_time_ps);
    if (first_sim_ps < kernel_.now_ps()) {
        first_sim_ps = kernel_.now_ps();
    }
    if (first_sim_ps < sim_end_ps) {
        kernel_.schedule_at(first_sim_ps, [this, idx, sim_end_ps]() {
            schedule_next_burst(idx, sim_end_ps);
        });
    }
}

void EndStation::schedule_next_burst(size_t stream_idx, tsn_time_ps_t sim_end_ps) {
    const StreamSpec& spec = talker_streams_[stream_idx];
    tsn_time_ps_t now_sim = kernel_.now_ps();
    if (now_sim >= sim_end_ps) {
        return;
    }

    uint32_t burst = spec.burst_count == 0 ? 1u : spec.burst_count;
    for (uint32_t b = 0; b < burst; ++b) {
        tsn_frame_t f;
        tsn_frame_init(&f);
        f.src_mac = mac_;
        f.dst_mac = spec.dst_mac;
        f.has_vlan = true;
        f.vid = spec.vid;
        f.pcp = spec.pcp & 0x07u;
        f.length_bytes = spec.frame_bytes;
        f.stream_id = spec.stream_id;
        f.frame_uid = global_frame_uid_++;
        f.seq_in_stream = stream_seq_[stream_idx]++;
        f.gen_time_ps = now_sim;
        f.deadline_ps = spec.deadline_ps;

        if (telemetry_) {
            telemetry_->record_tx(spec.stream_id);
        }
        tsn_queue_enqueue(&tx_queues_[f.pcp], &f);
    }

    try_tx();

    if (spec.period_ps > 0) {
        tsn_time_ps_t next_local = clock_.sim_to_local(now_sim) + spec.period_ps;
        tsn_time_ps_t next_sim = clock_.local_to_sim(next_local);
        if (next_sim < sim_end_ps) {
            kernel_.schedule_at(next_sim, [this, stream_idx, sim_end_ps]() {
                schedule_next_burst(stream_idx, sim_end_ps);
            });
        }
    }
}

void EndStation::try_tx() {
    if (tx_busy_) {
        return;
    }
    Link* link = get_link(0);
    if (!link) {
        return;
    }

    /* 1. Prioritize 802.1AS gPTP control frames */
    if (!tsn_queue_is_empty(&gptp_queue_)) {
        tsn_frame_t frame;
        if (tsn_queue_dequeue(&gptp_queue_, &frame) == TSN_OK) {
            tx_busy_ = true;
            tsn_status_t st = link->transmit(this, 0, frame, [this]() {
                on_tx_complete(0);
            });
            if (st != TSN_OK) {
                tx_busy_ = false;
            }
        }
        return;
    }

    for (int tc = static_cast<int>(TSN_NUM_TRAFFIC_CLASSES) - 1; tc >= 0; --tc) {
        if (!tsn_queue_is_empty(&tx_queues_[tc])) {
            tsn_frame_t frame;
            if (tsn_queue_dequeue(&tx_queues_[tc], &frame) == TSN_OK) {
                tx_busy_ = true;
                tsn_status_t st = link->transmit(this, 0, frame, [this]() {
                    on_tx_complete(0);
                });
                if (st != TSN_OK) {
                    tx_busy_ = false;
                }
            }
            return;
        }
    }
}

void EndStation::on_rx_frame(uint8_t ingress_port, const tsn_frame_t& frame) {
    tsn_frame_t reassembled;
    if (!tsn_qbu_rx_mpacket(&rx_qbu_, &frame, &reassembled)) {
        return;
    }
    if (reassembled.ethertype == TSN_ETHERTYPE_GPTP) {
        if (gptp_.enabled) {
            tsn_gptp_rx_frame(&gptp_, ingress_port, &reassembled);
        }
        return;
    }
    if (tsn_mac_equal(&reassembled.dst_mac, &mac_) ||
        tsn_mac_is_broadcast(&reassembled.dst_mac)) {
        if (telemetry_) {
            telemetry_->record_rx(reassembled, kernel_.now_ps());
        }
    }
}

void EndStation::on_tx_complete(uint8_t egress_port) {
    (void)egress_port;
    tx_busy_ = false;
    try_tx();
}

void EndStation::on_tx_timestamp(uint8_t egress_port, const tsn_frame_t& frame,
                                 tsn_time_ps_t tx_ts_ps) {
    if (frame.ethertype == TSN_ETHERTYPE_GPTP && gptp_.enabled) {
        tsn_gptp_on_tx_timestamp(&gptp_, egress_port, &frame, tx_ts_ps);
    }
}

/* ------------------------------------------------------------------------- */
/* SwitchNode                                                                */
/* ------------------------------------------------------------------------- */

SwitchNode::SwitchNode(std::string name, uint32_t node_id, uint8_t num_ports,
                       tsn_time_ps_t fabric_delay_ps, uint16_t queue_capacity,
                       SimKernel& kernel, LocalClock clock, Telemetry* telemetry)
    : SimNode(std::move(name), node_id, kernel, clock, telemetry),
      fabric_delay_ps_(fabric_delay_ps),
      queue_capacity_(queue_capacity) {
    tsn_hal_t hal = make_hal();
    if (tsn_bridge_init(&bridge_, num_ports, &hal) != TSN_OK) {
        throw std::runtime_error("Failed to initialize tsn_bridge_t for switch " + name_);
    }
}

void SwitchNode::config_port(uint8_t port_id, uint64_t link_rate_bps) {
    tsn_bridge_config_port(&bridge_, port_id, link_rate_bps, queue_capacity_);
}

void SwitchNode::add_fdb_entry(const tsn_mac_t& dst_mac, uint16_t vid, uint32_t port_mask) {
    tsn_bridge_add_fdb(&bridge_, &dst_mac, vid, port_mask);
}

void SwitchNode::config_qbv(uint8_t port_id, const tsn_qbv_config_t& admin,
                            bool guard_band_enabled) {
    if (tsn_bridge_config_qbv(&bridge_, port_id, &admin, guard_band_enabled) != TSN_OK) {
        throw std::runtime_error("Failed to configure Qbv on switch " + name_);
    }
}

void SwitchNode::config_qav(uint8_t port_id, uint8_t tc, double idle_slope_bps,
                            double hi_credit_bits, double lo_credit_bits) {
    if (tsn_bridge_config_qav(&bridge_, port_id, tc, idle_slope_bps,
                              hi_credit_bits, lo_credit_bits) != TSN_OK) {
        throw std::runtime_error("Failed to configure Qav on switch " + name_);
    }
}

void SwitchNode::config_qbu(uint8_t port_id, uint8_t preemptable_mask, uint8_t add_frag_size) {
    if (tsn_bridge_config_qbu(&bridge_, port_id, preemptable_mask, add_frag_size) != TSN_OK) {
        throw std::runtime_error("Failed to configure Qbu on switch " + name_);
    }
}

void SwitchNode::add_frer_gen(const tsn_mac_t& dst_mac, uint16_t vid, uint32_t split_port_mask) {
    if (tsn_bridge_add_frer_gen(&bridge_, &dst_mac, vid, split_port_mask) != TSN_OK) {
        throw std::runtime_error("Failed to add FRER gen rule on switch " + name_);
    }
}

void SwitchNode::add_frer_recov(const tsn_mac_t& dst_mac, uint16_t vid, tsn_frer_algo_t algo,
                                uint8_t history_length, bool strip_rtag,
                                tsn_time_ps_t reset_timeout_ps) {
    if (tsn_bridge_add_frer_recov(&bridge_, &dst_mac, vid, algo, history_length,
                                  strip_rtag, reset_timeout_ps) != TSN_OK) {
        throw std::runtime_error("Failed to add FRER recov rule on switch " + name_);
    }
}

void SwitchNode::init_gptp(const tsn_gptp_config_t& cfg, const tsn_mac_t& bridge_mac) {
    tsn_bridge_init_gptp(&bridge_, &cfg, &bridge_mac);
}

void SwitchNode::set_gptp_port_role(uint8_t port_id, tsn_gptp_port_role_t role) {
    tsn_gptp_set_port_role(&bridge_.gptp, port_id, role);
}

void SwitchNode::start_gptp() {
    if (bridge_.gptp.enabled) {
        tsn_gptp_start(&bridge_.gptp);
    }
}

void SwitchNode::on_rx_frame(uint8_t ingress_port, const tsn_frame_t& frame) {
    tsn_frame_t copy = frame;
    copy.ingress_time_ps = kernel_.now_ps();

    if (copy.ethertype == TSN_ETHERTYPE_GPTP || fabric_delay_ps_ <= 0) {
        tsn_bridge_rx_frame(&bridge_, ingress_port, &copy);
    } else {
        kernel_.schedule_after(fabric_delay_ps_, [this, ingress_port, copy]() {
            tsn_bridge_rx_frame(&bridge_, ingress_port, &copy);
        });
    }
}

void SwitchNode::on_tx_complete(uint8_t egress_port) {
    tsn_bridge_on_tx_complete(&bridge_, egress_port);
}

void SwitchNode::on_tx_timestamp(uint8_t egress_port, const tsn_frame_t& frame,
                                 tsn_time_ps_t tx_ts_ps) {
    tsn_bridge_on_tx_timestamp(&bridge_, egress_port, &frame, tx_ts_ps);
}

} // namespace sim
