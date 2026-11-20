#include "sim/link.hpp"
#include "sim/node.hpp"
#include "sim/telemetry.hpp"
#include <cmath>

namespace sim {

Link::Link(SimKernel& kernel, LinkConfig cfg, Endpoint a, Endpoint b, Telemetry* telemetry)
    : kernel_(kernel), cfg_(std::move(cfg)), ep_a_(a), ep_b_(b), telemetry_(telemetry) {}

void Link::set_up(bool up) {
    is_up_ = up;
}

bool Link::should_drop(DirectionState& dir, uint16_t frame_bytes) {
    if (!is_up_) {
        return true;
    }
    if (cfg_.drop_every_n > 0 && (dir.tx_count % cfg_.drop_every_n) == 0) {
        return true;
    }
    if (cfg_.ber > 0.0) {
        double bits = static_cast<double>(frame_bytes) * 8.0;
        double frame_error_prob = 1.0 - std::pow(1.0 - cfg_.ber, bits);
        if (kernel_.uniform_01() < frame_error_prob) {
            return true;
        }
    }
    return false;
}

tsn_status_t Link::transmit(SimNode* src_node, uint8_t src_port, const tsn_frame_t& frame,
                            std::function<void()> tx_done_cb) {
    DirectionState* dir = nullptr;
    Endpoint dst{};

    if (ep_a_.node == src_node && ep_a_.port_id == src_port) {
        dir = &dir_a_to_b_;
        dst = ep_b_;
    } else if (ep_b_.node == src_node && ep_b_.port_id == src_port) {
        dir = &dir_b_to_a_;
        dst = ep_a_;
    } else {
        return TSN_ERR_INVALID_ARG;
    }

    if (dir->tx_busy) {
        return TSN_ERR_BUSY;
    }

    dir->tx_busy = true;
    dir->tx_count++;

    tsn_time_ps_t start_ps = kernel_.now_ps();
    tsn_time_ps_t wire_dur_ps = tsn_frame_tx_time_ps(frame.length_bytes, cfg_.rate_bps, false);
    tsn_time_ps_t busy_dur_ps = tsn_frame_tx_time_ps(frame.length_bytes, cfg_.rate_bps, true);

    tsn_time_ps_t tx_local_ps = src_node->clock().sim_to_local(start_ps);
    tsn_frame_t tx_frame = frame;
    tx_frame.hw_tx_ts_ps = tx_local_ps;

    dir->active_start_ps = start_ps;
    dir->active_frame = tx_frame;
    dir->active_dst = dst;
    dir->active_tx_done_cb = tx_done_cb;
    dir->active_trace_idx = static_cast<size_t>(-1);
    dir->rx_event_id = 0;
    dir->tx_done_event_id = 0;

    if (telemetry_ && tx_frame.ethertype != TSN_ETHERTYPE_GPTP) {
        dir->active_trace_idx = telemetry_->record_tx_span(
            src_node->node_id(), src_port, tx_frame, start_ps, wire_dur_ps);
    }

    /* Notify sender of hardware TX timestamp (used by 802.1AS Follow_Up / Pdelay) */
    src_node->on_tx_timestamp(src_port, tx_frame, tx_local_ps);

    bool dropped = should_drop(*dir, tx_frame.length_bytes);
    dir->active_dropped = dropped;
    if (dropped) {
        dir->dropped_count++;
        if (telemetry_) {
            telemetry_->record_link_drop(cfg_.name);
        }
    } else if (dst.node != nullptr) {
        /* Store-and-forward arrival at receiver occurs after wire serialization + propagation */
        tsn_time_ps_t rx_delay_ps = wire_dur_ps + cfg_.propagation_delay_ps;
        tsn_frame_t rx_frame = tx_frame;
        dir->rx_event_id = kernel_.schedule_after(rx_delay_ps, [this, dst, rx_frame]() mutable {
            rx_frame.hw_rx_ts_ps = dst.node->clock().sim_to_local(kernel_.now_ps());
            dst.node->on_rx_frame(dst.port_id, rx_frame);
        });
    }

    /* Sender port becomes free after serialization + IFG */
    dir->tx_done_event_id = kernel_.schedule_after(busy_dur_ps, [dir]() {
        dir->tx_busy = false;
        dir->rx_event_id = 0;
        dir->tx_done_event_id = 0;
        auto cb = std::move(dir->active_tx_done_cb);
        if (cb) {
            cb();
        }
    });

    return TSN_OK;
}

tsn_status_t Link::preempt_tx(SimNode* src_node, uint8_t src_port, uint16_t cut_bytes) {
    DirectionState* dir = nullptr;
    if (ep_a_.node == src_node && ep_a_.port_id == src_port) {
        dir = &dir_a_to_b_;
    } else if (ep_b_.node == src_node && ep_b_.port_id == src_port) {
        dir = &dir_b_to_a_;
    } else {
        return TSN_ERR_INVALID_ARG;
    }

    if (!dir->tx_busy) {
        return TSN_ERR_EMPTY;
    }

    /* Cancel original full-frame RX and TX-complete events */
    kernel_.cancel(dir->rx_event_id);
    kernel_.cancel(dir->tx_done_event_id);
    dir->rx_event_id = 0;
    dir->tx_done_event_id = 0;

    /* Truncate in-flight mPacket to cut_bytes + 4B mCRC */
    uint16_t wire_len = static_cast<uint16_t>(cut_bytes + 4u);
    dir->active_frame.is_final_frag = false;
    dir->active_frame.frag_len_bytes = cut_bytes;
    dir->active_frame.length_bytes = wire_len;

    tsn_time_ps_t wire_dur_ps = tsn_frame_tx_time_ps(wire_len, cfg_.rate_bps, false);
    tsn_time_ps_t busy_dur_ps = tsn_frame_tx_time_ps(wire_len, cfg_.rate_bps, true);

    if (telemetry_ && dir->active_trace_idx != static_cast<size_t>(-1)) {
        telemetry_->update_tx_span_preempted(dir->active_trace_idx, wire_dur_ps);
    }

    if (!dir->active_dropped && dir->active_dst.node != nullptr) {
        tsn_time_ps_t rx_at_ps = dir->active_start_ps + wire_dur_ps + cfg_.propagation_delay_ps;
        Endpoint dst = dir->active_dst;
        tsn_frame_t rx_frame = dir->active_frame;
        dir->rx_event_id = kernel_.schedule_at(rx_at_ps, [this, dst, rx_frame]() mutable {
            rx_frame.hw_rx_ts_ps = dst.node->clock().sim_to_local(kernel_.now_ps());
            dst.node->on_rx_frame(dst.port_id, rx_frame);
        });
    }

    tsn_time_ps_t done_at_ps = dir->active_start_ps + busy_dur_ps;
    dir->tx_done_event_id = kernel_.schedule_at(done_at_ps, [dir]() {
        dir->tx_busy = false;
        dir->rx_event_id = 0;
        dir->tx_done_event_id = 0;
        auto cb = std::move(dir->active_tx_done_cb);
        if (cb) {
            cb();
        }
    });

    return TSN_OK;
}

} // namespace sim
