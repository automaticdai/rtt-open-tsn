#pragma once

#include "sim/kernel.hpp"
#include "tsn/frame.h"
#include <cstdint>
#include <functional>
#include <string>

namespace sim {

class SimNode;
class Telemetry;

struct LinkConfig {
    std::string   name;
    uint64_t      rate_bps = 1000000000ULL; /* 1 Gbps */
    tsn_time_ps_t propagation_delay_ps = 100 * TSN_PS_PER_NS; /* 100 ns (~20m) */
    double        ber = 0.0; /* Bit error rate (0 = lossless) */
    uint32_t      drop_every_n = 0; /* Deterministic drop: 0 = disabled */
};

/**
 * Full-duplex point-to-point Ethernet link connecting two (SimNode, port_id) endpoints.
 */
class Link {
public:
    struct Endpoint {
        SimNode* node = nullptr;
        uint8_t  port_id = 0;
    };

    Link(SimKernel& kernel, LinkConfig cfg, Endpoint a, Endpoint b, Telemetry* telemetry = nullptr);

    const LinkConfig& config() const { return cfg_; }
    bool is_up() const { return is_up_; }

    /** Schedule a link down or link up fault event. */
    void set_up(bool up);

    /**
     * Transmit a frame from (src_node, src_port) to the opposite endpoint.
     * Calls tx_done_cb when serialization (+ IFG) completes on the transmitting port.
     */
    tsn_status_t transmit(SimNode* src_node, uint8_t src_port, const tsn_frame_t& frame,
                          std::function<void()> tx_done_cb);

    /**
     * Truncate the currently active pMAC transmission from (src_node, src_port) at
     * cut_bytes (+ 4B mCRC + IFG), rescheduling its RX and TX-complete events.
     */
    tsn_status_t preempt_tx(SimNode* src_node, uint8_t src_port, uint16_t cut_bytes);

private:
    struct DirectionState {
        bool                  tx_busy = false;
        uint64_t              tx_count = 0;
        uint64_t              dropped_count = 0;
        EventId               rx_event_id = 0;
        EventId               tx_done_event_id = 0;
        tsn_time_ps_t         active_start_ps = 0;
        tsn_frame_t           active_frame{};
        bool                  active_dropped = false;
        Endpoint              active_dst{};
        std::function<void()> active_tx_done_cb;
        size_t                active_trace_idx = static_cast<size_t>(-1);
    };

    SimKernel&     kernel_;
    LinkConfig     cfg_;
    Endpoint       ep_a_;
    Endpoint       ep_b_;
    Telemetry*     telemetry_ = nullptr;
    bool           is_up_ = true;
    DirectionState dir_a_to_b_;
    DirectionState dir_b_to_a_;

    bool should_drop(DirectionState& dir, uint16_t frame_bytes);
};

} // namespace sim
