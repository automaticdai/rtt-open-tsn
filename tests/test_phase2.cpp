#include "sim/config.hpp"
#include "tsn/qav.h"
#include "tsn/qbv.h"
#include <cassert>
#include <cmath>
#include <iostream>

static void test_qav_credit_and_qbv_freeze() {
    tsn_qav_queue_t q;
    tsn_qav_init(&q);
    /* 200 Mbps idleSlope on 1 Gbps port -> sendSlope = -800 Mbps */
    assert(tsn_qav_config(&q, 200e6, 1000000000ULL, 0.0, 0.0) == TSN_OK);

    tsn_qav_on_enqueue(&q, 0);
    assert(tsn_qav_can_transmit(&q, 0));

    /* Transmit for 10 us -> credit drops by -800 Mbps * 10 us = -8000 bits */
    tsn_qav_on_tx_start(&q, 0);
    tsn_qav_on_tx_end(&q, 10 * TSN_PS_PER_US, true);
    assert(std::fabs(q.credit_bits - (-8000.0)) < 1e-6);
    assert(!tsn_qav_can_transmit(&q, 10 * TSN_PS_PER_US));

    /* Recovery time at +200 Mbps for 8000 bits = 40 us */
    tsn_time_ps_t wait_ps = tsn_qav_time_until_ready_ps(&q, 10 * TSN_PS_PER_US);
    assert(wait_ps == 40 * TSN_PS_PER_US);

    /* Close Qbv gate for 20 us -> credit MUST remain frozen at -8000 bits! */
    tsn_qav_on_gate_change(&q, 10 * TSN_PS_PER_US, false);
    tsn_qav_update(&q, 30 * TSN_PS_PER_US);
    assert(std::fabs(q.credit_bits - (-8000.0)) < 1e-6);

    /* Re-open gate at 30 us; after 40 us (at t=70 us), credit reaches 0 */
    tsn_qav_on_gate_change(&q, 30 * TSN_PS_PER_US, true);
    assert(!tsn_qav_can_transmit(&q, 69 * TSN_PS_PER_US));
    assert(tsn_qav_can_transmit(&q, 70 * TSN_PS_PER_US));
}

static void test_qbv_guard_band_protects_tt() {
    /* Verify that a 1500B BE frame arriving 5 us before a TT window is held by
     * the Qbv guard band so it does NOT delay the TT frame at t = 500 us. */
    const std::string cfg = R"({
      "simulation": { "seed": 1, "duration_ms": 2.0 },
      "nodes": [
        { "name": "T_TT", "type": "end_station", "mac": "02:00:00:00:00:01" },
        { "name": "T_BE", "type": "end_station", "mac": "02:00:00:00:00:03" },
        { "name": "SW1",  "type": "switch", "ports": 4, "fabric_delay_ns": 1000 },
        { "name": "L",    "type": "end_station", "mac": "02:00:00:00:00:02" }
      ],
      "links": [
        { "name": "L1", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "T_TT", "port": 0 }, "b": { "node": "SW1", "port": 0 } },
        { "name": "L2", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "T_BE", "port": 0 }, "b": { "node": "SW1", "port": 1 } },
        { "name": "L3", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "SW1", "port": 2 }, "b": { "node": "L", "port": 0 } }
      ],
      "fdb": [
        { "switch": "SW1", "dst_mac": "02:00:00:00:00:02", "vid": 1, "egress_ports": [2] }
      ],
      "qbv": [
        {
          "switch": "SW1", "port": 2, "base_time_us": 0.0, "cycle_time_us": 500.0,
          "guard_band": true,
          "gcl": [
            { "gate_mask": "0b01000000", "duration_us": 50.0 },
            { "gate_mask": "0b10111111", "duration_us": 450.0 }
          ]
        }
      ],
      "streams": [
        {
          "id": 1, "name": "TT", "talker": "T_TT", "listener": "L",
          "vid": 1, "pcp": 6, "frame_bytes": 128, "period_us": 500.0, "start_us": 500.0
        },
        {
          "id": 2, "name": "BE", "talker": "T_BE", "listener": "L",
          "vid": 1, "pcp": 0, "frame_bytes": 1500, "period_us": 500.0, "start_us": 480.0
        }
      ]
    })";

    sim::Scenario sc = sim::Scenario::load_from_json(sim::Json::parse(cfg));
    sc.run();

    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats.size() == 2);
    /* TT has 0 jitter and exact minimal 2-hop latency (no BE interference!):
     * 2 * 1088 ns + 2 * 100 ns + 1000 ns = 3376 ns */
    const auto& tt = stats[0];
    assert(tt.tx_frames == 3 && tt.rx_frames == 3);
    assert(tt.max_latency_ps == 3376 * TSN_PS_PER_NS);
    assert(tt.jitter_ps == 0);
}

static void test_phase2_full_scenario() {
    sim::Scenario sc = sim::Scenario::load_from_file("scenarios/phase2_tsn_shapers.json");
    sc.run();

    /* 1. Verify 802.1AS gPTP convergence on all 3 slave nodes despite +/-20 ppm drift */
    auto gptp = sc.telemetry().compute_gptp_stats();
    assert(gptp.size() == 3);
    for (const auto& g : gptp) {
        assert(g.sync_count >= 15);
        /* Final steady-state sync error must be well under 50 ns */
        int64_t abs_last = g.last_err_ps < 0 ? -g.last_err_ps : g.last_err_ps;
        assert(abs_last < 50 * TSN_PS_PER_NS);
    }

    /* 2. Verify streams delivered and TT_Control meets its 50 us deadline */
    auto streams = sc.telemetry().compute_stream_stats();
    assert(streams.size() == 3);
    const auto& tt = streams[0];
    assert(tt.stream_name == "TT_Control");
    assert(tt.rx_frames == tt.tx_frames);
    assert(tt.deadline_misses == 0);

    const auto& avb = streams[1];
    assert(avb.stream_name == "AVB_ClassA");
    assert(avb.rx_frames == avb.tx_frames);
    assert(avb.deadline_misses == 0);
    /* Because of 100 Mbps CBS pacing on 3-frame burst of 512B frames, AVB max latency
     * is noticeably higher than min latency (proving CBS pacing is active!) */
    assert(avb.max_latency_ps > avb.min_latency_ps + 30 * TSN_PS_PER_US);
}

int main() {
    test_qav_credit_and_qbv_freeze();
    test_qbv_guard_band_protects_tt();
    test_phase2_full_scenario();
    std::cout << "[PASS] test_phase2\n";
    return 0;
}
