#include "sim/config.hpp"
#include <cassert>
#include <iostream>

static void test_line_basic_scenario() {
    sim::Scenario sc = sim::Scenario::load_from_file("scenarios/line_basic.json");
    sc.run();

    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats.size() == 2);

    /* Stream 1: TT_Control (128B, PCP 6, starts at 100us, period 500us over 10ms)
     * Analytical minimum E2E latency across 3x 1Gbps links + 2 switches:
     *   3 * (128 + 8) * 8 ns = 3 * 1088 ns = 3264 ns
     *   + prop delays (100 + 200 + 100) ns = 400 ns
     *   + 2 * fabric delay (1000 ns)       = 2000 ns
     *   Total analytical min = 5664 ns = 5,664,000 ps.
     */
    const auto& tt = stats[0];
    assert(tt.stream_id == 1);
    assert(tt.tx_frames == 20);
    assert(tt.rx_frames == 20);
    assert(tt.min_latency_ps == 5664 * TSN_PS_PER_NS);
    assert(tt.deadline_misses == 0);

    /* Stream 2: BE_Background */
    const auto& be = stats[1];
    assert(be.stream_id == 2);
    assert(be.tx_frames == 80);
    assert(be.rx_frames >= 78); /* Last burst near 10ms boundary may still be in flight */
}

static void test_link_fault_injection() {
    const std::string cfg = R"({
      "simulation": { "seed": 1, "duration_ms": 5.0 },
      "nodes": [
        { "name": "T", "type": "end_station", "mac": "02:00:00:00:00:01" },
        { "name": "L", "type": "end_station", "mac": "02:00:00:00:00:02" }
      ],
      "links": [
        {
          "name": "L1", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "T", "port": 0 },
          "b": { "node": "L", "port": 0 }
        }
      ],
      "streams": [
        {
          "id": 1, "name": "S1", "talker": "T", "listener": "L",
          "pcp": 6, "frame_bytes": 100, "period_us": 1000.0, "start_us": 100.0
        }
      ],
      "faults": [
        { "link": "L1", "at_ms": 1.5, "action": "down" },
        { "link": "L1", "at_ms": 3.5, "action": "up" }
      ]
    })";

    sim::Scenario sc = sim::Scenario::load_from_json(sim::Json::parse(cfg));
    sc.run();

    /* Frames sent at 0.1, 1.1, 2.1 (dropped), 3.1 (dropped), 4.1 ms -> 5 TX, 3 RX, 2 drops */
    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats.size() == 1);
    assert(stats[0].tx_frames == 5);
    assert(stats[0].rx_frames == 3);
    assert(sc.telemetry().total_link_drops() == 2);
}

int main() {
    test_line_basic_scenario();
    test_link_fault_injection();
    std::cout << "[PASS] test_e2e_phase1\n";
    return 0;
}
