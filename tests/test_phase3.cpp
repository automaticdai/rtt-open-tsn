#include "sim/config.hpp"
#include "tsn/qbu.h"
#include <cassert>
#include <iostream>

static void test_qbu_fragmentation_and_reassembly() {
    tsn_qbu_port_t rx;
    tsn_qbu_init(&rx);

    /* 1. Receive SMD-S initial fragment (300 bytes of a 1500B frame) */
    tsn_frame_t frag1;
    tsn_frame_init(&frag1);
    frag1.smd_type = TSN_SMD_S;
    frag1.is_final_frag = false;
    frag1.frag_len_bytes = 300;
    frag1.length_bytes = 304;
    frag1.pcp = 0;

    tsn_frame_t out;
    assert(!tsn_qbu_rx_mpacket(&rx, &frag1, &out));
    assert(rx.rx.assembling);

    /* 2. Express frame (SMD-E) overtakes mid-reassembly without disturbing pMAC state */
    tsn_frame_t exp;
    tsn_frame_init(&exp);
    exp.smd_type = TSN_SMD_E;
    exp.length_bytes = 128;
    exp.pcp = 6;
    assert(tsn_qbu_rx_mpacket(&rx, &exp, &out));
    assert(out.pcp == 6 && out.length_bytes == 128);
    assert(rx.rx.assembling);

    /* 3. Receive SMD-C0 continuation (500 bytes, non-final) */
    tsn_frame_t frag2;
    tsn_frame_init(&frag2);
    frag2.smd_type = TSN_SMD_C;
    frag2.frag_count = 0;
    frag2.is_final_frag = false;
    frag2.frag_len_bytes = 500;
    assert(!tsn_qbu_rx_mpacket(&rx, &frag2, &out));

    /* 4. Receive SMD-C1 final continuation (700 bytes) -> reassembled 1500B frame! */
    tsn_frame_t frag3;
    tsn_frame_init(&frag3);
    frag3.smd_type = TSN_SMD_C;
    frag3.frag_count = 1;
    frag3.is_final_frag = true;
    frag3.frag_len_bytes = 700;
    assert(tsn_qbu_rx_mpacket(&rx, &frag3, &out));
    assert(out.length_bytes == 1500);
    assert(out.pcp == 0);
    assert(!rx.rx.assembling);
    assert(rx.reassembled_frames == 1);
    assert(rx.reassembly_errors == 0);
}

static void test_qbu_reduces_express_interference() {
    auto make_cfg = [](bool enable_qbu) {
        std::string qbu_block = enable_qbu
            ? R"("qbu": [{"switch": "SW1", "port": 2, "preemptable_mask": "0b00000001", "add_frag_size": 0}],)"
            : "";
        return R"({
          "simulation": { "seed": 1, "duration_ms": 1.0 },
          "nodes": [
            { "name": "T_E", "type": "end_station", "mac": "02:00:00:00:00:01" },
            { "name": "T_P", "type": "end_station", "mac": "02:00:00:00:00:03" },
            { "name": "SW1", "type": "switch", "ports": 4, "fabric_delay_ns": 1000 },
            { "name": "L",   "type": "end_station", "mac": "02:00:00:00:00:02" }
          ],
          "links": [
            { "name": "L1", "rate_mbps": 1000, "propagation_delay_ns": 100,
              "a": { "node": "T_E", "port": 0 }, "b": { "node": "SW1", "port": 0 } },
            { "name": "L2", "rate_mbps": 1000, "propagation_delay_ns": 100,
              "a": { "node": "T_P", "port": 0 }, "b": { "node": "SW1", "port": 1 } },
            { "name": "L3", "rate_mbps": 1000, "propagation_delay_ns": 100,
              "a": { "node": "SW1", "port": 2 }, "b": { "node": "L", "port": 0 } }
          ],
          "fdb": [
            { "switch": "SW1", "dst_mac": "02:00:00:00:00:02", "vid": 1, "egress_ports": [2] }
          ],
          )" + qbu_block + R"(
          "streams": [
            {
              "id": 1, "name": "Express", "talker": "T_E", "listener": "L",
              "vid": 1, "pcp": 6, "frame_bytes": 128, "period_us": 200.0, "start_us": 13.0
            },
            {
              "id": 2, "name": "Preemptable", "talker": "T_P", "listener": "L",
              "vid": 1, "pcp": 0, "frame_bytes": 1500, "period_us": 200.0, "start_us": 0.0
            }
          ]
        })";
    };

    /* Run WITHOUT preemption */
    sim::Scenario sc_no = sim::Scenario::load_from_json(sim::Json::parse(make_cfg(false)));
    sc_no.run();
    auto stats_no = sc_no.telemetry().compute_stream_stats();

    /* Run WITH preemption */
    sim::Scenario sc_yes = sim::Scenario::load_from_json(sim::Json::parse(make_cfg(true)));
    sc_yes.run();
    auto stats_yes = sc_yes.telemetry().compute_stream_stats();

    auto* sw1 = dynamic_cast<sim::SwitchNode*>(sc_yes.find_node("SW1"));
    assert(sw1 != nullptr);
    assert(sw1->bridge().ports[2].qbu.preemptions_count == 5);

    /* Without preemption, Express waits ~10 us behind 1500B BE frame (> 13 us E2E).
     * With preemption, SW1 cuts the BE frame and sends Express in < 4.5 us! */
    assert(stats_no[0].max_latency_ps > 12 * TSN_PS_PER_US);
    assert(stats_yes[0].max_latency_ps < 5 * TSN_PS_PER_US);

    /* All 5 Preemptable 1500B frames must be cleanly reassembled with exact 1500B length */
    assert(stats_yes[1].rx_frames == 5);
    for (const auto& rec : sc_yes.telemetry().frame_records()) {
        if (rec.stream_id == 2) {
            assert(rec.length_bytes == 1500);
        }
    }
}

static void test_qbv_hold_release_preemption() {
    /* Qbv SetAndHoldMAC at t=500us preempts a 1500B pMAC frame that started at ~497us */
    const std::string cfg = R"({
      "simulation": { "seed": 1, "duration_ms": 2.0 },
      "nodes": [
        { "name": "T_E", "type": "end_station", "mac": "02:00:00:00:00:01" },
        { "name": "T_P", "type": "end_station", "mac": "02:00:00:00:00:03" },
        { "name": "SW1", "type": "switch", "ports": 4, "fabric_delay_ns": 1000 },
        { "name": "L",   "type": "end_station", "mac": "02:00:00:00:00:02" }
      ],
      "links": [
        { "name": "L1", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "T_E", "port": 0 }, "b": { "node": "SW1", "port": 0 } },
        { "name": "L2", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "T_P", "port": 0 }, "b": { "node": "SW1", "port": 1 } },
        { "name": "L3", "rate_mbps": 1000, "propagation_delay_ns": 100,
          "a": { "node": "SW1", "port": 2 }, "b": { "node": "L", "port": 0 } }
      ],
      "fdb": [
        { "switch": "SW1", "dst_mac": "02:00:00:00:00:02", "vid": 1, "egress_ports": [2] }
      ],
      "qbu": [
        { "switch": "SW1", "port": 2, "preemptable_mask": "0b00000001", "add_frag_size": 0 }
      ],
      "qbv": [
        {
          "switch": "SW1", "port": 2, "base_time_us": 0.0, "cycle_time_us": 500.0,
          "guard_band": true,
          "gcl": [
            { "gate_mask": "0b01000000", "duration_us": 50.0,  "op": "hold" },
            { "gate_mask": "0b10111111", "duration_us": 450.0, "op": "release" }
          ]
        }
      ],
      "streams": [
        {
          "id": 1, "name": "TT", "talker": "T_E", "listener": "L",
          "vid": 1, "pcp": 6, "frame_bytes": 128, "period_us": 500.0, "start_us": 500.0
        },
        {
          "id": 2, "name": "BE", "talker": "T_P", "listener": "L",
          "vid": 1, "pcp": 0, "frame_bytes": 1500, "period_us": 500.0, "start_us": 484.0
        }
      ]
    })";

    sim::Scenario sc = sim::Scenario::load_from_json(sim::Json::parse(cfg));
    sc.run();

    auto* sw1 = dynamic_cast<sim::SwitchNode*>(sc.find_node("SW1"));
    assert(sw1 != nullptr);
    /* BE arrives at SW1 at ~497.1 us -> starts transmitting -> preempted by Hold at 500 us! */
    assert(sw1->bridge().ports[2].qbu.preemptions_count >= 1);

    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats[0].rx_frames == 3);
    assert(stats[0].max_latency_ps < 4500 * TSN_PS_PER_NS);
    assert(stats[1].rx_frames == 3);
}

static void test_phase3_scenario_file() {
    sim::Scenario sc = sim::Scenario::load_from_file("scenarios/phase3_preemption.json");
    sc.run();
    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats.size() == 2);
    assert(stats[0].rx_frames == stats[0].tx_frames);
    assert(stats[0].deadline_misses == 0);
    assert(stats[1].rx_frames == stats[1].tx_frames);
}

int main() {
    test_qbu_fragmentation_and_reassembly();
    test_qbu_reduces_express_interference();
    test_qbv_hold_release_preemption();
    test_phase3_scenario_file();
    std::cout << "[PASS] test_phase3\n";
    return 0;
}
