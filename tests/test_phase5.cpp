#include "sim/config.hpp"
#include <cassert>
#include <iostream>

static void test_disjoint_path_finder() {
    sim::TopologyGraph g;
    g.add_bidirectional_link("L0", "T", 0, "SW1", 0, 1000000000ULL, 100 * TSN_PS_PER_NS);
    g.add_bidirectional_link("LA1", "SW1", 1, "SW2", 0, 1000000000ULL, 100 * TSN_PS_PER_NS);
    g.add_bidirectional_link("LA2", "SW2", 1, "SW4", 0, 1000000000ULL, 100 * TSN_PS_PER_NS);
    g.add_bidirectional_link("LB1", "SW1", 2, "SW3", 0, 1000000000ULL, 400 * TSN_PS_PER_NS);
    g.add_bidirectional_link("LB2", "SW3", 1, "SW4", 1, 1000000000ULL, 400 * TSN_PS_PER_NS);
    g.add_bidirectional_link("L5", "SW4", 2, "L", 0, 1000000000ULL, 100 * TSN_PS_PER_NS);

    auto paths = g.disjoint_paths("T", "L", 2);
    assert(paths.size() == 2);
    assert(paths[0][1].to_node == "SW2");
    assert(paths[1][1].to_node == "SW3");
}

static void test_phase5_qcc_auto_and_dynamic_reconfig() {
    sim::Scenario sc = sim::Scenario::load_from_file("scenarios/phase5_qcc_auto.json");

    /* Before run, CNC has already routed the 3 initial streams and synthesized GCLs */
    assert(sc.cnc() != nullptr);
    assert(sc.cnc()->last_routed_streams().size() == 3);
    assert(sc.cnc()->last_routed_streams()[0].paths.size() == 2); /* FRER dual-path */
    assert(sc.cnc()->last_schedule().success);
    assert(sc.cnc()->last_schedule().hyperperiod_ps == 500 * TSN_PS_PER_US);
    assert(!sc.cnc()->last_schedule().port_gcls.empty());

    sc.run();

    /* After run, dynamic stream #4 was added at t=4.0ms via CNC re-deploy */
    assert(sc.cnc()->last_routed_streams().size() == 4);

    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats.size() == 4);

    /* 1. Safety_TT_FRER: 20/20 delivered with 0% loss despite Path A link cut (2..3.5ms)
     * and despite mid-simulation hitless GCL reconfiguration at t=4.0ms! */
    const auto& tt1 = stats[0];
    assert(tt1.stream_name == "Safety_TT_FRER");
    assert(tt1.tx_frames == 20 && tt1.rx_frames == 20);
    assert(tt1.deadline_misses == 0);

    /* 2. AVB_Video: auto-configured with 150 Mbps CBS */
    const auto& avb = stats[1];
    assert(avb.stream_name == "AVB_Video");
    assert(avb.rx_frames >= 30); /* Some dropped on Path A during 2..3.5ms cut since non-FRER */
    assert(avb.deadline_misses == 0);

    /* 3. Dynamic_TT: started at 4.15ms, period 500us -> 12 frames up to 10ms, 0 misses */
    const auto& dyn = stats[3];
    assert(dyn.stream_name == "Dynamic_TT");
    assert(dyn.tx_frames == 12 && dyn.rx_frames == 12);
    assert(dyn.deadline_misses == 0);
}

int main() {
    test_disjoint_path_finder();
    test_phase5_qcc_auto_and_dynamic_reconfig();
    std::cout << "[PASS] test_phase5\n";
    return 0;
}
