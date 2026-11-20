#include "sim/config.hpp"
#include "tsn/frer.h"
#include <cassert>
#include <iostream>

static void test_frer_vector_and_match_recovery() {
    tsn_frer_table_t tbl;
    tsn_frer_init(&tbl);
    tsn_mac_t dst = {{0x02, 0x00, 0x00, 0x00, 0x00, 0x02}};
    assert(tsn_frer_add_recov_rule(&tbl, &dst, 10, TSN_FRER_ALGO_VECTOR, 8, true, 0) == TSN_OK);

    auto make_tagged = [&](uint16_t seq) {
        tsn_frame_t f;
        tsn_frame_init(&f);
        f.dst_mac = dst;
        f.has_vlan = true;
        f.vid = 10;
        f.has_rtag = true;
        f.frer_seq = seq;
        f.length_bytes = 134; /* 128 + 6B R-TAG */
        return f;
    };

    /* Seq 0 accepted, R-TAG stripped back to 128B */
    auto f0 = make_tagged(0);
    assert(tsn_frer_process_recovery(&tbl, &f0, 1000));
    assert(!f0.has_rtag && f0.length_bytes == 128);

    /* Duplicate seq 0 discarded */
    auto f0_dup = make_tagged(0);
    assert(!tsn_frer_process_recovery(&tbl, &f0_dup, 2000));

    /* Seq 2 arrives before seq 1 (out-of-order) -> both accepted, duplicate of 1 discarded */
    auto f2 = make_tagged(2);
    assert(tsn_frer_process_recovery(&tbl, &f2, 3000));
    auto f1 = make_tagged(1);
    assert(tsn_frer_process_recovery(&tbl, &f1, 4000));
    auto f1_dup = make_tagged(1);
    assert(!tsn_frer_process_recovery(&tbl, &f1_dup, 5000));

    const auto& r = tbl.recov_rules[0];
    assert(r.passed_packets == 3);
    assert(r.discarded_packets == 2);
    assert(r.out_of_order_packets == 1);

    /* 16-bit wrap-around: 65535 -> 0 -> 1 */
    tsn_frer_table_t tbl_wrap;
    tsn_frer_init(&tbl_wrap);
    assert(tsn_frer_add_recov_rule(&tbl_wrap, &dst, 10, TSN_FRER_ALGO_VECTOR, 8, false, 0) == TSN_OK);
    auto fw1 = make_tagged(65535);
    auto fw2 = make_tagged(0);
    auto fw2_dup = make_tagged(0);
    assert(tsn_frer_process_recovery(&tbl_wrap, &fw1, 100));
    assert(tsn_frer_process_recovery(&tbl_wrap, &fw2, 200));
    assert(!tsn_frer_process_recovery(&tbl_wrap, &fw2_dup, 300));
    assert(tbl_wrap.recov_rules[0].passed_packets == 2);
    assert(tbl_wrap.recov_rules[0].discarded_packets == 1);
}

static void test_frer_diamond_zero_loss_failover() {
    sim::Scenario sc = sim::Scenario::load_from_file("scenarios/phase4_frer_diamond.json");
    sc.run();

    auto stats = sc.telemetry().compute_stream_stats();
    assert(stats.size() == 1);
    const auto& s = stats[0];

    /* 50 frames sent over 10ms (every 200us starting at 50us).
     * Path A is cut from 2..4ms (10 frames dropped on Path A),
     * Path B is cut from 6..8ms (10 frames dropped on Path B).
     * Result: 20 total link drops, yet 50/50 frames delivered with ZERO loss! */
    assert(s.tx_frames == 50);
    assert(s.rx_frames == 50);
    assert(s.deadline_misses == 0);
    assert(sc.telemetry().total_link_drops() == 20);

    /* Verify FRER counters on SW4_Elim: 50 passed, 30 duplicates discarded (80 arrived total) */
    auto* sw4 = dynamic_cast<sim::SwitchNode*>(sc.find_node("SW4_Elim"));
    assert(sw4 != nullptr);
    const auto& recov = sw4->bridge().frer.recov_rules[0];
    assert(recov.passed_packets == 50);
    assert(recov.discarded_packets == 30);
    assert(recov.lost_packets == 0);
}

int main() {
    test_frer_vector_and_match_recovery();
    test_frer_diamond_zero_loss_failover();
    std::cout << "[PASS] test_phase4\n";
    return 0;
}
