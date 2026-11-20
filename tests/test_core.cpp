#include "tsn/bridge.h"
#include "tsn/frame.h"
#include "tsn/queue.h"
#include <cassert>
#include <cstring>
#include <iostream>

static void test_frame_serialization_time() {
    /* 128B frame + 8B preamble = 136B = 1088 bits. At 1 Gbps -> 1088 ns = 1,088,000 ps */
    tsn_time_ps_t t_no_ifg = tsn_frame_tx_time_ps(128, 1000000000ULL, false);
    assert(t_no_ifg == 1088 * TSN_PS_PER_NS);

    /* With 12B IFG: 148B = 1184 bits -> 1184 ns */
    tsn_time_ps_t t_ifg = tsn_frame_tx_time_ps(128, 1000000000ULL, true);
    assert(t_ifg == 1184 * TSN_PS_PER_NS);

    /* At 10 Gbps: 128B + 8B = 136B -> 108.8 ns = 108,800 ps (exact integer picoseconds!) */
    tsn_time_ps_t t_10g = tsn_frame_tx_time_ps(128, 10000000000ULL, false);
    assert(t_10g == 108800LL);
}

static void test_ring_queue() {
    tsn_queue_t q;
    tsn_queue_init(&q, 3);
    assert(tsn_queue_is_empty(&q));
    assert(!tsn_queue_is_full(&q));

    for (uint16_t i = 0; i < 3; ++i) {
        tsn_frame_t f;
        tsn_frame_init(&f);
        f.length_bytes = static_cast<uint16_t>(64 + i);
        assert(tsn_queue_enqueue(&q, &f) == TSN_OK);
    }
    assert(tsn_queue_is_full(&q));
    assert(tsn_queue_size(&q) == 3);

    /* Overflow drops tail */
    tsn_frame_t extra;
    tsn_frame_init(&extra);
    assert(tsn_queue_enqueue(&q, &extra) == TSN_ERR_FULL);
    assert(q.dropped_total == 1);

    for (uint16_t i = 0; i < 3; ++i) {
        tsn_frame_t out;
        assert(tsn_queue_dequeue(&q, &out) == TSN_OK);
        assert(out.length_bytes == 64 + i);
    }
    assert(tsn_queue_is_empty(&q));
    tsn_frame_t dummy;
    assert(tsn_queue_dequeue(&q, &dummy) == TSN_ERR_EMPTY);
}

struct MockBridgeCtx {
    int tx_calls = 0;
    uint8_t last_port = 0xFF;
    uint8_t last_pcp = 0xFF;
};

static tsn_time_ps_t mock_get_time(void*) { return 0; }
static tsn_status_t mock_tx(void* ctx, uint8_t port_id, const tsn_frame_t* frame) {
    auto* m = static_cast<MockBridgeCtx*>(ctx);
    m->tx_calls++;
    m->last_port = port_id;
    m->last_pcp = frame->pcp;
    return TSN_OK;
}

static void test_bridge_strict_priority() {
    MockBridgeCtx ctx;
    tsn_hal_t hal{};
    hal.ctx = &ctx;
    hal.get_time_ps = mock_get_time;
    hal.tx_frame = mock_tx;

    tsn_bridge_t br;
    assert(tsn_bridge_init(&br, 4, &hal) == TSN_OK);

    tsn_mac_t dst = {{0x02, 0x00, 0x00, 0x00, 0x00, 0x02}};
    assert(tsn_bridge_add_fdb(&br, &dst, 10, (1u << 1)) == TSN_OK);

    /* First frame starts TX immediately on port 1 (makes port 1 busy) */
    tsn_frame_t f1;
    tsn_frame_init(&f1);
    f1.dst_mac = dst;
    f1.has_vlan = true;
    f1.vid = 10;
    f1.pcp = 0;
    assert(tsn_bridge_rx_frame(&br, 0, &f1) == TSN_OK);
    assert(ctx.tx_calls == 1 && ctx.last_port == 1 && ctx.last_pcp == 0);

    /* While busy, enqueue a low-priority (PCP 1) then a high-priority (PCP 6) frame */
    tsn_frame_t f_low = f1;
    f_low.pcp = 1;
    tsn_frame_t f_high = f1;
    f_high.pcp = 6;
    assert(tsn_bridge_rx_frame(&br, 0, &f_low) == TSN_OK);
    assert(tsn_bridge_rx_frame(&br, 0, &f_high) == TSN_OK);
    assert(ctx.tx_calls == 1);

    /* When TX completes, PCP 6 must be sent before PCP 1 */
    tsn_bridge_on_tx_complete(&br, 1);
    assert(ctx.tx_calls == 2 && ctx.last_pcp == 6);

    tsn_bridge_on_tx_complete(&br, 1);
    assert(ctx.tx_calls == 3 && ctx.last_pcp == 1);
}

int main() {
    test_frame_serialization_time();
    test_ring_queue();
    test_bridge_strict_priority();
    std::cout << "[PASS] test_core\n";
    return 0;
}
