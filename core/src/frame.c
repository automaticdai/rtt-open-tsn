#include "tsn/frame.h"
#include <string.h>

void tsn_frame_init(tsn_frame_t *frame) {
    if (!frame) {
        return;
    }
    memset(frame, 0, sizeof(*frame));
    frame->ethertype = TSN_ETHERTYPE_TSN_DATA;
    frame->length_bytes = TSN_ETH_MIN_FRAME_BYTES;
    frame->smd_type = TSN_SMD_E;
    frame->is_final_frag = true;
}

tsn_time_ps_t tsn_frame_tx_time_ps(uint16_t frame_bytes, uint64_t link_rate_bps,
                                   bool include_ifg) {
    if (link_rate_bps == 0) {
        return 0;
    }
    uint64_t wire_bytes = (uint64_t)frame_bytes + TSN_ETH_PREAMBLE_BYTES +
                          (include_ifg ? TSN_ETH_IFG_BYTES : 0u);
    uint64_t bits = wire_bytes * 8u;
    /* Use 128-bit arithmetic to avoid overflow when scaling to picoseconds */
    __uint128_t ps = ((__uint128_t)bits * (__uint128_t)TSN_PS_PER_S) / link_rate_bps;
    return (tsn_time_ps_t)ps;
}
