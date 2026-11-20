#ifndef TSN_QBU_H
#define TSN_QBU_H

#include "tsn/frame.h"
#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TSN_QBU_MIN_REMAINING_BYTES 64u /* 60B payload/hdr + 4B FCS */
#define TSN_QBU_MCRC_BYTES          4u

typedef struct tsn_qbu_tx {
    /* Suspended (preempted) pMAC frame awaiting resumption */
    bool        has_suspended;
    tsn_frame_t suspended_frame;
    uint8_t     suspended_tc;
    uint16_t    suspended_bytes_sent;
    uint8_t     suspended_frag_count;

    /* Currently transmitting frame on the wire */
    bool          active;
    bool          active_is_preemptable;
    bool          active_preempt_requested;
    tsn_frame_t   active_frame;
    uint8_t       active_tc;
    tsn_time_ps_t active_start_ps;
    uint16_t      active_offset_bytes;
    uint16_t      active_frag_bytes;
} tsn_qbu_tx_t;

typedef struct tsn_qbu_rx {
    bool        assembling;
    tsn_frame_t partial_frame;
    uint16_t    bytes_received;
    uint8_t     expected_frag_count;
} tsn_qbu_rx_t;

typedef struct tsn_qbu_port {
    bool         enabled;
    uint8_t      preemptable_mask; /* Bit tc=1 -> pMAC, bit tc=0 -> eMAC */
    uint8_t      add_frag_size;    /* 0..3 -> min fragment = 64 * (1 + add_frag_size) */
    bool         hold_active;      /* Driven by 802.1Qbv SetAndHoldMAC / SetAndReleaseMAC */

    tsn_qbu_tx_t tx;
    tsn_qbu_rx_t rx;

    /* Telemetry counters */
    uint64_t     preemptions_count;
    uint64_t     fragments_tx_count;
    uint64_t     fragments_rx_count;
    uint64_t     reassembled_frames;
    uint64_t     reassembly_errors;
} tsn_qbu_port_t;

void tsn_qbu_init(tsn_qbu_port_t *qbu);

tsn_status_t tsn_qbu_config(tsn_qbu_port_t *qbu, uint8_t preemptable_mask,
                            uint8_t add_frag_size);

static inline bool tsn_qbu_is_preemptable(const tsn_qbu_port_t *qbu, uint8_t tc) {
    if (!qbu || !qbu->enabled || tc >= TSN_NUM_TRAFFIC_CLASSES) {
        return false;
    }
    return ((qbu->preemptable_mask >> tc) & 1u) != 0u;
}

static inline uint16_t tsn_qbu_min_frag_bytes(const tsn_qbu_port_t *qbu) {
    uint8_t add = (qbu && qbu->add_frag_size <= 3u) ? qbu->add_frag_size : 0u;
    return (uint16_t)(64u * (1u + (uint16_t)add));
}

/**
 * Check if the currently active pMAC transmission can be preempted at now_ps.
 * If yes, writes the byte count of the truncated fragment into *cut_bytes_out
 * (excluding 4B mCRC) and returns true.
 */
bool tsn_qbu_can_preempt(const tsn_qbu_port_t *qbu, tsn_time_ps_t now_ps,
                         uint64_t link_rate_bps, uint16_t *cut_bytes_out);

/**
 * Record that the active pMAC frame is being truncated at cut_bytes (without mCRC),
 * saving the remaining bytes into the suspended state for later SMD-C resumption.
 */
void tsn_qbu_mark_preempted(tsn_qbu_port_t *qbu, uint16_t cut_bytes);

/**
 * Prepare a freshly dequeued frame (or resumed suspended fragment) for wire TX.
 * Sets smd_type, frag_count, is_final_frag, frag_len_bytes, and length_bytes.
 */
void tsn_qbu_prepare_tx_start(tsn_qbu_port_t *qbu, uint8_t tc, bool is_resuming,
                              tsn_time_ps_t now_ps, tsn_frame_t *frame_inout);

/**
 * Called when the wire TX of the current mPacket finishes.
 */
void tsn_qbu_on_tx_complete(tsn_qbu_port_t *qbu);

/**
 * Process an incoming mPacket at ingress MAC.
 * Returns true and populates *completed_out when a complete frame is ready
 * (immediately for Express SMD-E or unpreempted SMD-S, or upon final SMD-C).
 * Returns false if the mPacket is a non-final fragment buffered for reassembly
 * or was dropped due to a reassembly error.
 */
bool tsn_qbu_rx_mpacket(tsn_qbu_port_t *qbu, const tsn_frame_t *mpacket,
                        tsn_frame_t *completed_out);

#ifdef __cplusplus
}
#endif

#endif /* TSN_QBU_H */
