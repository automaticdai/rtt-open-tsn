#include "tsn/qbu.h"
#include <string.h>

void tsn_qbu_init(tsn_qbu_port_t *qbu) {
    if (!qbu) {
        return;
    }
    memset(qbu, 0, sizeof(*qbu));
}

tsn_status_t tsn_qbu_config(tsn_qbu_port_t *qbu, uint8_t preemptable_mask,
                            uint8_t add_frag_size) {
    if (!qbu || add_frag_size > 3u) {
        return TSN_ERR_INVALID_ARG;
    }
    qbu->enabled = true;
    qbu->preemptable_mask = preemptable_mask;
    qbu->add_frag_size = add_frag_size;
    return TSN_OK;
}

bool tsn_qbu_can_preempt(const tsn_qbu_port_t *qbu, tsn_time_ps_t now_ps,
                         uint64_t link_rate_bps, uint16_t *cut_bytes_out) {
    if (!qbu || !qbu->enabled || !qbu->tx.active || !qbu->tx.active_is_preemptable ||
        qbu->tx.active_preempt_requested || link_rate_bps == 0) {
        return false;
    }

    uint16_t min_frag = tsn_qbu_min_frag_bytes(qbu);
    uint16_t total_frag_bytes = qbu->tx.active_frag_bytes;

    /* Must have at least min_frag + 64B remaining to allow preemption */
    if (total_frag_bytes < (uint16_t)(min_frag + TSN_QBU_MIN_REMAINING_BYTES)) {
        return false;
    }

    /* Compute how many payload/header bytes (after 8B preamble) have already been serialized */
    tsn_time_ps_t elapsed_ps = now_ps > qbu->tx.active_start_ps
                                   ? (now_ps - qbu->tx.active_start_ps)
                                   : 0;
    __uint128_t bits_sent = ((__uint128_t)elapsed_ps * (__uint128_t)link_rate_bps) /
                            (__uint128_t)TSN_PS_PER_S;
    uint64_t wire_bytes_sent = (uint64_t)(bits_sent / 8u);
    uint64_t body_bytes_sent = wire_bytes_sent > TSN_ETH_PREAMBLE_BYTES
                                   ? (wire_bytes_sent - TSN_ETH_PREAMBLE_BYTES)
                                   : 0u;

    /* Cut at max(body_bytes_sent + 1, min_frag) so we never cut in the past */
    uint64_t cut = body_bytes_sent + 1u;
    if (cut < min_frag) {
        cut = min_frag;
    }

    /* Ensure at least TSN_QBU_MIN_REMAINING_BYTES (64B) remain for the continuation */
    if (cut + TSN_QBU_MIN_REMAINING_BYTES > total_frag_bytes) {
        return false;
    }

    if (cut_bytes_out) {
        *cut_bytes_out = (uint16_t)cut;
    }
    return true;
}

void tsn_qbu_mark_preempted(tsn_qbu_port_t *qbu, uint16_t cut_bytes) {
    if (!qbu || !qbu->tx.active) {
        return;
    }
    qbu->tx.active_preempt_requested = true;
    qbu->tx.has_suspended = true;
    qbu->tx.suspended_frame = qbu->tx.active_frame;
    qbu->tx.suspended_tc = qbu->tx.active_tc;
    qbu->tx.suspended_bytes_sent = (uint16_t)(qbu->tx.active_offset_bytes + cut_bytes);
    if (qbu->tx.active_offset_bytes == 0u) {
        /* First fragment was SMD-S; next continuation starts at frag_count = 0 */
        qbu->tx.suspended_frag_count = 0u;
    } else {
        qbu->tx.suspended_frag_count = (uint8_t)((qbu->tx.suspended_frag_count + 1u) & 0x03u);
    }
    qbu->preemptions_count++;
}

void tsn_qbu_prepare_tx_start(tsn_qbu_port_t *qbu, uint8_t tc, bool is_resuming,
                              tsn_time_ps_t now_ps, tsn_frame_t *frame_inout) {
    if (!qbu || !frame_inout) {
        return;
    }

    if (!qbu->enabled || !tsn_qbu_is_preemptable(qbu, tc)) {
        frame_inout->smd_type = TSN_SMD_E;
        frame_inout->frag_count = 0;
        frame_inout->is_final_frag = true;
        frame_inout->frag_len_bytes = frame_inout->length_bytes;
        qbu->tx.active = true;
        qbu->tx.active_is_preemptable = false;
        qbu->tx.active_preempt_requested = false;
        qbu->tx.active_tc = tc;
        qbu->tx.active_start_ps = now_ps;
        return;
    }

    qbu->tx.active = true;
    qbu->tx.active_is_preemptable = true;
    qbu->tx.active_preempt_requested = false;
    qbu->tx.active_tc = tc;
    qbu->tx.active_start_ps = now_ps;
    qbu->fragments_tx_count++;

    if (is_resuming) {
        /* Resuming a previously preempted pMAC frame with SMD-C */
        qbu->tx.active_frame = qbu->tx.suspended_frame;
        qbu->tx.active_offset_bytes = qbu->tx.suspended_bytes_sent;
        uint16_t remaining = (uint16_t)(qbu->tx.suspended_frame.length_bytes -
                                        qbu->tx.suspended_bytes_sent);
        qbu->tx.active_frag_bytes = remaining;

        *frame_inout = qbu->tx.suspended_frame;
        frame_inout->smd_type = TSN_SMD_C;
        frame_inout->frag_count = qbu->tx.suspended_frag_count;
        frame_inout->is_final_frag = true;
        frame_inout->frag_len_bytes = remaining;
        frame_inout->length_bytes = remaining;

        qbu->tx.has_suspended = false;
    } else {
        /* Fresh pMAC frame starting with SMD-S */
        qbu->tx.active_frame = *frame_inout;
        qbu->tx.active_offset_bytes = 0;
        qbu->tx.active_frag_bytes = frame_inout->length_bytes;

        frame_inout->smd_type = TSN_SMD_S;
        frame_inout->frag_count = 0;
        frame_inout->is_final_frag = true;
        frame_inout->frag_len_bytes = frame_inout->length_bytes;
    }
}

void tsn_qbu_on_tx_complete(tsn_qbu_port_t *qbu) {
    if (!qbu) {
        return;
    }
    qbu->tx.active = false;
    qbu->tx.active_is_preemptable = false;
    qbu->tx.active_preempt_requested = false;
}

bool tsn_qbu_rx_mpacket(tsn_qbu_port_t *qbu, const tsn_frame_t *mpacket,
                        tsn_frame_t *completed_out) {
    if (!qbu || !mpacket || !completed_out) {
        return false;
    }

    /* Express frames (SMD-E) bypass reassembly completely */
    if (mpacket->smd_type == TSN_SMD_E) {
        *completed_out = *mpacket;
        return true;
    }

    qbu->fragments_rx_count++;

    if (mpacket->smd_type == TSN_SMD_S) {
        if (mpacket->is_final_frag) {
            /* Unpreempted full pMAC frame */
            *completed_out = *mpacket;
            return true;
        }
        /* Start of a fragmented pMAC frame */
        if (qbu->rx.assembling) {
            qbu->reassembly_errors++;
        }
        qbu->rx.assembling = true;
        qbu->rx.partial_frame = *mpacket;
        qbu->rx.bytes_received = mpacket->frag_len_bytes;
        qbu->rx.expected_frag_count = 0;
        return false;
    }

    if (mpacket->smd_type == TSN_SMD_C) {
        if (!qbu->rx.assembling || mpacket->frag_count != qbu->rx.expected_frag_count) {
            qbu->reassembly_errors++;
            qbu->rx.assembling = false;
            return false;
        }
        qbu->rx.bytes_received = (uint16_t)(qbu->rx.bytes_received + mpacket->frag_len_bytes);
        qbu->rx.expected_frag_count = (uint8_t)((qbu->rx.expected_frag_count + 1u) & 0x03u);

        if (mpacket->is_final_frag) {
            *completed_out = qbu->rx.partial_frame;
            completed_out->length_bytes = qbu->rx.bytes_received;
            completed_out->smd_type = TSN_SMD_S;
            completed_out->is_final_frag = true;
            completed_out->frag_len_bytes = qbu->rx.bytes_received;
            qbu->rx.assembling = false;
            qbu->reassembled_frames++;
            return true;
        }
        return false;
    }

    return false;
}
