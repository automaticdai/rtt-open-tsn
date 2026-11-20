#include "tsn/frer.h"
#include <string.h>

void tsn_frer_init(tsn_frer_table_t *tbl) {
    if (!tbl) {
        return;
    }
    memset(tbl, 0, sizeof(*tbl));
}

tsn_status_t tsn_frer_add_gen_rule(tsn_frer_table_t *tbl, const tsn_mac_t *dst_mac,
                                   uint16_t vid, uint32_t split_port_mask) {
    if (!tbl || !dst_mac || tbl->gen_count >= TSN_FRER_MAX_RULES) {
        return tbl && dst_mac ? TSN_ERR_FULL : TSN_ERR_INVALID_ARG;
    }
    tsn_frer_gen_rule_t *r = &tbl->gen_rules[tbl->gen_count++];
    memset(r, 0, sizeof(*r));
    r->valid = true;
    r->dst_mac = *dst_mac;
    r->vid = vid;
    r->split_port_mask = split_port_mask;
    return TSN_OK;
}

tsn_status_t tsn_frer_add_recov_rule(tsn_frer_table_t *tbl, const tsn_mac_t *dst_mac,
                                     uint16_t vid, tsn_frer_algo_t algo,
                                     uint8_t history_length, bool strip_rtag,
                                     tsn_time_ps_t reset_timeout_ps) {
    if (!tbl || !dst_mac || tbl->recov_count >= TSN_FRER_MAX_RULES) {
        return tbl && dst_mac ? TSN_ERR_FULL : TSN_ERR_INVALID_ARG;
    }
    if (history_length < 2u) {
        history_length = 2u;
    }
    if (history_length > TSN_FRER_MAX_HISTORY_LEN) {
        history_length = TSN_FRER_MAX_HISTORY_LEN;
    }
    tsn_frer_recov_rule_t *r = &tbl->recov_rules[tbl->recov_count++];
    memset(r, 0, sizeof(*r));
    r->valid = true;
    r->dst_mac = *dst_mac;
    r->vid = vid;
    r->algo = algo;
    r->history_length = history_length;
    r->take_no_sequence = false;
    r->strip_rtag = strip_rtag;
    r->reset_timeout_ps = reset_timeout_ps;
    r->take_any = true;
    return TSN_OK;
}

static bool match_stream(const tsn_mac_t *rule_mac, uint16_t rule_vid,
                         const tsn_frame_t *frame) {
    if (!tsn_mac_equal(rule_mac, &frame->dst_mac)) {
        return false;
    }
    if (rule_vid != 0u) {
        if (!frame->has_vlan || frame->vid != rule_vid) {
            return false;
        }
    }
    return true;
}

static bool run_vector_recovery(tsn_frer_recov_rule_t *r, uint16_t seq) {
    if (r->take_any) {
        r->take_any = false;
        r->recov_seq_num = seq;
        r->history_mask = 1ULL;
        r->passed_packets++;
        return true;
    }

    /* Signed 16-bit delta in mod-65536 sequence space */
    int16_t delta = (int16_t)(seq - r->recov_seq_num);

    if (delta > 0) {
        /* Newer sequence number: advance sliding window */
        if (delta > 1) {
            r->lost_packets += (uint64_t)(delta - 1);
        }
        if ((uint16_t)delta >= (uint16_t)r->history_length || delta >= 64) {
            r->history_mask = 1ULL;
        } else {
            r->history_mask = (r->history_mask << (uint8_t)delta) | 1ULL;
            if (r->history_length < 64u) {
                uint64_t mask = (1ULL << r->history_length) - 1ULL;
                r->history_mask &= mask;
            }
        }
        r->recov_seq_num = seq;
        r->passed_packets++;
        return true;
    }

    /* delta <= 0: either duplicate, in-window out-of-order, or old rogue */
    int32_t age = -(int32_t)delta;
    if (age >= (int32_t)r->history_length) {
        r->rogue_packets++;
        r->discarded_packets++;
        return false;
    }

    uint64_t bit = 1ULL << (uint8_t)age;
    if ((r->history_mask & bit) != 0ULL) {
        /* Already received -> duplicate! */
        r->discarded_packets++;
        return false;
    }

    /* Not yet seen within history window -> out-of-order new frame! */
    r->history_mask |= bit;
    r->out_of_order_packets++;
    r->passed_packets++;
    return true;
}

static bool run_match_recovery(tsn_frer_recov_rule_t *r, uint16_t seq) {
    if (r->take_any || seq != r->recov_seq_num) {
        r->take_any = false;
        r->recov_seq_num = seq;
        r->passed_packets++;
        return true;
    }
    r->discarded_packets++;
    return false;
}

bool tsn_frer_process_recovery(tsn_frer_table_t *tbl, tsn_frame_t *frame,
                               tsn_time_ps_t now_ps) {
    if (!tbl || !frame) {
        return true;
    }
    for (uint16_t i = 0; i < tbl->recov_count; ++i) {
        tsn_frer_recov_rule_t *r = &tbl->recov_rules[i];
        if (!r->valid || !match_stream(&r->dst_mac, r->vid, frame)) {
            continue;
        }

        /* Check sequence recovery reset timeout */
        if (r->reset_timeout_ps > 0 && !r->take_any &&
            now_ps > r->last_rx_ps && (now_ps - r->last_rx_ps) >= r->reset_timeout_ps) {
            r->take_any = true;
            r->history_mask = 0ULL;
            r->resets_count++;
        }

        if (!frame->has_rtag) {
            r->tagless_packets++;
            if (r->take_no_sequence) {
                r->passed_packets++;
                r->last_rx_ps = now_ps;
                return true;
            }
            r->discarded_packets++;
            return false;
        }

        bool accept = (r->algo == TSN_FRER_ALGO_VECTOR)
                          ? run_vector_recovery(r, frame->frer_seq)
                          : run_match_recovery(r, frame->frer_seq);

        if (accept) {
            r->last_rx_ps = now_ps;
            if (r->strip_rtag && frame->has_rtag) {
                frame->has_rtag = false;
                if (frame->length_bytes > TSN_ETH_MIN_FRAME_BYTES + TSN_FRER_RTAG_BYTES) {
                    frame->length_bytes = (uint16_t)(frame->length_bytes - TSN_FRER_RTAG_BYTES);
                }
            }
        }
        return accept;
    }
    return true;
}

bool tsn_frer_process_generation(tsn_frer_table_t *tbl, tsn_frame_t *frame,
                                 uint32_t *split_mask_out) {
    if (!tbl || !frame) {
        return false;
    }
    for (uint16_t i = 0; i < tbl->gen_count; ++i) {
        tsn_frer_gen_rule_t *r = &tbl->gen_rules[i];
        if (!r->valid || !match_stream(&r->dst_mac, r->vid, frame)) {
            continue;
        }
        if (!frame->has_rtag) {
            frame->has_rtag = true;
            frame->frer_seq = r->next_seq++;
            if (frame->length_bytes + TSN_FRER_RTAG_BYTES <= TSN_ETH_MAX_FRAME_BYTES) {
                frame->length_bytes = (uint16_t)(frame->length_bytes + TSN_FRER_RTAG_BYTES);
            }
        }
        r->generated_packets++;
        if (split_mask_out) {
            *split_mask_out = r->split_port_mask;
        }
        return true;
    }
    return false;
}
