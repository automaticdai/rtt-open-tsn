#ifndef TSN_FRER_H
#define TSN_FRER_H

#include "tsn/frame.h"
#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TSN_FRER_MAX_RULES       32u
#define TSN_FRER_RTAG_BYTES      6u /* 2B TPID (0xF1C1) + 2B reserved + 2B seq */
#define TSN_FRER_MAX_HISTORY_LEN 64u

typedef enum tsn_frer_algo {
    TSN_FRER_ALGO_VECTOR = 0,
    TSN_FRER_ALGO_MATCH = 1
} tsn_frer_algo_t;

typedef struct tsn_frer_gen_rule {
    bool      valid;
    tsn_mac_t dst_mac;
    uint16_t  vid;             /* 0 = wildcard */
    uint32_t  split_port_mask; /* Egress ports for replication */
    uint16_t  next_seq;
    uint64_t  generated_packets;
} tsn_frer_gen_rule_t;

typedef struct tsn_frer_recov_rule {
    bool            valid;
    tsn_mac_t       dst_mac;
    uint16_t        vid;            /* 0 = wildcard */
    tsn_frer_algo_t algo;
    uint8_t         history_length; /* 2..64 for Vector Recovery */
    bool            take_no_sequence;
    bool            strip_rtag;
    tsn_time_ps_t   reset_timeout_ps; /* 0 = disabled */

    /* Runtime recovery state */
    bool          take_any;
    uint16_t      recov_seq_num;
    uint64_t      history_mask;
    tsn_time_ps_t last_rx_ps;

    /* IEEE 802.1CB standard counters */
    uint64_t passed_packets;
    uint64_t discarded_packets;
    uint64_t out_of_order_packets;
    uint64_t rogue_packets;
    uint64_t lost_packets;
    uint64_t tagless_packets;
    uint64_t resets_count;
} tsn_frer_recov_rule_t;

typedef struct tsn_frer_table {
    tsn_frer_gen_rule_t   gen_rules[TSN_FRER_MAX_RULES];
    uint16_t              gen_count;
    tsn_frer_recov_rule_t recov_rules[TSN_FRER_MAX_RULES];
    uint16_t              recov_count;
} tsn_frer_table_t;

void tsn_frer_init(tsn_frer_table_t *tbl);

tsn_status_t tsn_frer_add_gen_rule(tsn_frer_table_t *tbl, const tsn_mac_t *dst_mac,
                                   uint16_t vid, uint32_t split_port_mask);

tsn_status_t tsn_frer_add_recov_rule(tsn_frer_table_t *tbl, const tsn_mac_t *dst_mac,
                                     uint16_t vid, tsn_frer_algo_t algo,
                                     uint8_t history_length, bool strip_rtag,
                                     tsn_time_ps_t reset_timeout_ps);

/**
 * Match a Sequence Recovery rule for frame.
 * Returns true if the frame should be forwarded (or if no rule matched).
 * Returns false if the frame is a duplicate/rogue and must be eliminated.
 */
bool tsn_frer_process_recovery(tsn_frer_table_t *tbl, tsn_frame_t *frame,
                               tsn_time_ps_t now_ps);

/**
 * Match a Sequence Generation & Stream Splitting rule for frame.
 * If matched, stamps R-TAG sequence number on frame, writes split_port_mask into
 * *split_mask_out, and returns true. Otherwise returns false.
 */
bool tsn_frer_process_generation(tsn_frer_table_t *tbl, tsn_frame_t *frame,
                                 uint32_t *split_mask_out);

#ifdef __cplusplus
}
#endif

#endif /* TSN_FRER_H */
