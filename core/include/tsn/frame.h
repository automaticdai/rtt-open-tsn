#ifndef TSN_FRAME_H
#define TSN_FRAME_H

#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IEEE 802.3br mPacket Start-of-Packet Delimiter types */
typedef enum tsn_smd_type {
    TSN_SMD_E = 0,  /* Express frame */
    TSN_SMD_S = 1,  /* Preemptable initial fragment */
    TSN_SMD_C = 2   /* Preemptable continuation fragment */
} tsn_smd_type_t;

/* IEEE 802.1AS gPTP message types */
typedef enum tsn_gptp_msg_type {
    TSN_GPTP_NONE = 0,
    TSN_GPTP_SYNC = 1,
    TSN_GPTP_FOLLOW_UP = 2,
    TSN_GPTP_PDELAY_REQ = 3,
    TSN_GPTP_PDELAY_RESP = 4,
    TSN_GPTP_PDELAY_RESP_FUP = 5
} tsn_gptp_msg_type_t;

typedef struct tsn_gptp_hdr {
    tsn_gptp_msg_type_t msg_type;
    uint16_t            seq_id;
    uint8_t             src_port_id;
    tsn_time_ps_t       precise_origin_ts_ps;
    int64_t             correction_field_ps;
    tsn_time_ps_t       request_receipt_ts_ps;
    tsn_time_ps_t       response_origin_ts_ps;
} tsn_gptp_hdr_t;

typedef struct tsn_frame {
    /* L2 Header */
    tsn_mac_t dst_mac;
    tsn_mac_t src_mac;
    uint16_t  ethertype;

    /* 802.1Q VLAN Tag */
    bool      has_vlan;
    uint8_t   pcp;   /* Priority Code Point (0..7) */
    uint8_t   dei;   /* Drop Eligible Indicator (0..1) */
    uint16_t  vid;   /* VLAN Identifier (0..4095) */

    /* 802.1CB FRER R-TAG (TPID 0xF1C1) */
    bool      has_rtag;
    uint16_t  frer_seq;

    /* Wire frame size in bytes (excluding preamble/SFD and IFG, including FCS) */
    uint16_t  length_bytes;

    /* 802.3br Preemption mPacket metadata */
    tsn_smd_type_t smd_type;
    uint8_t        frag_count;
    bool           is_final_frag;
    uint16_t       frag_len_bytes;

    /* 802.1AS gPTP payload & hardware timestamps (local clock) */
    tsn_gptp_hdr_t gptp;
    tsn_time_ps_t  hw_tx_ts_ps;
    tsn_time_ps_t  hw_rx_ts_ps;

#ifdef TSN_SIM
    /* Simulation-only tracking metadata (stripped on embedded builds) */
    uint32_t      stream_id;
    uint64_t      frame_uid;
    uint32_t      seq_in_stream;
    tsn_time_ps_t gen_time_ps;
    tsn_time_ps_t ingress_time_ps;
    tsn_time_ps_t deadline_ps;
    uint8_t       hop_count;
#endif
} tsn_frame_t;

/**
 * Initialize a frame with defaults (valid 802.1Q frame, Express SMD-E).
 */
void tsn_frame_init(tsn_frame_t *frame);

/**
 * Compute serialization duration in picoseconds for a frame on a link of link_rate_bps.
 * Includes Ethernet preamble/SFD (8B) and optionally IFG (12B).
 */
tsn_time_ps_t tsn_frame_tx_time_ps(uint16_t frame_bytes, uint64_t link_rate_bps,
                                   bool include_ifg);

#ifdef __cplusplus
}
#endif

#endif /* TSN_FRAME_H */
