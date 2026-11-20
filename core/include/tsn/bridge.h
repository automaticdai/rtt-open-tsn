#ifndef TSN_BRIDGE_H
#define TSN_BRIDGE_H

#include "tsn/frer.h"
#include "tsn/gptp.h"
#include "tsn/hal.h"
#include "tsn/qav.h"
#include "tsn/qbv.h"
#include "tsn/qbu.h"
#include "tsn/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tsn_fdb_entry {
    tsn_mac_t dst_mac;
    uint16_t  vid;       /* 0 means wildcard / match any VID */
    uint32_t  port_mask; /* Bitmask of egress ports */
    bool      valid;
} tsn_fdb_entry_t;

struct tsn_bridge;

typedef struct tsn_qav_wakeup_ctx {
    struct tsn_bridge *br;
    uint8_t            port_id;
    uint8_t            tc;
} tsn_qav_wakeup_ctx_t;

typedef struct tsn_bridge_port {
    uint8_t              port_id;
    bool                 enabled;
    bool                 tx_busy;
    int8_t               active_tx_tc; /* -1 when idle or gPTP control */
    uint64_t             link_rate_bps;
    tsn_queue_t          gptp_queue; /* Dedicated control queue for 802.1AS */
    tsn_queue_t          queues[TSN_NUM_TRAFFIC_CLASSES];
    uint8_t              pcp_to_tc[TSN_NUM_TRAFFIC_CLASSES];
    tsn_qbv_port_t       qbv;
    tsn_qav_queue_t      qav[TSN_NUM_TRAFFIC_CLASSES];
    tsn_qav_wakeup_ctx_t qav_wakeup[TSN_NUM_TRAFFIC_CLASSES];
    tsn_qbu_port_t       qbu;
} tsn_bridge_port_t;

typedef struct tsn_bridge {
    uint8_t           num_ports;
    tsn_hal_t         hal;
    tsn_hal_t         gptp_hal;
    tsn_bridge_port_t ports[TSN_MAX_PORTS];
    tsn_fdb_entry_t   fdb[TSN_MAX_FDB_ENTRIES];
    uint16_t          fdb_count;
    tsn_gptp_node_t   gptp;
    tsn_frer_table_t  frer;
} tsn_bridge_t;

tsn_status_t tsn_bridge_init(tsn_bridge_t *br, uint8_t num_ports, const tsn_hal_t *hal);
tsn_status_t tsn_bridge_config_port(tsn_bridge_t *br, uint8_t port_id,
                                    uint64_t link_rate_bps, uint16_t queue_capacity);
tsn_status_t tsn_bridge_add_fdb(tsn_bridge_t *br, const tsn_mac_t *dst_mac,
                                uint16_t vid, uint32_t port_mask);

/* 802.1Qbv & 802.1Qav configuration */
tsn_status_t tsn_bridge_config_qbv(tsn_bridge_t *br, uint8_t port_id,
                                   const tsn_qbv_config_t *admin, bool guard_band_enabled);
tsn_status_t tsn_bridge_config_qav(tsn_bridge_t *br, uint8_t port_id, uint8_t tc,
                                   double idle_slope_bps, double hi_credit_bits,
                                   double lo_credit_bits);
tsn_status_t tsn_bridge_config_qbu(tsn_bridge_t *br, uint8_t port_id,
                                   uint8_t preemptable_mask, uint8_t add_frag_size);

/* 802.1CB FRER configuration */
tsn_status_t tsn_bridge_add_frer_gen(tsn_bridge_t *br, const tsn_mac_t *dst_mac,
                                     uint16_t vid, uint32_t split_port_mask);
tsn_status_t tsn_bridge_add_frer_recov(tsn_bridge_t *br, const tsn_mac_t *dst_mac,
                                       uint16_t vid, tsn_frer_algo_t algo,
                                       uint8_t history_length, bool strip_rtag,
                                       tsn_time_ps_t reset_timeout_ps);

/* 802.1AS gPTP configuration */
void tsn_bridge_init_gptp(tsn_bridge_t *br, const tsn_gptp_config_t *cfg,
                          const tsn_mac_t *bridge_mac);
void tsn_bridge_on_tx_timestamp(tsn_bridge_t *br, uint8_t egress_port,
                                const tsn_frame_t *frame, tsn_time_ps_t tx_ts_ps);

/**
 * Called when a frame arrives on ingress_port (after switch fabric delay for data,
 * or immediately for gPTP).
 */
tsn_status_t tsn_bridge_rx_frame(tsn_bridge_t *br, uint8_t ingress_port,
                                 const tsn_frame_t *frame);

/**
 * Called by HAL / PHY when an egress port finishes transmitting a frame.
 */
void tsn_bridge_on_tx_complete(tsn_bridge_t *br, uint8_t egress_port);

/**
 * Attempt to start transmission on egress_port if idle and any queue is eligible.
 */
void tsn_bridge_try_tx(tsn_bridge_t *br, uint8_t egress_port);

#ifdef __cplusplus
}
#endif

#endif /* TSN_BRIDGE_H */
