#ifndef TSN_GPTP_H
#define TSN_GPTP_H

#include "tsn/frame.h"
#include "tsn/hal.h"
#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum tsn_gptp_port_role {
    TSN_GPTP_ROLE_DISABLED = 0,
    TSN_GPTP_ROLE_MASTER = 1,
    TSN_GPTP_ROLE_SLAVE = 2
} tsn_gptp_port_role_t;

typedef struct tsn_gptp_port {
    uint8_t              port_id;
    tsn_gptp_port_role_t role;

    /* Peer delay measurement state */
    uint16_t      pdelay_seq_id;
    tsn_time_ps_t t1_req_tx_ps;
    tsn_time_ps_t t2_req_rx_ps;
    tsn_time_ps_t t3_resp_tx_ps;
    tsn_time_ps_t t4_resp_rx_ps;

    /* Previous Pdelay exchange for neighborRateRatio */
    bool          has_prev_pdelay;
    tsn_time_ps_t prev_t3_ps;
    tsn_time_ps_t prev_t4_ps;

    tsn_time_ps_t mean_link_delay_ps;
    double        neighbor_rate_ratio;
    bool          pdelay_valid;

    /* Last Sync received on this port (for Follow_Up pairing) */
    uint16_t      last_sync_seq;
    tsn_time_ps_t last_sync_rx_ps;
    bool          sync_pending;
} tsn_gptp_port_t;

typedef struct tsn_gptp_config {
    bool          is_gm;
    uint8_t       num_ports;
    tsn_time_ps_t sync_interval_ps;
    tsn_time_ps_t pdelay_interval_ps;
    double        servo_kp; /* Proportional gain (ppb per ns error, or fraction) */
    double        servo_ki; /* Integral gain */
} tsn_gptp_config_t;

typedef struct tsn_gptp_node {
    bool              enabled;
    tsn_gptp_config_t cfg;
    const tsn_hal_t  *hal;
    tsn_mac_t         node_mac;

    tsn_gptp_port_t   ports[TSN_MAX_PORTS];

    /* Grandmaster Sync sequence counter */
    uint16_t      gm_sync_seq;
    uint64_t      sync_timer_id;
    uint64_t      pdelay_timer_id;

    /* PI Clock Servo state */
    bool          servo_initialized;
    double        servo_integral_ppb;
    double        servo_rate_ppb;
    int64_t       last_offset_err_ps;
    uint64_t      sync_rx_count;
} tsn_gptp_node_t;

void tsn_gptp_init(tsn_gptp_node_t *gptp, const tsn_gptp_config_t *cfg,
                   const tsn_hal_t *hal, const tsn_mac_t *node_mac);

void tsn_gptp_set_port_role(tsn_gptp_node_t *gptp, uint8_t port_id,
                            tsn_gptp_port_role_t role);

/** Start periodic Pdelay (all enabled ports) and Sync (if Grandmaster) timers. */
void tsn_gptp_start(tsn_gptp_node_t *gptp);

/** Process an incoming 802.1AS gPTP frame on ingress_port. */
void tsn_gptp_rx_frame(tsn_gptp_node_t *gptp, uint8_t ingress_port,
                       const tsn_frame_t *frame);

/** Notify gPTP of the hardware TX timestamp when a gPTP frame hits the wire. */
void tsn_gptp_on_tx_timestamp(tsn_gptp_node_t *gptp, uint8_t port_id,
                              const tsn_frame_t *frame, tsn_time_ps_t tx_ts_ps);

#ifdef __cplusplus
}
#endif

#endif /* TSN_GPTP_H */
