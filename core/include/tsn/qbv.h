#ifndef TSN_QBV_H
#define TSN_QBV_H

#include "tsn/hal.h"
#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TSN_QBV_MAX_GCL_ENTRIES 64u

typedef enum tsn_qbv_op {
    TSN_QBV_OP_SET_GATE_STATES = 0,
    TSN_QBV_OP_SET_AND_HOLD_MAC = 1,
    TSN_QBV_OP_SET_AND_RELEASE_MAC = 2
} tsn_qbv_op_t;

typedef struct tsn_qbv_gcl_entry {
    uint8_t       gate_mask;   /* Bit i = 1 -> TC i gate open */
    tsn_time_ps_t duration_ps; /* Duration of this time interval */
    tsn_qbv_op_t  op;
} tsn_qbv_gcl_entry_t;

typedef struct tsn_qbv_config {
    tsn_time_ps_t       base_time_ps;
    tsn_time_ps_t       cycle_time_ps;
    tsn_time_ps_t       cycle_time_ext_ps;
    uint8_t             initial_gate_states;
    uint16_t            entry_count;
    tsn_qbv_gcl_entry_t entries[TSN_QBV_MAX_GCL_ENTRIES];
} tsn_qbv_config_t;

typedef void (*tsn_qbv_gate_change_cb_t)(void *arg, uint8_t port_id,
                                         uint8_t new_gate_mask, bool mac_held);

typedef struct tsn_qbv_port {
    uint8_t                  port_id;
    bool                     enabled;
    bool                     guard_band_enabled;
    const tsn_hal_t         *hal;
    tsn_qbv_gate_change_cb_t on_gate_change;
    void                    *cb_arg;

    /* Operational and pending Admin configurations */
    bool             oper_valid;
    tsn_qbv_config_t oper;
    bool             config_pending;
    tsn_qbv_config_t admin;
    tsn_time_ps_t    config_change_time_ps;

    /* Runtime GCL state */
    uint8_t       oper_gate_states;
    bool          mac_held;
    uint16_t      current_entry_idx;
    tsn_time_ps_t cycle_start_ps;
    tsn_time_ps_t next_change_ps;
    uint64_t      timer_id;
} tsn_qbv_port_t;

void tsn_qbv_init(tsn_qbv_port_t *qbv, uint8_t port_id, const tsn_hal_t *hal,
                  tsn_qbv_gate_change_cb_t cb, void *cb_arg);

/**
 * Load an Admin GCL configuration and trigger ConfigChange.
 * If oper is not yet running, starts at admin->base_time_ps (or next cycle if in past).
 * If oper is already running, atomically swaps at the next AdminBaseTime >= now.
 */
tsn_status_t tsn_qbv_set_admin_config(tsn_qbv_port_t *qbv, const tsn_qbv_config_t *admin,
                                      bool guard_band_enabled);

/** Return true if TC gate is currently open. */
bool tsn_qbv_is_gate_open(const tsn_qbv_port_t *qbv, uint8_t tc);

/**
 * Check if a frame of duration tx_dur_ps on traffic class tc is allowed to start
 * transmitting at now_ps (checks gate state + guard band before gate closing edge).
 */
bool tsn_qbv_can_transmit(const tsn_qbv_port_t *qbv, uint8_t tc,
                          tsn_time_ps_t now_ps, tsn_time_ps_t tx_dur_ps);

#ifdef __cplusplus
}
#endif

#endif /* TSN_QBV_H */
