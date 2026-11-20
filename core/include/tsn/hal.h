#ifndef TSN_HAL_H
#define TSN_HAL_H

#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct tsn_frame;

typedef void (*tsn_timer_cb_t)(void *arg);

typedef enum tsn_metric_type {
    TSN_METRIC_FRAME_ENQUEUE = 1,
    TSN_METRIC_FRAME_DROP = 2,
    TSN_METRIC_FRAME_TX_START = 3,
    TSN_METRIC_FRAME_TX_END = 4,
    TSN_METRIC_FRAME_RX = 5,
    TSN_METRIC_GPTP_SYNC = 6,
    TSN_METRIC_QBV_GATE_CHANGE = 7
} tsn_metric_type_t;

typedef struct tsn_metric_event {
    tsn_metric_type_t type;
    tsn_time_ps_t     timestamp_ps;
    uint8_t           port_id;
    uint8_t           tc;
    uint32_t          stream_id;
    uint64_t          frame_uid;
    int64_t           value;
} tsn_metric_event_t;

/**
 * Hardware Abstraction Layer (HAL) for libtsn_core.
 * In simulation, SimHal binds these callbacks to the DES kernel and LocalClock.
 * On real-time / embedded targets, a platform HAL binds them to hardware PTP & NIC DMA.
 */
typedef struct tsn_hal {
    void *ctx;

    /* Current local synchronized time in picoseconds */
    tsn_time_ps_t (*get_time_ps)(void *ctx);

    /* Adjust local clock phase offset (ps) and frequency rate (ppb) */
    void (*adjust_clock)(void *ctx, int64_t offset_ps, double rate_ppb);

    /* Schedule a one-shot timer at target_local_ps; returns non-zero timer handle */
    uint64_t (*schedule_timer)(void *ctx, tsn_time_ps_t target_local_ps,
                               tsn_timer_cb_t cb, void *arg);

    /* Cancel a previously scheduled timer */
    void (*cancel_timer)(void *ctx, uint64_t timer_id);

    /* Submit a frame for transmission on port_id; returns TSN_OK or TSN_ERR_BUSY */
    tsn_status_t (*tx_frame)(void *ctx, uint8_t port_id, const struct tsn_frame *frame);

    /* Request 802.3br preemption of the currently active pMAC TX on port_id at cut_bytes */
    tsn_status_t (*preempt_tx)(void *ctx, uint8_t port_id, uint16_t cut_bytes);

    /* Optional telemetry / trace hook */
    void (*emit_metric)(void *ctx, const tsn_metric_event_t *evt);
} tsn_hal_t;

#ifdef __cplusplus
}
#endif

#endif /* TSN_HAL_H */
