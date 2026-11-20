#ifndef TSN_QAV_H
#define TSN_QAV_H

#include "tsn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * IEEE 802.1Qav Credit-Based Shaper (CBS) per-queue state.
 * Event-driven: credit is updated lazily on state transitions, and exact
 * zero-crossing wakeup delays are computed when credit < 0.
 */
typedef struct tsn_qav_queue {
    bool          enabled;
    double        idle_slope_bps;  /* > 0 */
    double        send_slope_bps;  /* < 0 (idle_slope - port_rate) */
    double        hi_credit_bits;  /* >= 0 */
    double        lo_credit_bits;  /* <= 0 */

    double        credit_bits;
    tsn_time_ps_t last_update_ps;

    bool          transmitting;
    bool          gate_open;
    bool          has_queued_frame;
    uint64_t      wakeup_timer_id;
} tsn_qav_queue_t;

void tsn_qav_init(tsn_qav_queue_t *q);

tsn_status_t tsn_qav_config(tsn_qav_queue_t *q, double idle_slope_bps,
                            uint64_t port_rate_bps, double hi_credit_bits,
                            double lo_credit_bits);

/** Lazily advance CBS credit up to now_ps. */
void tsn_qav_update(tsn_qav_queue_t *q, tsn_time_ps_t now_ps);

/** State transition hooks */
void tsn_qav_on_tx_start(tsn_qav_queue_t *q, tsn_time_ps_t now_ps);
void tsn_qav_on_tx_end(tsn_qav_queue_t *q, tsn_time_ps_t now_ps, bool still_has_frames);
void tsn_qav_on_enqueue(tsn_qav_queue_t *q, tsn_time_ps_t now_ps);
void tsn_qav_on_gate_change(tsn_qav_queue_t *q, tsn_time_ps_t now_ps, bool gate_open);

/** Return true if CBS allows transmitting now (credit >= 0 or CBS disabled). */
bool tsn_qav_can_transmit(tsn_qav_queue_t *q, tsn_time_ps_t now_ps);

/**
 * If credit < 0 and queue is eligible to recover (gate_open && has_queued_frame),
 * return picoseconds until credit reaches 0. Returns 0 if already ready or not recovering.
 */
tsn_time_ps_t tsn_qav_time_until_ready_ps(tsn_qav_queue_t *q, tsn_time_ps_t now_ps);

#ifdef __cplusplus
}
#endif

#endif /* TSN_QAV_H */
