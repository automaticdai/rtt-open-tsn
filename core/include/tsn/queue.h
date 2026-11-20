#ifndef TSN_QUEUE_H
#define TSN_QUEUE_H

#include "tsn/frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fixed-capacity ring buffer for tsn_frame_t (zero dynamic allocation).
 */
typedef struct tsn_queue {
    tsn_frame_t slots[TSN_QUEUE_CAPACITY];
    uint16_t    head;
    uint16_t    tail;
    uint16_t    count;
    uint16_t    capacity;
    uint64_t    enqueued_total;
    uint64_t    dropped_total;
} tsn_queue_t;

void         tsn_queue_init(tsn_queue_t *q, uint16_t capacity);
bool         tsn_queue_is_empty(const tsn_queue_t *q);
bool         tsn_queue_is_full(const tsn_queue_t *q);
uint16_t     tsn_queue_size(const tsn_queue_t *q);
tsn_status_t tsn_queue_enqueue(tsn_queue_t *q, const tsn_frame_t *frame);
tsn_status_t tsn_queue_dequeue(tsn_queue_t *q, tsn_frame_t *out_frame);
const tsn_frame_t *tsn_queue_peek(const tsn_queue_t *q);

#ifdef __cplusplus
}
#endif

#endif /* TSN_QUEUE_H */
