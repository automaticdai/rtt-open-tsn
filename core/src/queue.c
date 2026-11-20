#include "tsn/queue.h"
#include <string.h>

void tsn_queue_init(tsn_queue_t *q, uint16_t capacity) {
    if (!q) {
        return;
    }
    memset(q, 0, sizeof(*q));
    if (capacity == 0 || capacity > TSN_QUEUE_CAPACITY) {
        capacity = TSN_QUEUE_CAPACITY;
    }
    q->capacity = capacity;
}

bool tsn_queue_is_empty(const tsn_queue_t *q) {
    return !q || q->count == 0;
}

bool tsn_queue_is_full(const tsn_queue_t *q) {
    return !q || q->count >= q->capacity;
}

uint16_t tsn_queue_size(const tsn_queue_t *q) {
    return q ? q->count : 0;
}

tsn_status_t tsn_queue_enqueue(tsn_queue_t *q, const tsn_frame_t *frame) {
    if (!q || !frame) {
        return TSN_ERR_INVALID_ARG;
    }
    if (q->count >= q->capacity) {
        q->dropped_total++;
        return TSN_ERR_FULL;
    }
    q->slots[q->tail] = *frame;
    q->tail = (uint16_t)((q->tail + 1u) % q->capacity);
    q->count++;
    q->enqueued_total++;
    return TSN_OK;
}

tsn_status_t tsn_queue_dequeue(tsn_queue_t *q, tsn_frame_t *out_frame) {
    if (!q || !out_frame) {
        return TSN_ERR_INVALID_ARG;
    }
    if (q->count == 0) {
        return TSN_ERR_EMPTY;
    }
    *out_frame = q->slots[q->head];
    q->head = (uint16_t)((q->head + 1u) % q->capacity);
    q->count--;
    return TSN_OK;
}

const tsn_frame_t *tsn_queue_peek(const tsn_queue_t *q) {
    if (!q || q->count == 0) {
        return NULL;
    }
    return &q->slots[q->head];
}
