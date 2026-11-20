#ifndef TSN_TYPES_H
#define TSN_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Simulation / hardware timestamp in picoseconds */
typedef int64_t tsn_time_ps_t;

#define TSN_PS_PER_NS   ((int64_t)1000LL)
#define TSN_PS_PER_US   ((int64_t)1000000LL)
#define TSN_PS_PER_MS   ((int64_t)1000000000LL)
#define TSN_PS_PER_S    ((int64_t)1000000000000LL)

/* Ethernet & TSN protocol constants */
#define TSN_ETH_ALEN            6u
#define TSN_ETH_MIN_FRAME_BYTES 64u
#define TSN_ETH_MAX_FRAME_BYTES 1522u
#define TSN_ETH_PREAMBLE_BYTES  8u   /* 7B preamble + 1B SFD */
#define TSN_ETH_IFG_BYTES       12u  /* Inter-frame gap */
#define TSN_ETH_OVERHEAD_BYTES  (TSN_ETH_PREAMBLE_BYTES + TSN_ETH_IFG_BYTES)

#define TSN_TPID_VLAN           0x8100u
#define TSN_TPID_FRER           0xF1C1u
#define TSN_ETHERTYPE_GPTP      0x88F7u
#define TSN_ETHERTYPE_IPV4      0x0800u
#define TSN_ETHERTYPE_TSN_DATA  0x88B5u /* Local experimental EtherType */

#define TSN_NUM_TRAFFIC_CLASSES 8u
#define TSN_MAX_PORTS           16u
#define TSN_MAX_FDB_ENTRIES     128u
#define TSN_QUEUE_CAPACITY      64u
#define TSN_MAX_HOPS            16u

typedef struct tsn_mac {
    uint8_t bytes[TSN_ETH_ALEN];
} tsn_mac_t;

typedef enum tsn_status {
    TSN_OK = 0,
    TSN_ERR_INVALID_ARG = -1,
    TSN_ERR_FULL = -2,
    TSN_ERR_EMPTY = -3,
    TSN_ERR_NOT_FOUND = -4,
    TSN_ERR_BUSY = -5
} tsn_status_t;

static inline bool tsn_mac_equal(const tsn_mac_t *a, const tsn_mac_t *b) {
    for (size_t i = 0; i < TSN_ETH_ALEN; ++i) {
        if (a->bytes[i] != b->bytes[i]) {
            return false;
        }
    }
    return true;
}

static inline bool tsn_mac_is_broadcast(const tsn_mac_t *m) {
    for (size_t i = 0; i < TSN_ETH_ALEN; ++i) {
        if (m->bytes[i] != 0xFFu) {
            return false;
        }
    }
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* TSN_TYPES_H */
