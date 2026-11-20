#pragma once

#include "tsn/types.h"
#include <cstdint>
#include <functional>
#include <queue>
#include <random>
#include <unordered_set>
#include <vector>

namespace sim {

using EventId = uint64_t;
using EventCallback = std::function<void()>;

/**
 * Deterministic picosecond discrete-event simulation kernel.
 * Events are strictly ordered by (timestamp_ps, priority, seq_id).
 */
class SimKernel {
public:
    explicit SimKernel(uint64_t seed = 42);

    tsn_time_ps_t now_ps() const { return now_ps_; }
    uint64_t executed_events() const { return executed_events_; }
    size_t pending_events() const { return queue_.size(); }

    EventId schedule_at(tsn_time_ps_t timestamp_ps, EventCallback cb, int priority = 0);
    EventId schedule_after(tsn_time_ps_t delay_ps, EventCallback cb, int priority = 0);
    void cancel(EventId id);

    void run_until(tsn_time_ps_t end_time_ps);
    void run_all();
    void stop() { stopped_ = true; }

    std::mt19937_64& rng() { return rng_; }
    double uniform_01();

private:
    struct Event {
        tsn_time_ps_t timestamp_ps;
        int           priority;
        EventId       id;
        EventCallback callback;

        bool operator>(const Event& other) const {
            if (timestamp_ps != other.timestamp_ps) return timestamp_ps > other.timestamp_ps;
            if (priority != other.priority) return priority > other.priority;
            return id > other.id;
        }
    };

    tsn_time_ps_t now_ps_ = 0;
    EventId next_id_ = 1;
    uint64_t executed_events_ = 0;
    bool stopped_ = false;
    std::mt19937_64 rng_;
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> queue_;
    std::unordered_set<EventId> cancelled_;
};

} // namespace sim
