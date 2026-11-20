#include "sim/kernel.hpp"
#include <stdexcept>

namespace sim {

SimKernel::SimKernel(uint64_t seed) : rng_(seed) {}

EventId SimKernel::schedule_at(tsn_time_ps_t timestamp_ps, EventCallback cb, int priority) {
    if (timestamp_ps < now_ps_) {
        timestamp_ps = now_ps_;
    }
    EventId id = next_id_++;
    queue_.push(Event{timestamp_ps, priority, id, std::move(cb)});
    return id;
}

EventId SimKernel::schedule_after(tsn_time_ps_t delay_ps, EventCallback cb, int priority) {
    if (delay_ps < 0) {
        delay_ps = 0;
    }
    return schedule_at(now_ps_ + delay_ps, std::move(cb), priority);
}

void SimKernel::cancel(EventId id) {
    if (id != 0) {
        cancelled_.insert(id);
    }
}

void SimKernel::run_until(tsn_time_ps_t end_time_ps) {
    stopped_ = false;
    while (!stopped_ && !queue_.empty()) {
        if (queue_.top().timestamp_ps > end_time_ps) {
            break;
        }
        Event evt = queue_.top();
        queue_.pop();

        auto it = cancelled_.find(evt.id);
        if (it != cancelled_.end()) {
            cancelled_.erase(it);
            continue;
        }

        now_ps_ = evt.timestamp_ps;
        executed_events_++;
        if (evt.callback) {
            evt.callback();
        }
    }
    if (!stopped_ && now_ps_ < end_time_ps) {
        now_ps_ = end_time_ps;
    }
}

void SimKernel::run_all() {
    stopped_ = false;
    while (!stopped_ && !queue_.empty()) {
        Event evt = queue_.top();
        queue_.pop();

        auto it = cancelled_.find(evt.id);
        if (it != cancelled_.end()) {
            cancelled_.erase(it);
            continue;
        }

        now_ps_ = evt.timestamp_ps;
        executed_events_++;
        if (evt.callback) {
            evt.callback();
        }
    }
}

double SimKernel::uniform_01() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng_);
}

} // namespace sim
