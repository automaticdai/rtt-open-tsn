#include "sim/clock.hpp"
#include <cmath>

namespace sim {

LocalClock::LocalClock(tsn_time_ps_t initial_offset_ps, double drift_ppb)
    : anchor_sim_ps_(0),
      anchor_local_ps_(initial_offset_ps),
      drift_ppb_(drift_ppb),
      servo_rate_ppb_(0.0) {}

tsn_time_ps_t LocalClock::sim_to_local(tsn_time_ps_t sim_time_ps) const {
    double dt_sim = static_cast<double>(sim_time_ps - anchor_sim_ps_);
    double dt_local = dt_sim * effective_rate();
    return anchor_local_ps_ + static_cast<tsn_time_ps_t>(std::llround(dt_local));
}

tsn_time_ps_t LocalClock::local_to_sim(tsn_time_ps_t target_local_ps) const {
    double dt_local = static_cast<double>(target_local_ps - anchor_local_ps_);
    double dt_sim = dt_local / effective_rate();
    return anchor_sim_ps_ + static_cast<tsn_time_ps_t>(std::llround(dt_sim));
}

void LocalClock::adjust(tsn_time_ps_t sim_time_ps, int64_t phase_offset_ps,
                        double servo_rate_ppb) {
    /* Re-anchor at sim_time_ps to preserve continuity before changing rate/phase */
    tsn_time_ps_t current_local = sim_to_local(sim_time_ps);
    anchor_sim_ps_ = sim_time_ps;
    anchor_local_ps_ = current_local + phase_offset_ps;
    servo_rate_ppb_ = servo_rate_ppb;
}

} // namespace sim
