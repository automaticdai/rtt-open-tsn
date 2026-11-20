#pragma once

#include "tsn/types.h"
#include <cstdint>

namespace sim {

/**
 * Local hardware oscillator model with initial phase offset, static frequency
 * drift (in parts-per-billion), and gPTP servo phase/rate adjustments.
 */
class LocalClock {
public:
    explicit LocalClock(tsn_time_ps_t initial_offset_ps = 0, double drift_ppb = 0.0);

    /** Convert global simulation time to this node's local clock reading. */
    tsn_time_ps_t sim_to_local(tsn_time_ps_t sim_time_ps) const;

    /** Convert a target local clock reading to global simulation time. */
    tsn_time_ps_t local_to_sim(tsn_time_ps_t target_local_ps) const;

    /** Apply a gPTP servo adjustment at sim_time_ps. */
    void adjust(tsn_time_ps_t sim_time_ps, int64_t phase_offset_ps, double servo_rate_ppb);

    double drift_ppb() const { return drift_ppb_; }
    double servo_rate_ppb() const { return servo_rate_ppb_; }
    double effective_rate() const { return 1.0 + (drift_ppb_ + servo_rate_ppb_) * 1e-9; }

private:
    /* Anchor point where the rate/offset was last updated */
    tsn_time_ps_t anchor_sim_ps_ = 0;
    tsn_time_ps_t anchor_local_ps_ = 0;
    double drift_ppb_ = 0.0;
    double servo_rate_ppb_ = 0.0;
};

} // namespace sim
