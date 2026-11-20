#pragma once

#include "sim/node.hpp"
#include "tsn/qbv.h"
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace sim {

struct TopoEdge {
    std::string   link_name;
    std::string   from_node;
    uint8_t       from_port = 0;
    std::string   to_node;
    uint8_t       to_port = 0;
    uint64_t      rate_bps = 1000000000ULL;
    tsn_time_ps_t propagation_delay_ps = 0;
};

struct TopologyGraph {
    std::map<std::string, tsn_time_ps_t> switch_fabric_delay_ps;
    std::map<std::string, std::vector<TopoEdge>> adj;

    void add_bidirectional_link(const std::string& link_name,
                                const std::string& a_node, uint8_t a_port,
                                const std::string& b_node, uint8_t b_port,
                                uint64_t rate_bps, tsn_time_ps_t prop_delay_ps);

    /** Shortest path (by propagation + fabric delay) avoiding banned edges/nodes. */
    std::vector<TopoEdge> shortest_path(const std::string& src, const std::string& dst,
                                        const std::vector<std::string>& banned_links = {},
                                        const std::vector<std::string>& banned_nodes = {}) const;

    /** Compute up to 2 link-disjoint (and ideally node-disjoint) paths for 802.1CB FRER. */
    std::vector<std::vector<TopoEdge>> disjoint_paths(const std::string& src,
                                                      const std::string& dst,
                                                      int max_paths = 2) const;
};

struct CncStreamReq {
    StreamSpec spec;
    bool       is_scheduled_tt = false; /* True -> synthesize Qbv GCL windows */
    int        redundancy_paths = 1;    /* >= 2 -> auto-configure 802.1CB FRER */
    double     avb_idle_slope_mbps = 0.0; /* > 0 -> auto-configure 802.1Qav CBS */
};

struct RoutedStream {
    CncStreamReq req;
    std::vector<std::vector<TopoEdge>> paths; /* 1 path normal, 2 paths for FRER */
};

struct PortKey {
    std::string switch_name;
    uint8_t     port_id = 0;

    bool operator<(const PortKey& o) const {
        if (switch_name != o.switch_name) return switch_name < o.switch_name;
        return port_id < o.port_id;
    }
};

struct SchedulerOptions {
    tsn_time_ps_t guard_margin_ps = 2 * TSN_PS_PER_US; /* Safety margin per TT window */
    bool          use_preemption_hold = false;
    bool          guard_band_enabled = true;
};

struct ScheduleResult {
    bool                               success = false;
    std::string                        error_msg;
    tsn_time_ps_t                      hyperperiod_ps = 0;
    std::map<PortKey, tsn_qbv_config_t> port_gcls;
    std::map<uint32_t, tsn_time_ps_t>  talker_start_offsets_ps;
};

/**
 * Pluggable IEEE 802.1Qcc GCL Scheduler interface.
 * Implement this interface to plug in external SMT/ILP solvers (e.g., Z3, OR-Tools).
 */
class IGclScheduler {
public:
    virtual ~IGclScheduler() = default;
    virtual ScheduleResult compute(const TopologyGraph& topo,
                                   const std::vector<RoutedStream>& streams,
                                   const SchedulerOptions& opts) = 0;
};

/**
 * Built-in v1 deterministic heuristic GCL synthesizer.
 * Computes the LCM hyperperiod and assigns conflict-free per-hop TT windows.
 */
class HeuristicGclScheduler : public IGclScheduler {
public:
    ScheduleResult compute(const TopologyGraph& topo,
                           const std::vector<RoutedStream>& streams,
                           const SchedulerOptions& opts) override;
};

/**
 * IEEE 802.1Qcc Centralized Network / User Configuration (CNC/CUC) controller.
 */
class CncController {
public:
    explicit CncController(std::unique_ptr<IGclScheduler> scheduler =
                               std::make_unique<HeuristicGclScheduler>());

    TopologyGraph& topology() { return topo_; }
    const TopologyGraph& topology() const { return topo_; }

    void set_options(const SchedulerOptions& opts) { opts_ = opts; }
    void register_stream(const CncStreamReq& req);
    void remove_stream(uint32_t stream_id);

    /**
     * Compute routes, FRER split/elim rules, and Qbv GCLs, then push Admin
     * configurations to all SwitchNodes with synchronized base_time_ps.
     */
    ScheduleResult deploy(const std::map<std::string, SimNode*>& nodes,
                          tsn_time_ps_t base_time_ps,
                          bool install_fdb_and_frer = true);

    const std::vector<RoutedStream>& last_routed_streams() const { return last_routed_; }
    const ScheduleResult& last_schedule() const { return last_schedule_; }

private:
    std::unique_ptr<IGclScheduler> scheduler_;
    TopologyGraph                  topo_;
    SchedulerOptions               opts_;
    std::vector<CncStreamReq>      requests_;
    std::vector<RoutedStream>      last_routed_;
    ScheduleResult                 last_schedule_;
};

} // namespace sim
