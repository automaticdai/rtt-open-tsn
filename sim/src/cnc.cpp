#include "sim/cnc.hpp"
#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <set>

namespace sim {

void TopologyGraph::add_bidirectional_link(const std::string& link_name,
                                           const std::string& a_node, uint8_t a_port,
                                           const std::string& b_node, uint8_t b_port,
                                           uint64_t rate_bps, tsn_time_ps_t prop_delay_ps) {
    adj[a_node].push_back(TopoEdge{link_name, a_node, a_port, b_node, b_port, rate_bps, prop_delay_ps});
    adj[b_node].push_back(TopoEdge{link_name, b_node, b_port, a_node, a_port, rate_bps, prop_delay_ps});
}

std::vector<TopoEdge> TopologyGraph::shortest_path(
    const std::string& src, const std::string& dst,
    const std::vector<std::string>& banned_links,
    const std::vector<std::string>& banned_nodes) const {
    std::set<std::string> ban_l(banned_links.begin(), banned_links.end());
    std::set<std::string> ban_n(banned_nodes.begin(), banned_nodes.end());

    using QItem = std::pair<tsn_time_ps_t, std::string>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    std::map<std::string, tsn_time_ps_t> dist;
    std::map<std::string, TopoEdge> prev;

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        if (u == dst) break;

        auto it = adj.find(u);
        if (it == adj.end()) continue;
        for (const auto& e : it->second) {
            if (ban_l.count(e.link_name)) continue;
            if (e.to_node != dst && ban_n.count(e.to_node)) continue;

            tsn_time_ps_t fab = 0;
            auto fit = switch_fabric_delay_ps.find(e.to_node);
            if (fit != switch_fabric_delay_ps.end()) fab = fit->second;

            tsn_time_ps_t nd = d + e.propagation_delay_ps + fab + 1;
            auto dit = dist.find(e.to_node);
            if (dit == dist.end() || nd < dit->second) {
                dist[e.to_node] = nd;
                prev[e.to_node] = e;
                pq.push({nd, e.to_node});
            }
        }
    }

    if (prev.find(dst) == prev.end()) {
        return {};
    }

    std::vector<TopoEdge> path;
    std::string cur = dst;
    while (cur != src) {
        const auto& e = prev.at(cur);
        path.push_back(e);
        cur = e.from_node;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::vector<TopoEdge>> TopologyGraph::disjoint_paths(
    const std::string& src, const std::string& dst, int max_paths) const {
    std::vector<std::vector<TopoEdge>> result;
    auto p1 = shortest_path(src, dst);
    if (p1.empty()) return result;
    result.push_back(p1);
    if (max_paths <= 1) return result;

    /* Try node-disjoint intermediate switches first (excluding first/last access switch
     * if Talker/Listener only have 1 port). */
    std::vector<std::string> inner_nodes;
    std::vector<std::string> inner_links;
    if (p1.size() >= 3) {
        /* Links between switches: indices 1 .. p1.size()-2 */
        for (size_t i = 1; i + 1 < p1.size(); ++i) {
            inner_links.push_back(p1[i].link_name);
        }
        /* Strictly intermediate switches (excluding entry & exit switches) */
        for (size_t i = 2; i + 1 < p1.size(); ++i) {
            inner_nodes.push_back(p1[i].from_node);
        }
    } else {
        for (const auto& e : p1) inner_links.push_back(e.link_name);
    }

    auto p2 = shortest_path(src, dst, inner_links, inner_nodes);
    if (p2.empty() && !inner_nodes.empty()) {
        /* Fallback to edge-disjoint only */
        p2 = shortest_path(src, dst, inner_links, {});
    }
    if (!p2.empty()) {
        result.push_back(p2);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* HeuristicGclScheduler                                                     */
/* ------------------------------------------------------------------------- */

namespace {

int64_t lcm_i64(int64_t a, int64_t b) {
    if (a <= 0) return b;
    if (b <= 0) return a;
    return (a / std::gcd(a, b)) * b;
}

struct ReservedWindow {
    tsn_time_ps_t start_ps;
    tsn_time_ps_t end_ps;
    uint8_t       pcp;
};

bool overlaps(const ReservedWindow& a, const ReservedWindow& b) {
    return a.start_ps < b.end_ps && b.start_ps < a.end_ps;
}

tsn_time_ps_t find_non_overlapping_slot(const std::vector<ReservedWindow>& existing,
                                        tsn_time_ps_t desired_start,
                                        tsn_time_ps_t duration,
                                        tsn_time_ps_t hyperperiod) {
    tsn_time_ps_t s = desired_start % hyperperiod;
    for (int iter = 0; iter < 256; ++iter) {
        ReservedWindow cand{s, s + duration, 0};
        bool conflict = false;
        for (const auto& w : existing) {
            if (overlaps(cand, w)) {
                s = w.end_ps;
                conflict = true;
                break;
            }
        }
        if (!conflict) {
            return s;
        }
    }
    return desired_start % hyperperiod;
}

} // namespace

ScheduleResult HeuristicGclScheduler::compute(const TopologyGraph& topo,
                                              const std::vector<RoutedStream>& streams,
                                              const SchedulerOptions& opts) {
    ScheduleResult res;
    res.success = true;

    /* Collect scheduled TT streams */
    std::vector<const RoutedStream*> tt_streams;
    tsn_time_ps_t hyperperiod = 0;
    uint8_t all_tt_pcp_mask = 0;

    for (const auto& rs : streams) {
        if (rs.req.is_scheduled_tt && rs.req.spec.period_ps > 0) {
            tt_streams.push_back(&rs);
            hyperperiod = lcm_i64(hyperperiod, rs.req.spec.period_ps);
            all_tt_pcp_mask |= static_cast<uint8_t>(1u << (rs.req.spec.pcp & 0x07u));
        }
    }

    if (tt_streams.empty()) {
        res.hyperperiod_ps = 0;
        return res;
    }
    res.hyperperiod_ps = hyperperiod;

    /* Sort TT streams by tightest deadline / smallest period first */
    std::sort(tt_streams.begin(), tt_streams.end(),
              [](const RoutedStream* a, const RoutedStream* b) {
                  tsn_time_ps_t da = a->req.spec.deadline_ps > 0 ? a->req.spec.deadline_ps
                                                                 : a->req.spec.period_ps;
                  tsn_time_ps_t db = b->req.spec.deadline_ps > 0 ? b->req.spec.deadline_ps
                                                                 : b->req.spec.period_ps;
                  if (da != db) return da < db;
                  return a->req.spec.stream_id < b->req.spec.stream_id;
              });

    std::map<PortKey, std::vector<ReservedWindow>> port_windows;

    /* Reserve conflict-free hop-by-hop windows for every TT stream instance in hyperperiod */
    for (const auto* rs : tt_streams) {
        const auto& spec = rs->req.spec;
        tsn_time_ps_t base_offset = spec.start_time_ps % spec.period_ps;
        res.talker_start_offsets_ps[spec.stream_id] = base_offset;

        int64_t instances = hyperperiod / spec.period_ps;
        uint16_t wire_bytes = spec.frame_bytes;
        if (rs->paths.size() >= 2) {
            wire_bytes = static_cast<uint16_t>(wire_bytes + 6u); /* Account for 802.1CB R-TAG */
        }

        for (int64_t k = 0; k < instances; ++k) {
            tsn_time_ps_t tx_time = base_offset + k * spec.period_ps;

            for (const auto& path : rs->paths) {
                tsn_time_ps_t cur_t = tx_time;
                for (size_t hop = 0; hop < path.size(); ++hop) {
                    const auto& e = path[hop];
                    uint16_t hop_bytes = (hop == 0 || hop + 1 == path.size())
                                             ? spec.frame_bytes
                                             : wire_bytes;
                    tsn_time_ps_t tx_dur = tsn_frame_tx_time_ps(hop_bytes, e.rate_bps, true);

                    if (hop == 0) {
                        /* EndStation talker link -> arrives at first switch after tx + prop */
                        cur_t += tsn_frame_tx_time_ps(hop_bytes, e.rate_bps, false) +
                                 e.propagation_delay_ps;
                        auto fit = topo.switch_fabric_delay_ps.find(e.to_node);
                        if (fit != topo.switch_fabric_delay_ps.end()) {
                            cur_t += fit->second;
                        }
                    } else {
                        /* Switch egress port: reserve a Qbv window */
                        PortKey pk{e.from_node, e.from_port};
                        tsn_time_ps_t win_dur = tx_dur + opts.guard_margin_ps;
                        tsn_time_ps_t open_t = find_non_overlapping_slot(
                            port_windows[pk], cur_t, win_dur, hyperperiod);

                        if (open_t + win_dur <= hyperperiod) {
                            port_windows[pk].push_back(
                                ReservedWindow{open_t, open_t + win_dur, spec.pcp});
                        }

                        cur_t = open_t + tsn_frame_tx_time_ps(hop_bytes, e.rate_bps, false) +
                                e.propagation_delay_ps;
                        auto fit = topo.switch_fabric_delay_ps.find(e.to_node);
                        if (fit != topo.switch_fabric_delay_ps.end()) {
                            cur_t += fit->second;
                        }
                    }
                }
            }
        }
    }

    /* Convert each port's reserved windows into a valid tsn_qbv_config_t GCL */
    uint8_t non_tt_mask = static_cast<uint8_t>(~all_tt_pcp_mask);
    for (auto& [pk, wins] : port_windows) {
        std::sort(wins.begin(), wins.end(),
                  [](const ReservedWindow& a, const ReservedWindow& b) {
                      return a.start_ps < b.start_ps;
                  });

        tsn_qbv_config_t gcl{};
        gcl.base_time_ps = 0;
        gcl.cycle_time_ps = hyperperiod;
        gcl.initial_gate_states = 0xFFu;
        gcl.entry_count = 0;

        tsn_time_ps_t cursor = 0;
        auto push_entry = [&](uint8_t mask, tsn_time_ps_t dur, tsn_qbv_op_t op) {
            if (dur <= 0 || gcl.entry_count >= TSN_QBV_MAX_GCL_ENTRIES) return;
            /* Coalesce adjacent entries with identical mask & op */
            if (gcl.entry_count > 0 &&
                gcl.entries[gcl.entry_count - 1].gate_mask == mask &&
                gcl.entries[gcl.entry_count - 1].op == op) {
                gcl.entries[gcl.entry_count - 1].duration_ps += dur;
                return;
            }
            gcl.entries[gcl.entry_count++] = tsn_qbv_gcl_entry_t{mask, dur, op};
        };

        for (const auto& w : wins) {
            if (w.start_ps > cursor) {
                push_entry(non_tt_mask, w.start_ps - cursor,
                           opts.use_preemption_hold ? TSN_QBV_OP_SET_AND_RELEASE_MAC
                                                    : TSN_QBV_OP_SET_GATE_STATES);
                cursor = w.start_ps;
            }
            tsn_time_ps_t w_end = std::min(w.end_ps, hyperperiod);
            if (w_end > cursor) {
                uint8_t tt_mask = static_cast<uint8_t>(1u << (w.pcp & 0x07u));
                push_entry(tt_mask, w_end - cursor,
                           opts.use_preemption_hold ? TSN_QBV_OP_SET_AND_HOLD_MAC
                                                    : TSN_QBV_OP_SET_GATE_STATES);
                cursor = w_end;
            }
        }
        if (cursor < hyperperiod) {
            push_entry(non_tt_mask, hyperperiod - cursor,
                       opts.use_preemption_hold ? TSN_QBV_OP_SET_AND_RELEASE_MAC
                                                : TSN_QBV_OP_SET_GATE_STATES);
        }

        res.port_gcls[pk] = gcl;
    }

    return res;
}

/* ------------------------------------------------------------------------- */
/* CncController                                                             */
/* ------------------------------------------------------------------------- */

CncController::CncController(std::unique_ptr<IGclScheduler> scheduler)
    : scheduler_(std::move(scheduler)) {}

void CncController::register_stream(const CncStreamReq& req) {
    for (auto& r : requests_) {
        if (r.spec.stream_id == req.spec.stream_id) {
            r = req;
            return;
        }
    }
    requests_.push_back(req);
}

void CncController::remove_stream(uint32_t stream_id) {
    requests_.erase(
        std::remove_if(requests_.begin(), requests_.end(),
                       [stream_id](const CncStreamReq& r) {
                           return r.spec.stream_id == stream_id;
                       }),
        requests_.end());
}

ScheduleResult CncController::deploy(const std::map<std::string, SimNode*>& nodes,
                                     tsn_time_ps_t base_time_ps,
                                     bool install_fdb_and_frer) {
    last_routed_.clear();

    /* 1. Compute paths for every registered stream */
    for (const auto& req : requests_) {
        RoutedStream rs;
        rs.req = req;
        rs.paths = topo_.disjoint_paths(req.spec.talker, req.spec.listener,
                                        req.redundancy_paths);
        if (rs.paths.empty()) {
            ScheduleResult fail;
            fail.success = false;
            fail.error_msg = "CNC failed to find path for stream " + req.spec.name;
            return fail;
        }
        last_routed_.push_back(std::move(rs));
    }

    /* 2. Install FDB, FRER split/elim, and CBS rules on switches */
    if (install_fdb_and_frer) {
        for (const auto& rs : last_routed_) {
            const auto& spec = rs.req.spec;
            /* Collect egress ports per switch across all paths for this stream */
            std::map<std::string, uint32_t> sw_egress_masks;
            for (const auto& path : rs.paths) {
                for (size_t hop = 1; hop < path.size(); ++hop) {
                    const auto& e = path[hop];
                    sw_egress_masks[e.from_node] |= (1u << e.from_port);

                    /* Optional 802.1Qav CBS auto-config */
                    if (rs.req.avb_idle_slope_mbps > 0.0) {
                        auto nit = nodes.find(e.from_node);
                        if (nit != nodes.end()) {
                            if (auto* sw = dynamic_cast<SwitchNode*>(nit->second)) {
                                sw->config_qav(e.from_port, spec.pcp,
                                               rs.req.avb_idle_slope_mbps * 1e6);
                            }
                        }
                    }
                }
            }

            /* Install FDB entries on every switch along the paths */
            for (const auto& [sw_name, mask] : sw_egress_masks) {
                auto nit = nodes.find(sw_name);
                if (nit == nodes.end()) continue;
                if (auto* sw = dynamic_cast<SwitchNode*>(nit->second)) {
                    sw->add_fdb_entry(spec.dst_mac, spec.vid, mask);
                }
            }

            /* If FRER dual-path: configure Split on first switch & Recovery on last switch */
            if (rs.paths.size() >= 2 && rs.paths[0].size() >= 2 && rs.paths[1].size() >= 2) {
                std::string split_sw = rs.paths[0][1].from_node;
                std::string elim_sw = rs.paths[0].back().from_node;
                uint32_t split_mask = sw_egress_masks[split_sw];

                if (auto it = nodes.find(split_sw); it != nodes.end()) {
                    if (auto* sw = dynamic_cast<SwitchNode*>(it->second)) {
                        sw->add_frer_gen(spec.dst_mac, spec.vid, split_mask);
                    }
                }
                if (auto it = nodes.find(elim_sw); it != nodes.end()) {
                    if (auto* sw = dynamic_cast<SwitchNode*>(it->second)) {
                        sw->add_frer_recov(spec.dst_mac, spec.vid, TSN_FRER_ALGO_VECTOR,
                                           16, true, 0);
                    }
                }
            }
        }
    }

    /* 3. Synthesize Qbv GCLs and push synchronized Admin configs */
    last_schedule_ = scheduler_->compute(topo_, last_routed_, opts_);
    if (!last_schedule_.success) {
        return last_schedule_;
    }

    for (auto& [pk, gcl] : last_schedule_.port_gcls) {
        gcl.base_time_ps = base_time_ps;
        auto nit = nodes.find(pk.switch_name);
        if (nit == nodes.end()) continue;
        if (auto* sw = dynamic_cast<SwitchNode*>(nit->second)) {
            sw->config_qbv(pk.port_id, gcl, opts_.guard_band_enabled);
        }
    }

    return last_schedule_;
}

} // namespace sim
