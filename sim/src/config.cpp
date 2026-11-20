#include "sim/config.hpp"
#include <cstdio>
#include <stdexcept>

namespace sim {

static uint8_t parse_gate_mask(const Json& j) {
    if (j.is_int()) {
        return static_cast<uint8_t>(j.as_int() & 0xFF);
    }
    const std::string& s = j.as_string();
    if (s.rfind("0b", 0) == 0 || s.rfind("0B", 0) == 0) {
        return static_cast<uint8_t>(std::stoul(s.substr(2), nullptr, 2) & 0xFF);
    }
    return static_cast<uint8_t>(std::stoul(s, nullptr, 0) & 0xFF);
}

static tsn_gptp_port_role_t parse_gptp_role(const std::string& s) {
    if (s == "master") return TSN_GPTP_ROLE_MASTER;
    if (s == "slave") return TSN_GPTP_ROLE_SLAVE;
    return TSN_GPTP_ROLE_DISABLED;
}

tsn_mac_t parse_mac(const std::string& s) {
    tsn_mac_t mac{};
    unsigned int b[6] = {};
    if (std::sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",
                    &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        throw std::runtime_error("Invalid MAC address format: " + s);
    }
    for (int i = 0; i < 6; ++i) {
        mac.bytes[i] = static_cast<uint8_t>(b[i] & 0xFFu);
    }
    return mac;
}

Scenario Scenario::load_from_file(const std::string& json_path) {
    return load_from_json(Json::parse_file(json_path));
}

Scenario Scenario::load_from_json(const Json& root) {
    Scenario sc;

    const auto& sim_cfg = root.at("simulation");
    uint64_t seed = static_cast<uint64_t>(sim_cfg.get_or<int64_t>("seed", 42));
    double duration_ms = sim_cfg.get_or<double>("duration_ms", 10.0);
    sc.duration_ps_ = static_cast<tsn_time_ps_t>(duration_ms * static_cast<double>(TSN_PS_PER_MS));
    sc.kernel_ = std::make_unique<SimKernel>(seed);
    sc.cnc_ = std::make_unique<CncController>();

    /* 1. Create nodes */
    uint32_t next_node_id = 1;
    std::map<std::string, tsn_mac_t> end_station_macs;

    for (const auto& n : root.at("nodes").as_array()) {
        std::string name = n.at("name").as_string();
        std::string type = n.at("type").as_string();
        double drift_ppb = n.get_or<double>("clock_drift_ppb", 0.0);
        int64_t offset_ns = n.get_or<int64_t>("clock_offset_ns", 0);
        LocalClock clk(offset_ns * TSN_PS_PER_NS, drift_ppb);

        uint32_t nid = next_node_id++;
        sc.telemetry_.register_node(nid, name);

        if (type == "end_station") {
            tsn_mac_t mac = parse_mac(n.at("mac").as_string());
            auto es = std::make_unique<EndStation>(name, nid, mac, *sc.kernel_, clk, &sc.telemetry_);
            sc.node_by_name_[name] = es.get();
            end_station_macs[name] = mac;
            sc.end_stations_.push_back(std::move(es));
        } else if (type == "switch") {
            uint8_t num_ports = static_cast<uint8_t>(n.get_or<int64_t>("ports", 4));
            int64_t fabric_ns = n.get_or<int64_t>("fabric_delay_ns", 1000);
            uint16_t q_cap = static_cast<uint16_t>(n.get_or<int64_t>("queue_capacity", TSN_QUEUE_CAPACITY));
            auto sw = std::make_unique<SwitchNode>(name, nid, num_ports,
                                                   fabric_ns * TSN_PS_PER_NS, q_cap,
                                                   *sc.kernel_, clk, &sc.telemetry_);
            sc.cnc_->topology().switch_fabric_delay_ps[name] = fabric_ns * TSN_PS_PER_NS;
            sc.node_by_name_[name] = sw.get();
            sc.switches_.push_back(std::move(sw));
        } else {
            throw std::runtime_error("Unknown node type: " + type);
        }
    }

    /* 2. Create links */
    for (const auto& l : root.at("links").as_array()) {
        LinkConfig cfg;
        cfg.name = l.at("name").as_string();
        double rate_mbps = l.get_or<double>("rate_mbps", 1000.0);
        cfg.rate_bps = static_cast<uint64_t>(rate_mbps * 1e6);
        int64_t delay_ns = l.get_or<int64_t>("propagation_delay_ns", 100);
        cfg.propagation_delay_ps = delay_ns * TSN_PS_PER_NS;
        cfg.ber = l.get_or<double>("ber", 0.0);
        cfg.drop_every_n = static_cast<uint32_t>(l.get_or<int64_t>("drop_every_n", 0));

        const auto& a_obj = l.at("a");
        const auto& b_obj = l.at("b");
        SimNode* node_a = sc.find_node(a_obj.at("node").as_string());
        SimNode* node_b = sc.find_node(b_obj.at("node").as_string());
        if (!node_a || !node_b) {
            throw std::runtime_error("Link " + cfg.name + " references unknown node");
        }
        uint8_t port_a = static_cast<uint8_t>(a_obj.get_or<int64_t>("port", 0));
        uint8_t port_b = static_cast<uint8_t>(b_obj.get_or<int64_t>("port", 0));

        auto link = std::make_unique<Link>(*sc.kernel_, cfg,
                                           Link::Endpoint{node_a, port_a},
                                           Link::Endpoint{node_b, port_b},
                                           &sc.telemetry_);
        node_a->attach_link(port_a, link.get());
        node_b->attach_link(port_b, link.get());

        if (auto* sw_a = dynamic_cast<SwitchNode*>(node_a)) {
            sw_a->config_port(port_a, cfg.rate_bps);
        }
        if (auto* sw_b = dynamic_cast<SwitchNode*>(node_b)) {
            sw_b->config_port(port_b, cfg.rate_bps);
        }

        sc.cnc_->topology().add_bidirectional_link(
            cfg.name, node_a->name(), port_a, node_b->name(), port_b,
            cfg.rate_bps, cfg.propagation_delay_ps);

        sc.link_by_name_[cfg.name] = link.get();
        sc.links_.push_back(std::move(link));
    }

    /* 3. Load static FDB entries on switches */
    if (root.contains("fdb")) {
        for (const auto& entry : root.at("fdb").as_array()) {
            std::string sw_name = entry.at("switch").as_string();
            auto* sw = dynamic_cast<SwitchNode*>(sc.find_node(sw_name));
            if (!sw) {
                throw std::runtime_error("FDB references unknown switch: " + sw_name);
            }
            tsn_mac_t mac = parse_mac(entry.at("dst_mac").as_string());
            uint16_t vid = static_cast<uint16_t>(entry.get_or<int64_t>("vid", 0));
            uint32_t port_mask = 0;
            for (const auto& p : entry.at("egress_ports").as_array()) {
                port_mask |= (1u << static_cast<uint8_t>(p.as_int()));
            }
            sw->add_fdb_entry(mac, vid, port_mask);
        }
    }

    /* 4. Load Talker streams */
    if (root.contains("streams")) {
        uint32_t auto_id = 1;
        for (const auto& s : root.at("streams").as_array()) {
            StreamSpec spec;
            spec.stream_id = static_cast<uint32_t>(s.get_or<int64_t>("id", auto_id++));
            spec.name = s.get_or<std::string>("name", "stream_" + std::to_string(spec.stream_id));
            spec.talker = s.at("talker").as_string();
            spec.listener = s.at("listener").as_string();

            if (s.contains("dst_mac")) {
                spec.dst_mac = parse_mac(s.at("dst_mac").as_string());
            } else {
                auto it = end_station_macs.find(spec.listener);
                if (it == end_station_macs.end()) {
                    throw std::runtime_error("Stream " + spec.name + " listener not found: " + spec.listener);
                }
                spec.dst_mac = it->second;
            }

            spec.vid = static_cast<uint16_t>(s.get_or<int64_t>("vid", 1));
            spec.pcp = static_cast<uint8_t>(s.get_or<int64_t>("pcp", 0));
            spec.frame_bytes = static_cast<uint16_t>(s.get_or<int64_t>("frame_bytes", 128));
            double period_us = s.get_or<double>("period_us", 1000.0);
            spec.period_ps = static_cast<tsn_time_ps_t>(period_us * static_cast<double>(TSN_PS_PER_US));
            double start_us = s.get_or<double>("start_us", 0.0);
            spec.start_time_ps = static_cast<tsn_time_ps_t>(start_us * static_cast<double>(TSN_PS_PER_US));
            double deadline_us = s.get_or<double>("deadline_us", 0.0);
            spec.deadline_ps = static_cast<tsn_time_ps_t>(deadline_us * static_cast<double>(TSN_PS_PER_US));
            spec.burst_count = static_cast<uint32_t>(s.get_or<int64_t>("burst_count", 1));

            auto* talker = dynamic_cast<EndStation*>(sc.find_node(spec.talker));
            if (!talker) {
                throw std::runtime_error("Stream " + spec.name + " talker not found: " + spec.talker);
            }
            sc.telemetry_.register_stream(spec.stream_id, spec.name);

            /* Register with CNC/CUC if qcc auto-config is enabled */
            CncStreamReq creq;
            creq.spec = spec;
            creq.is_scheduled_tt = s.get_or<bool>("scheduled", spec.pcp >= 6);
            creq.redundancy_paths = static_cast<int>(s.get_or<int64_t>("redundancy_paths", 1));
            creq.avb_idle_slope_mbps = s.get_or<double>("avb_idle_slope_mbps", 0.0);
            sc.cnc_->register_stream(creq);

            talker->add_talker_stream(spec);
        }
    }

    /* 4b. IEEE 802.1Qcc (CNC/CUC) auto-configuration & dynamic reconfiguration */
    if (root.contains("qcc") && root.at("qcc").get_or<bool>("enabled", false)) {
        const auto& qcc = root.at("qcc");
        SchedulerOptions opts;
        opts.guard_margin_ps = static_cast<tsn_time_ps_t>(
            qcc.get_or<double>("guard_margin_us", 2.0) * static_cast<double>(TSN_PS_PER_US));
        opts.use_preemption_hold = qcc.get_or<bool>("use_preemption_hold", false);
        opts.guard_band_enabled = qcc.get_or<bool>("guard_band", true);
        sc.cnc_->set_options(opts);

        auto res = sc.cnc_->deploy(sc.node_by_name_, 0, true);
        if (!res.success) {
            throw std::runtime_error("802.1Qcc CNC deploy failed: " + res.error_msg);
        }

        /* Schedule mid-simulation dynamic stream additions */
        if (qcc.contains("dynamic_streams")) {
            for (const auto& ds : qcc.at("dynamic_streams").as_array()) {
                double at_ms = ds.at("at_ms").as_double();
                tsn_time_ps_t at_ps = static_cast<tsn_time_ps_t>(
                    at_ms * static_cast<double>(TSN_PS_PER_MS));

                StreamSpec spec;
                spec.stream_id = static_cast<uint32_t>(ds.at("id").as_int());
                spec.name = ds.get_or<std::string>("name", "dyn_stream_" + std::to_string(spec.stream_id));
                spec.talker = ds.at("talker").as_string();
                spec.listener = ds.at("listener").as_string();
                auto it_mac = end_station_macs.find(spec.listener);
                if (it_mac == end_station_macs.end()) {
                    throw std::runtime_error("Dynamic stream listener not found: " + spec.listener);
                }
                spec.dst_mac = it_mac->second;
                spec.vid = static_cast<uint16_t>(ds.get_or<int64_t>("vid", 1));
                spec.pcp = static_cast<uint8_t>(ds.get_or<int64_t>("pcp", 6));
                spec.frame_bytes = static_cast<uint16_t>(ds.get_or<int64_t>("frame_bytes", 128));
                spec.period_ps = static_cast<tsn_time_ps_t>(
                    ds.get_or<double>("period_us", 500.0) * static_cast<double>(TSN_PS_PER_US));
                spec.start_time_ps = at_ps + static_cast<tsn_time_ps_t>(
                    ds.get_or<double>("offset_us", 20.0) * static_cast<double>(TSN_PS_PER_US));
                spec.deadline_ps = static_cast<tsn_time_ps_t>(
                    ds.get_or<double>("deadline_us", 50.0) * static_cast<double>(TSN_PS_PER_US));
                spec.burst_count = static_cast<uint32_t>(ds.get_or<int64_t>("burst_count", 1));

                CncStreamReq creq;
                creq.spec = spec;
                creq.is_scheduled_tt = ds.get_or<bool>("scheduled", spec.pcp >= 6);
                creq.redundancy_paths = static_cast<int>(ds.get_or<int64_t>("redundancy_paths", 1));
                creq.avb_idle_slope_mbps = ds.get_or<double>("avb_idle_slope_mbps", 0.0);

                sc.telemetry_.register_stream(spec.stream_id, spec.name);

                /* Capture raw pointers stable across Scenario move (unique_ptr heap objects) */
                CncController* cnc_ptr = sc.cnc_.get();
                auto nodes_copy = sc.node_by_name_;
                tsn_time_ps_t sim_end = sc.duration_ps_;
                auto* talker = dynamic_cast<EndStation*>(sc.find_node(spec.talker));
                if (!talker) {
                    throw std::runtime_error("Dynamic stream talker not found: " + spec.talker);
                }

                sc.kernel_->schedule_at(at_ps, [cnc_ptr, nodes_copy, creq, talker, at_ps, sim_end]() {
                    cnc_ptr->register_stream(creq);
                    cnc_ptr->deploy(nodes_copy, at_ps, true);
                    talker->start_single_talker(creq.spec, sim_end);
                });
            }
        }
    }

    /* 5. Load 802.1Qbv (TAS) schedules */
    if (root.contains("qbv")) {
        for (const auto& q : root.at("qbv").as_array()) {
            std::string sw_name = q.at("switch").as_string();
            auto* sw = dynamic_cast<SwitchNode*>(sc.find_node(sw_name));
            if (!sw) {
                throw std::runtime_error("Qbv references unknown switch: " + sw_name);
            }
            uint8_t port_id = static_cast<uint8_t>(q.at("port").as_int());
            bool guard_band = q.get_or<bool>("guard_band", true);

            tsn_qbv_config_t cfg{};
            cfg.base_time_ps = static_cast<tsn_time_ps_t>(
                q.get_or<double>("base_time_us", 0.0) * static_cast<double>(TSN_PS_PER_US));
            cfg.cycle_time_ps = static_cast<tsn_time_ps_t>(
                q.at("cycle_time_us").as_double() * static_cast<double>(TSN_PS_PER_US));
            cfg.initial_gate_states = 0xFFu;

            const auto& gcl = q.at("gcl").as_array();
            cfg.entry_count = static_cast<uint16_t>(gcl.size());
            for (size_t i = 0; i < gcl.size() && i < TSN_QBV_MAX_GCL_ENTRIES; ++i) {
                cfg.entries[i].gate_mask = parse_gate_mask(gcl[i].at("gate_mask"));
                cfg.entries[i].duration_ps = static_cast<tsn_time_ps_t>(
                    gcl[i].at("duration_us").as_double() * static_cast<double>(TSN_PS_PER_US));
                std::string op_s = gcl[i].get_or<std::string>("op", "set");
                if (op_s == "hold") {
                    cfg.entries[i].op = TSN_QBV_OP_SET_AND_HOLD_MAC;
                } else if (op_s == "release") {
                    cfg.entries[i].op = TSN_QBV_OP_SET_AND_RELEASE_MAC;
                } else {
                    cfg.entries[i].op = TSN_QBV_OP_SET_GATE_STATES;
                }
            }
            sw->config_qbv(port_id, cfg, guard_band);
        }
    }

    /* 6. Load 802.1Qav (CBS) shapers */
    if (root.contains("qav")) {
        for (const auto& q : root.at("qav").as_array()) {
            std::string sw_name = q.at("switch").as_string();
            auto* sw = dynamic_cast<SwitchNode*>(sc.find_node(sw_name));
            if (!sw) {
                throw std::runtime_error("Qav references unknown switch: " + sw_name);
            }
            uint8_t port_id = static_cast<uint8_t>(q.at("port").as_int());
            uint8_t tc = static_cast<uint8_t>(q.at("tc").as_int());
            double idle_mbps = q.at("idle_slope_mbps").as_double();
            double hi_bits = q.get_or<double>("hi_credit_bits", 0.0);
            double lo_bits = q.get_or<double>("lo_credit_bits", 0.0);
            sw->config_qav(port_id, tc, idle_mbps * 1e6, hi_bits, lo_bits);
        }
    }

    /* 6b. Load 802.1Qbu / 802.3br (Frame Preemption) config */
    if (root.contains("qbu")) {
        for (const auto& q : root.at("qbu").as_array()) {
            std::string sw_name = q.at("switch").as_string();
            auto* sw = dynamic_cast<SwitchNode*>(sc.find_node(sw_name));
            if (!sw) {
                throw std::runtime_error("Qbu references unknown switch: " + sw_name);
            }
            uint8_t port_id = static_cast<uint8_t>(q.at("port").as_int());
            uint8_t preempt_mask = parse_gate_mask(q.at("preemptable_mask"));
            uint8_t add_frag = static_cast<uint8_t>(q.get_or<int64_t>("add_frag_size", 0));
            sw->config_qbu(port_id, preempt_mask, add_frag);
        }
    }

    /* 6c. Load 802.1CB (FRER) generation & recovery rules */
    if (root.contains("frer")) {
        const auto& frer = root.at("frer");
        if (frer.contains("generate")) {
            for (const auto& g : frer.at("generate").as_array()) {
                std::string sw_name = g.at("switch").as_string();
                auto* sw = dynamic_cast<SwitchNode*>(sc.find_node(sw_name));
                if (!sw) {
                    throw std::runtime_error("FRER generate references unknown switch: " + sw_name);
                }
                tsn_mac_t mac = parse_mac(g.at("dst_mac").as_string());
                uint16_t vid = static_cast<uint16_t>(g.get_or<int64_t>("vid", 0));
                uint32_t mask = 0;
                for (const auto& p : g.at("split_ports").as_array()) {
                    mask |= (1u << static_cast<uint8_t>(p.as_int()));
                }
                sw->add_frer_gen(mac, vid, mask);
            }
        }
        if (frer.contains("recover")) {
            for (const auto& r : frer.at("recover").as_array()) {
                std::string sw_name = r.at("switch").as_string();
                auto* sw = dynamic_cast<SwitchNode*>(sc.find_node(sw_name));
                if (!sw) {
                    throw std::runtime_error("FRER recover references unknown switch: " + sw_name);
                }
                tsn_mac_t mac = parse_mac(r.at("dst_mac").as_string());
                uint16_t vid = static_cast<uint16_t>(r.get_or<int64_t>("vid", 0));
                std::string algo_s = r.get_or<std::string>("algorithm", "vector");
                tsn_frer_algo_t algo = (algo_s == "match") ? TSN_FRER_ALGO_MATCH : TSN_FRER_ALGO_VECTOR;
                uint8_t hist = static_cast<uint8_t>(r.get_or<int64_t>("history_length", 16));
                bool strip = r.get_or<bool>("strip_rtag", true);
                double reset_ms = r.get_or<double>("reset_timeout_ms", 0.0);
                tsn_time_ps_t reset_ps = static_cast<tsn_time_ps_t>(
                    reset_ms * static_cast<double>(TSN_PS_PER_MS));
                sw->add_frer_recov(mac, vid, algo, hist, strip, reset_ps);
            }
        }
    }

    /* 7. Load 802.1AS (gPTP) configuration */
    if (root.contains("gptp") && root.at("gptp").get_or<bool>("enabled", false)) {
        const auto& g = root.at("gptp");
        std::string gm_name = g.at("gm_node").as_string();
        double sync_ms = g.get_or<double>("sync_interval_ms", 1.0);
        double pdelay_ms = g.get_or<double>("pdelay_interval_ms", 1.0);
        double kp = g.get_or<double>("servo_kp", 0.7);
        double ki = g.get_or<double>("servo_ki", 0.3);

        for (const auto& nr : g.at("nodes").as_array()) {
            std::string nname = nr.at("node").as_string();
            SimNode* node = sc.find_node(nname);
            if (!node) {
                throw std::runtime_error("gPTP references unknown node: " + nname);
            }
            tsn_gptp_config_t gcfg{};
            gcfg.is_gm = (nname == gm_name);
            gcfg.sync_interval_ps = static_cast<tsn_time_ps_t>(sync_ms * static_cast<double>(TSN_PS_PER_MS));
            gcfg.pdelay_interval_ps = static_cast<tsn_time_ps_t>(pdelay_ms * static_cast<double>(TSN_PS_PER_MS));
            gcfg.servo_kp = kp;
            gcfg.servo_ki = ki;

            if (auto* sw = dynamic_cast<SwitchNode*>(node)) {
                gcfg.num_ports = sw->bridge().num_ports;
                tsn_mac_t bmac = {{0x02, 0x10, 0x00, 0x00, 0x00, static_cast<uint8_t>(sw->node_id())}};
                sw->init_gptp(gcfg, bmac);
                for (const auto& pr : nr.at("ports").as_array()) {
                    uint8_t pid = static_cast<uint8_t>(pr.at("port").as_int());
                    sw->set_gptp_port_role(pid, parse_gptp_role(pr.at("role").as_string()));
                }
            } else if (auto* es = dynamic_cast<EndStation*>(node)) {
                gcfg.num_ports = 1;
                tsn_gptp_port_role_t role = TSN_GPTP_ROLE_SLAVE;
                if (nr.contains("ports") && !nr.at("ports").as_array().empty()) {
                    role = parse_gptp_role(nr.at("ports").as_array()[0].at("role").as_string());
                }
                es->init_gptp(gcfg, role);
            }
        }
    }

    /* 8. Load scheduled fault events */
    if (root.contains("faults")) {
        for (const auto& f : root.at("faults").as_array()) {
            std::string link_name = f.at("link").as_string();
            Link* link = sc.find_link(link_name);
            if (!link) {
                throw std::runtime_error("Fault references unknown link: " + link_name);
            }
            double at_ms = f.at("at_ms").as_double();
            std::string action = f.at("action").as_string();
            tsn_time_ps_t t_ps = static_cast<tsn_time_ps_t>(at_ms * static_cast<double>(TSN_PS_PER_MS));
            bool up = (action == "up");
            sc.kernel_->schedule_at(t_ps, [link, up]() {
                link->set_up(up);
            });
        }
    }

    return sc;
}

void Scenario::run() {
    for (auto& sw : switches_) {
        sw->start_gptp();
    }
    for (auto& es : end_stations_) {
        es->start_gptp();
        es->start_talkers(duration_ps_);
    }
    kernel_->run_until(duration_ps_);
}

SimNode* Scenario::find_node(const std::string& name) const {
    auto it = node_by_name_.find(name);
    return it != node_by_name_.end() ? it->second : nullptr;
}

Link* Scenario::find_link(const std::string& name) const {
    auto it = link_by_name_.find(name);
    return it != link_by_name_.end() ? it->second : nullptr;
}

} // namespace sim
