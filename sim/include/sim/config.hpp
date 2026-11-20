#pragma once

#include "sim/cnc.hpp"
#include "sim/kernel.hpp"
#include "sim/link.hpp"
#include "sim/node.hpp"
#include "sim/telemetry.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sim {

/**
 * Holds a fully constructed TSN simulation scenario loaded from JSON.
 */
class Scenario {
public:
    static Scenario load_from_file(const std::string& json_path);
    static Scenario load_from_json(const Json& root);

    void run();

    SimKernel& kernel() { return *kernel_; }
    const Telemetry& telemetry() const { return telemetry_; }
    tsn_time_ps_t duration_ps() const { return duration_ps_; }
    CncController* cnc() const { return cnc_.get(); }

    SimNode* find_node(const std::string& name) const;
    Link* find_link(const std::string& name) const;

private:
    std::unique_ptr<SimKernel>     kernel_;
    std::unique_ptr<CncController> cnc_;
    Telemetry                      telemetry_;
    tsn_time_ps_t                  duration_ps_ = 0;

    std::vector<std::unique_ptr<EndStation>> end_stations_;
    std::vector<std::unique_ptr<SwitchNode>> switches_;
    std::vector<std::unique_ptr<Link>>       links_;

    std::map<std::string, SimNode*> node_by_name_;
    std::map<std::string, Link*>    link_by_name_;
};

tsn_mac_t parse_mac(const std::string& s);

} // namespace sim
