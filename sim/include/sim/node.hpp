#pragma once

#include "sim/clock.hpp"
#include "sim/kernel.hpp"
#include "sim/link.hpp"
#include "tsn/bridge.h"
#include "tsn/frame.h"
#include "tsn/hal.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim {

class Telemetry;

struct StreamSpec {
    uint32_t      stream_id = 0;
    std::string   name;
    std::string   talker;
    std::string   listener;
    tsn_mac_t     dst_mac{};
    uint16_t      vid = 1;
    uint8_t       pcp = 0;
    uint16_t      frame_bytes = 128;
    tsn_time_ps_t period_ps = 1000 * TSN_PS_PER_US;
    tsn_time_ps_t start_time_ps = 0;
    tsn_time_ps_t deadline_ps = 0; /* 0 = no deadline */
    uint32_t      burst_count = 1;
};

/**
 * Base class for simulated network nodes (EndStation, SwitchNode).
 * Owns a LocalClock, attached Links, and provides a tsn_hal_t adapter.
 */
class SimNode {
public:
    SimNode(std::string name, uint32_t node_id, SimKernel& kernel,
            LocalClock clock, Telemetry* telemetry);
    virtual ~SimNode() = default;

    const std::string& name() const { return name_; }
    uint32_t node_id() const { return node_id_; }
    LocalClock& clock() { return clock_; }
    const LocalClock& clock() const { return clock_; }

    void attach_link(uint8_t port_id, Link* link);
    Link* get_link(uint8_t port_id) const;

    virtual void on_rx_frame(uint8_t ingress_port, const tsn_frame_t& frame) = 0;
    virtual void on_tx_complete(uint8_t egress_port) = 0;
    virtual void on_tx_timestamp(uint8_t egress_port, const tsn_frame_t& frame,
                                 tsn_time_ps_t tx_ts_ps) {
        (void)egress_port;
        (void)frame;
        (void)tx_ts_ps;
    }

protected:
    tsn_hal_t make_hal();

    std::string name_;
    uint32_t    node_id_;
    SimKernel&  kernel_;
    LocalClock  clock_;
    Telemetry*  telemetry_;
    std::vector<Link*> ports_;

private:
    static tsn_time_ps_t hal_get_time_ps(void* ctx);
    static void hal_adjust_clock(void* ctx, int64_t offset_ps, double rate_ppb);
    static uint64_t hal_schedule_timer(void* ctx, tsn_time_ps_t target_local_ps,
                                       tsn_timer_cb_t cb, void* arg);
    static void hal_cancel_timer(void* ctx, uint64_t timer_id);
    static tsn_status_t hal_tx_frame(void* ctx, uint8_t port_id, const tsn_frame_t* frame);
    static tsn_status_t hal_preempt_tx(void* ctx, uint8_t port_id, uint16_t cut_bytes);
    static void hal_emit_metric(void* ctx, const tsn_metric_event_t* evt);
};

/**
 * TSN End Station hosting Talker traffic generators and Listener sinks.
 */
class EndStation : public SimNode {
public:
    EndStation(std::string name, uint32_t node_id, tsn_mac_t mac,
               SimKernel& kernel, LocalClock clock, Telemetry* telemetry);

    const tsn_mac_t& mac() const { return mac_; }

    void add_talker_stream(const StreamSpec& spec);
    void start_talkers(tsn_time_ps_t sim_end_ps);
    void start_single_talker(const StreamSpec& spec, tsn_time_ps_t sim_end_ps);

    void init_gptp(const tsn_gptp_config_t& cfg, tsn_gptp_port_role_t port0_role);
    void start_gptp();
    const tsn_gptp_node_t& gptp() const { return gptp_; }

    void on_rx_frame(uint8_t ingress_port, const tsn_frame_t& frame) override;
    void on_tx_complete(uint8_t egress_port) override;
    void on_tx_timestamp(uint8_t egress_port, const tsn_frame_t& frame,
                         tsn_time_ps_t tx_ts_ps) override;

private:
    void schedule_next_burst(size_t stream_idx, tsn_time_ps_t sim_end_ps);
    void try_tx();
    static tsn_status_t es_gptp_tx_adapter(void* ctx, uint8_t port_id, const tsn_frame_t* frame);

    tsn_mac_t mac_;
    std::vector<StreamSpec> talker_streams_;
    std::vector<uint32_t>   stream_seq_;
    tsn_queue_t             gptp_queue_{};
    tsn_queue_t             tx_queues_[TSN_NUM_TRAFFIC_CLASSES];
    tsn_gptp_node_t         gptp_{};
    tsn_hal_t               gptp_hal_{};
    tsn_qbu_port_t          rx_qbu_{};
    bool                    tx_busy_ = false;
    static uint64_t         global_frame_uid_;
};

/**
 * TSN Bridge / Switch wrapping the portable C11 tsn_bridge_t core.
 */
class SwitchNode : public SimNode {
public:
    SwitchNode(std::string name, uint32_t node_id, uint8_t num_ports,
               tsn_time_ps_t fabric_delay_ps, uint16_t queue_capacity,
               SimKernel& kernel, LocalClock clock, Telemetry* telemetry);

    tsn_bridge_t& bridge() { return bridge_; }
    const tsn_bridge_t& bridge() const { return bridge_; }

    void config_port(uint8_t port_id, uint64_t link_rate_bps);
    void add_fdb_entry(const tsn_mac_t& dst_mac, uint16_t vid, uint32_t port_mask);
    void config_qbv(uint8_t port_id, const tsn_qbv_config_t& admin, bool guard_band_enabled);
    void config_qav(uint8_t port_id, uint8_t tc, double idle_slope_bps,
                    double hi_credit_bits = 0.0, double lo_credit_bits = 0.0);
    void config_qbu(uint8_t port_id, uint8_t preemptable_mask, uint8_t add_frag_size = 0);
    void add_frer_gen(const tsn_mac_t& dst_mac, uint16_t vid, uint32_t split_port_mask);
    void add_frer_recov(const tsn_mac_t& dst_mac, uint16_t vid, tsn_frer_algo_t algo,
                        uint8_t history_length, bool strip_rtag,
                        tsn_time_ps_t reset_timeout_ps = 0);
    void init_gptp(const tsn_gptp_config_t& cfg, const tsn_mac_t& bridge_mac);
    void set_gptp_port_role(uint8_t port_id, tsn_gptp_port_role_t role);
    void start_gptp();

    void on_rx_frame(uint8_t ingress_port, const tsn_frame_t& frame) override;
    void on_tx_complete(uint8_t egress_port) override;
    void on_tx_timestamp(uint8_t egress_port, const tsn_frame_t& frame,
                         tsn_time_ps_t tx_ts_ps) override;

private:
    tsn_bridge_t  bridge_{};
    tsn_time_ps_t fabric_delay_ps_;
    uint16_t      queue_capacity_;
};

} // namespace sim
