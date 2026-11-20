#pragma once

#include "sim/json.hpp"
#include "tsn/frame.h"
#include "tsn/types.h"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace sim {

struct FrameRecord {
    uint32_t      stream_id;
    uint64_t      frame_uid;
    uint32_t      seq_in_stream;
    uint8_t       pcp;
    uint16_t      length_bytes;
    tsn_time_ps_t gen_time_ps;
    tsn_time_ps_t rx_time_ps;
    tsn_time_ps_t latency_ps;
    tsn_time_ps_t deadline_ps;
    uint8_t       hop_count;
    bool          missed_deadline;
};

struct TraceSpan {
    std::string   name;
    std::string   category;
    uint32_t      pid; /* Node ID */
    uint32_t      tid; /* Port ID */
    tsn_time_ps_t start_ps;
    tsn_time_ps_t dur_ps;
    uint32_t      stream_id;
    uint64_t      frame_uid;
    uint8_t       pcp;
};

struct StreamStats {
    uint32_t      stream_id = 0;
    std::string   stream_name;
    uint64_t      tx_frames = 0;
    uint64_t      rx_frames = 0;
    uint64_t      deadline_misses = 0;
    tsn_time_ps_t min_latency_ps = 0;
    tsn_time_ps_t max_latency_ps = 0;
    double        avg_latency_ps = 0.0;
    tsn_time_ps_t p99_latency_ps = 0;
    tsn_time_ps_t jitter_ps = 0; /* max - min */
};

struct GptpSyncStats {
    uint32_t    node_id = 0;
    std::string node_name;
    uint64_t    sync_count = 0;
    int64_t     max_abs_err_ps = 0;
    int64_t     last_err_ps = 0;
};

struct QbvGateEvent {
    uint32_t      node_id;
    uint8_t       port_id;
    tsn_time_ps_t timestamp_ps;
    uint8_t       gate_mask;
};

class Telemetry {
public:
    void register_stream(uint32_t stream_id, const std::string& name);
    void register_node(uint32_t node_id, const std::string& name);

    void record_tx(uint32_t stream_id);
    void record_rx(const tsn_frame_t& frame, tsn_time_ps_t rx_time_ps);
    size_t record_tx_span(uint32_t node_id, uint8_t port_id, const tsn_frame_t& frame,
                          tsn_time_ps_t start_ps, tsn_time_ps_t dur_ps);
    void update_tx_span_preempted(size_t span_idx, tsn_time_ps_t new_dur_ps);
    void record_link_drop(const std::string& link_name);
    void record_gptp_sync(uint32_t node_id, int64_t offset_err_ps);
    void record_qbv_gate_change(uint32_t node_id, uint8_t port_id,
                                tsn_time_ps_t timestamp_ps, uint8_t gate_mask);

    std::vector<StreamStats> compute_stream_stats() const;
    std::vector<GptpSyncStats> compute_gptp_stats() const;
    const std::vector<FrameRecord>& frame_records() const { return records_; }
    uint64_t total_link_drops() const { return link_drops_; }

    Json to_summary_json(tsn_time_ps_t sim_duration_ps, uint64_t executed_events) const;
    void print_summary(std::ostream& os, tsn_time_ps_t sim_duration_ps,
                       uint64_t executed_events) const;
    void write_csv(const std::string& path) const;
    void write_perfetto_trace(const std::string& path) const;

private:
    std::map<uint32_t, std::string>   stream_names_;
    std::map<uint32_t, std::string>   node_names_;
    std::map<uint32_t, uint64_t>      tx_counts_;
    std::map<uint32_t, GptpSyncStats> gptp_stats_;
    std::vector<FrameRecord>          records_;
    std::vector<TraceSpan>            spans_;
    std::vector<QbvGateEvent>         qbv_events_;
    uint64_t                          link_drops_ = 0;
};

} // namespace sim
