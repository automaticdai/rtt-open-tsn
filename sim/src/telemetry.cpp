#include "sim/telemetry.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace sim {

void Telemetry::register_stream(uint32_t stream_id, const std::string& name) {
    stream_names_[stream_id] = name;
    if (tx_counts_.find(stream_id) == tx_counts_.end()) {
        tx_counts_[stream_id] = 0;
    }
}

void Telemetry::register_node(uint32_t node_id, const std::string& name) {
    node_names_[node_id] = name;
}

void Telemetry::record_tx(uint32_t stream_id) {
    tx_counts_[stream_id]++;
}

void Telemetry::record_rx(const tsn_frame_t& frame, tsn_time_ps_t rx_time_ps) {
    tsn_time_ps_t latency_ps = rx_time_ps - frame.gen_time_ps;
    bool missed = (frame.deadline_ps > 0) && (latency_ps > frame.deadline_ps);
    records_.push_back(FrameRecord{
        frame.stream_id,
        frame.frame_uid,
        frame.seq_in_stream,
        frame.pcp,
        frame.length_bytes,
        frame.gen_time_ps,
        rx_time_ps,
        latency_ps,
        frame.deadline_ps,
        frame.hop_count,
        missed
    });
}

size_t Telemetry::record_tx_span(uint32_t node_id, uint8_t port_id, const tsn_frame_t& frame,
                                 tsn_time_ps_t start_ps, tsn_time_ps_t dur_ps) {
    std::string name = "S" + std::to_string(frame.stream_id) + "#" +
                       std::to_string(frame.seq_in_stream);
    if (frame.smd_type == TSN_SMD_C) {
        name += "[C" + std::to_string(frame.frag_count) + "]";
    }
    spans_.push_back(TraceSpan{
        std::move(name),
        "pcp" + std::to_string(frame.pcp),
        node_id,
        port_id,
        start_ps,
        dur_ps,
        frame.stream_id,
        frame.frame_uid,
        frame.pcp
    });
    return spans_.size() - 1u;
}

void Telemetry::update_tx_span_preempted(size_t span_idx, tsn_time_ps_t new_dur_ps) {
    if (span_idx < spans_.size()) {
        spans_[span_idx].dur_ps = new_dur_ps;
        spans_[span_idx].name += "[PREEMPT]";
    }
}

void Telemetry::record_link_drop(const std::string& link_name) {
    (void)link_name;
    link_drops_++;
}

void Telemetry::record_gptp_sync(uint32_t node_id, int64_t offset_err_ps) {
    auto& st = gptp_stats_[node_id];
    if (st.sync_count == 0) {
        st.node_id = node_id;
        auto it = node_names_.find(node_id);
        st.node_name = (it != node_names_.end()) ? it->second : ("node_" + std::to_string(node_id));
    }
    st.sync_count++;
    st.last_err_ps = offset_err_ps;
    /* Ignore initial step error (sync #1) when computing steady-state max error */
    int64_t abs_err = offset_err_ps < 0 ? -offset_err_ps : offset_err_ps;
    if (st.sync_count > 1 && abs_err > st.max_abs_err_ps) {
        st.max_abs_err_ps = abs_err;
    }
}

void Telemetry::record_qbv_gate_change(uint32_t node_id, uint8_t port_id,
                                       tsn_time_ps_t timestamp_ps, uint8_t gate_mask) {
    qbv_events_.push_back(QbvGateEvent{node_id, port_id, timestamp_ps, gate_mask});
}

std::vector<GptpSyncStats> Telemetry::compute_gptp_stats() const {
    std::vector<GptpSyncStats> out;
    for (const auto& [_, st] : gptp_stats_) {
        out.push_back(st);
    }
    return out;
}

std::vector<StreamStats> Telemetry::compute_stream_stats() const {
    std::map<uint32_t, std::vector<tsn_time_ps_t>> latencies;
    std::map<uint32_t, uint64_t> misses;

    for (const auto& [sid, _] : tx_counts_) {
        latencies[sid];
        misses[sid] = 0;
    }

    for (const auto& rec : records_) {
        latencies[rec.stream_id].push_back(rec.latency_ps);
        if (rec.missed_deadline) {
            misses[rec.stream_id]++;
        }
    }

    std::vector<StreamStats> out;
    for (auto& [sid, lats] : latencies) {
        StreamStats st;
        st.stream_id = sid;
        auto name_it = stream_names_.find(sid);
        st.stream_name = (name_it != stream_names_.end()) ? name_it->second
                                                          : ("stream_" + std::to_string(sid));
        auto tx_it = tx_counts_.find(sid);
        st.tx_frames = (tx_it != tx_counts_.end()) ? tx_it->second : 0;
        st.rx_frames = lats.size();
        st.deadline_misses = misses[sid];

        if (!lats.empty()) {
            std::sort(lats.begin(), lats.end());
            st.min_latency_ps = lats.front();
            st.max_latency_ps = lats.back();
            st.jitter_ps = st.max_latency_ps - st.min_latency_ps;

            double sum = 0.0;
            for (auto l : lats) sum += static_cast<double>(l);
            st.avg_latency_ps = sum / static_cast<double>(lats.size());

            size_t p99_idx = static_cast<size_t>(0.99 * static_cast<double>(lats.size() - 1));
            st.p99_latency_ps = lats[p99_idx];
        }
        out.push_back(std::move(st));
    }
    return out;
}

Json Telemetry::to_summary_json(tsn_time_ps_t sim_duration_ps, uint64_t executed_events) const {
    Json root;
    root["sim_duration_us"] = static_cast<double>(sim_duration_ps) / static_cast<double>(TSN_PS_PER_US);
    root["executed_events"] = executed_events;
    root["total_link_drops"] = link_drops_;

    Json streams_arr = Json::Array{};
    for (const auto& st : compute_stream_stats()) {
        Json s;
        s["stream_id"] = static_cast<int64_t>(st.stream_id);
        s["name"] = st.stream_name;
        s["tx_frames"] = st.tx_frames;
        s["rx_frames"] = st.rx_frames;
        s["deadline_misses"] = st.deadline_misses;
        s["min_latency_us"] = static_cast<double>(st.min_latency_ps) / static_cast<double>(TSN_PS_PER_US);
        s["avg_latency_us"] = st.avg_latency_ps / static_cast<double>(TSN_PS_PER_US);
        s["p99_latency_us"] = static_cast<double>(st.p99_latency_ps) / static_cast<double>(TSN_PS_PER_US);
        s["max_latency_us"] = static_cast<double>(st.max_latency_ps) / static_cast<double>(TSN_PS_PER_US);
        s["jitter_us"] = static_cast<double>(st.jitter_ps) / static_cast<double>(TSN_PS_PER_US);
        streams_arr.push_back(std::move(s));
    }
    root["streams"] = std::move(streams_arr);

    if (!gptp_stats_.empty()) {
        Json gptp_arr = Json::Array{};
        for (const auto& g : compute_gptp_stats()) {
            Json jg;
            jg["node"] = g.node_name;
            jg["sync_count"] = g.sync_count;
            jg["max_abs_err_ns"] = static_cast<double>(g.max_abs_err_ps) / static_cast<double>(TSN_PS_PER_NS);
            jg["last_err_ns"] = static_cast<double>(g.last_err_ps) / static_cast<double>(TSN_PS_PER_NS);
            gptp_arr.push_back(std::move(jg));
        }
        root["gptp"] = std::move(gptp_arr);
    }
    return root;
}

void Telemetry::print_summary(std::ostream& os, tsn_time_ps_t sim_duration_ps,
                              uint64_t executed_events) const {
    os << "=== rtt-open-tsn Simulation Summary ===\n";
    os << "Duration: " << std::fixed << std::setprecision(3)
       << (static_cast<double>(sim_duration_ps) / static_cast<double>(TSN_PS_PER_MS))
       << " ms | Events: " << executed_events
       << " | Link drops: " << link_drops_ << "\n\n";

    os << std::left << std::setw(18) << "Stream"
       << std::right << std::setw(8) << "TX"
       << std::setw(8) << "RX"
       << std::setw(12) << "Min(us)"
       << std::setw(12) << "Avg(us)"
       << std::setw(12) << "P99(us)"
       << std::setw(12) << "Max(us)"
       << std::setw(12) << "Jitter(us)"
       << std::setw(8) << "Miss" << "\n";
    os << std::string(102, '-') << "\n";

    for (const auto& st : compute_stream_stats()) {
        auto to_us = [](double ps) { return ps / static_cast<double>(TSN_PS_PER_US); };
        os << std::left << std::setw(18) << st.stream_name
           << std::right << std::setw(8) << st.tx_frames
           << std::setw(8) << st.rx_frames
           << std::setw(12) << std::setprecision(3) << to_us(static_cast<double>(st.min_latency_ps))
           << std::setw(12) << std::setprecision(3) << to_us(st.avg_latency_ps)
           << std::setw(12) << std::setprecision(3) << to_us(static_cast<double>(st.p99_latency_ps))
           << std::setw(12) << std::setprecision(3) << to_us(static_cast<double>(st.max_latency_ps))
           << std::setw(12) << std::setprecision(3) << to_us(static_cast<double>(st.jitter_ps))
           << std::setw(8) << st.deadline_misses << "\n";
    }

    if (!gptp_stats_.empty()) {
        os << "\n802.1AS gPTP Synchronization:\n";
        os << std::left << std::setw(18) << "Node"
           << std::right << std::setw(10) << "Syncs"
           << std::setw(18) << "MaxErr(ns)"
           << std::setw(18) << "LastErr(ns)" << "\n";
        os << std::string(64, '-') << "\n";
        for (const auto& g : compute_gptp_stats()) {
            os << std::left << std::setw(18) << g.node_name
               << std::right << std::setw(10) << g.sync_count
               << std::setw(18) << std::setprecision(3)
               << (static_cast<double>(g.max_abs_err_ps) / static_cast<double>(TSN_PS_PER_NS))
               << std::setw(18) << std::setprecision(3)
               << (static_cast<double>(g.last_err_ps) / static_cast<double>(TSN_PS_PER_NS))
               << "\n";
        }
    }
}

void Telemetry::write_csv(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs) {
        throw std::runtime_error("Failed to open CSV output file: " + path);
    }
    ofs << "stream_id,frame_uid,seq,pcp,bytes,gen_time_ps,rx_time_ps,latency_ps,deadline_ps,hops,missed\n";
    for (const auto& r : records_) {
        ofs << r.stream_id << ',' << r.frame_uid << ',' << r.seq_in_stream << ','
            << static_cast<int>(r.pcp) << ',' << r.length_bytes << ','
            << r.gen_time_ps << ',' << r.rx_time_ps << ',' << r.latency_ps << ','
            << r.deadline_ps << ',' << static_cast<int>(r.hop_count) << ','
            << (r.missed_deadline ? 1 : 0) << '\n';
    }
}

void Telemetry::write_perfetto_trace(const std::string& path) const {
    Json root;
    Json events = Json::Array{};

    /* Emit process_name metadata events so Perfetto shows node names */
    for (const auto& [nid, name] : node_names_) {
        Json meta;
        meta["name"] = "process_name";
        meta["ph"] = "M";
        meta["pid"] = static_cast<int64_t>(nid);
        Json args;
        args["name"] = name;
        meta["args"] = std::move(args);
        events.push_back(std::move(meta));
    }

    /* Chrome Trace / Perfetto timestamps are in microseconds (double) */
    for (const auto& sp : spans_) {
        Json ev;
        ev["name"] = sp.name;
        ev["cat"] = sp.category;
        ev["ph"] = "X";
        ev["pid"] = static_cast<int64_t>(sp.pid);
        ev["tid"] = static_cast<int64_t>(sp.tid);
        ev["ts"] = static_cast<double>(sp.start_ps) / static_cast<double>(TSN_PS_PER_US);
        ev["dur"] = static_cast<double>(sp.dur_ps) / static_cast<double>(TSN_PS_PER_US);
        Json args;
        args["stream_id"] = static_cast<int64_t>(sp.stream_id);
        args["frame_uid"] = static_cast<int64_t>(sp.frame_uid);
        args["pcp"] = static_cast<int64_t>(sp.pcp);
        ev["args"] = std::move(args);
        events.push_back(std::move(ev));
    }

    /* Emit Qbv gate state transitions as instant markers on port tracks */
    for (const auto& g : qbv_events_) {
        Json ev;
        char buf[16];
        std::snprintf(buf, sizeof(buf), "GCL:0x%02X", g.gate_mask);
        ev["name"] = std::string(buf);
        ev["cat"] = "qbv";
        ev["ph"] = "i";
        ev["s"] = "t";
        ev["pid"] = static_cast<int64_t>(g.node_id);
        ev["tid"] = static_cast<int64_t>(g.port_id);
        ev["ts"] = static_cast<double>(g.timestamp_ps) / static_cast<double>(TSN_PS_PER_US);
        events.push_back(std::move(ev));
    }

    root["traceEvents"] = std::move(events);
    root["displayTimeUnit"] = "ns";

    std::ofstream ofs(path);
    if (!ofs) {
        throw std::runtime_error("Failed to open Perfetto trace file: " + path);
    }
    ofs << root.dump(2) << '\n';
}

} // namespace sim
