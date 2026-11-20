#include "sim/config.hpp"
#include <fstream>
#include <iostream>
#include <string>

static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " <scenario.json> [--summary <out.json>] [--csv <out.csv>] [--trace <perfetto.json>]\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string scenario_path = argv[1];
    std::string summary_path;
    std::string csv_path;
    std::string trace_path;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--summary" && i + 1 < argc) {
            summary_path = argv[++i];
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--trace" && i + 1 < argc) {
            trace_path = argv[++i];
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    try {
        sim::Scenario sc = sim::Scenario::load_from_file(scenario_path);
        sc.run();
        sc.telemetry().print_summary(std::cout, sc.duration_ps(), sc.kernel().executed_events());

        if (!summary_path.empty()) {
            std::ofstream ofs(summary_path);
            ofs << sc.telemetry()
                       .to_summary_json(sc.duration_ps(), sc.kernel().executed_events())
                       .dump(2)
                << '\n';
        }
        if (!csv_path.empty()) {
            sc.telemetry().write_csv(csv_path);
        }
        if (!trace_path.empty()) {
            sc.telemetry().write_perfetto_trace(trace_path);
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
