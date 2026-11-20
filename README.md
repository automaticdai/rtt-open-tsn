# rtt-open-tsn

An open **Real-Time TSN (`rtt`)** implementation and discrete-event simulator for **Time-Sensitive Networking (TSN)**, designed with a strict separation between a **portable C11 TSN core (`libtsn_core`)** (zero dynamic allocation on the packet hot path, RTOS / bare-metal embedded ready) and a **C++20 picosecond discrete-event simulator (`tsn-sim`)**.

## Supported IEEE 802.1 TSN Standards

- **IEEE 802.1AS (gPTP)**: Peer delay measurement (`Pdelay_Req`/`Resp`/`Resp_Fup`) with `neighborRateRatio`, multi-hop `Sync`/`Follow_Up` with residence-time `correctionField`, hardware TX/RX timestamping, and PI clock servo.
- **IEEE 802.1Qbv (TAS)**: Event-driven Time-Aware Shaper with `Admin`/`Oper` Gate Control Lists (GCL), atomic `ConfigChangeTime` swap, `SetGateStates`/`SetAndHoldMAC`/`SetAndReleaseMAC`, and guard band.
- **IEEE 802.1Qav (CBS)**: Event-driven Credit-Based Shaper with exact zero-crossing wakeup scheduling and IEEE 802.1Q-2018 §8.6.8.2 credit freeze during closed Qbv gates.
- **IEEE 802.1Qbu / 802.3br (Frame Preemption)**: eMAC/pMAC classification, dynamic Express-triggered and Qbv Hold-triggered fragmentation (`SMD-S`/`SMD-C`/`4B mCRC`), `addFragSize` enforcement, and ingress mPacket reassembly.
- **IEEE 802.1CB (FRER)**: Null Stream ID, 16-bit R-TAG (`0xF1C1`) sequence generation, multi-port stream splitting, Vector & Match sequence recovery, and R-TAG stripping.
- **IEEE 802.1Qcc (CNC / CUC)**: Topology graph, shortest-path & disjoint-path routing, pluggable `IGclScheduler` (`HeuristicGclScheduler`), auto-configuration of FDB/FRER/Qbv/Qav, and dynamic mid-simulation stream reconfiguration.

See [docs/simulator-design.md](docs/simulator-design.md) for the full architecture specification.

## Build & Test

Zero external dependencies (requires CMake ≥ 3.20 and a C11 / C++20 compiler):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Running Scenarios

```bash
# Phase 1: Basic 2-switch line topology (strict priority)
./build/tsn-sim scenarios/line_basic.json --trace build/line_basic_trace.json

# Phase 2: 802.1AS gPTP + 802.1Qbv TAS + 802.1Qav CBS (drifting clocks)
./build/tsn-sim scenarios/phase2_tsn_shapers.json --trace build/phase2_trace.json

# Phase 3: 802.1Qbu / 802.3br Frame Preemption
./build/tsn-sim scenarios/phase3_preemption.json --trace build/phase3_trace.json

# Phase 4: 802.1CB FRER Diamond Topology with Link-Cut Fault Injection
./build/tsn-sim scenarios/phase4_frer_diamond.json --trace build/phase4_trace.json

# Phase 5: 802.1Qcc CNC/CUC Auto-Config + FRER + Dynamic Runtime Reconfiguration
./build/tsn-sim scenarios/phase5_qcc_auto.json --trace build/phase5_trace.json --csv build/phase5.csv --summary build/phase5_summary.json
```

Open any generated `*_trace.json` file in [ui.perfetto.dev](https://ui.perfetto.dev/) or `chrome://tracing` to inspect hop-by-hop frame transmissions, preemption fragments, and Qbv GCL gate transitions.
