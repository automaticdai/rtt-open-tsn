#include "sim/clock.hpp"
#include "sim/json.hpp"
#include "sim/kernel.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

static void test_kernel_deterministic_order() {
    sim::SimKernel k(123);
    std::vector<int> order;

    k.schedule_at(2000, [&]() { order.push_back(4); });
    k.schedule_at(1000, [&]() { order.push_back(2); }, /*priority=*/1);
    k.schedule_at(1000, [&]() { order.push_back(1); }, /*priority=*/0);
    auto cancelled_id = k.schedule_at(1500, [&]() { order.push_back(99); });
    k.schedule_at(1000, [&]() { order.push_back(3); }, /*priority=*/1);
    k.cancel(cancelled_id);

    k.run_until(3000);
    assert(order.size() == 4);
    assert(order[0] == 1);
    assert(order[1] == 2);
    assert(order[2] == 3);
    assert(order[3] == 4);
    assert(k.now_ps() == 3000);
}

static void test_local_clock_drift_and_adjust() {
    /* +10 ppm = +10,000 ppb -> over 1 second (1e12 ps), clock gains 10 us (1e7 ps) */
    sim::LocalClock clk(500 * TSN_PS_PER_NS, 10000.0);

    tsn_time_ps_t t_local = clk.sim_to_local(TSN_PS_PER_S);
    assert(t_local == TSN_PS_PER_S + 10 * TSN_PS_PER_US + 500 * TSN_PS_PER_NS);

    /* Round-trip conversion local -> sim -> local */
    tsn_time_ps_t sim_back = clk.local_to_sim(t_local);
    assert(std::llabs(sim_back - TSN_PS_PER_S) <= 1);

    /* Servo cancels the +10,000 ppb drift and corrects offset at t = 1s */
    clk.adjust(TSN_PS_PER_S, -(10 * TSN_PS_PER_US + 500 * TSN_PS_PER_NS), -10000.0);
    assert(clk.sim_to_local(TSN_PS_PER_S) == TSN_PS_PER_S);
    /* After another 1 second with zero net drift, local == sim */
    assert(clk.sim_to_local(2 * TSN_PS_PER_S) == 2 * TSN_PS_PER_S);
}

static void test_json_roundtrip() {
    const std::string src = R"({"a": 42, "b": 3.5, "c": [true, false, null, "hi"]})";
    auto j = sim::Json::parse(src);
    assert(j.at("a").as_int() == 42);
    assert(std::fabs(j.at("b").as_double() - 3.5) < 1e-9);
    assert(j.at("c").as_array().size() == 4);
    assert(j.at("c").as_array()[3].as_string() == "hi");

    auto j2 = sim::Json::parse(j.dump());
    assert(j2.at("a").as_int() == 42);
}

int main() {
    test_kernel_deterministic_order();
    test_local_clock_drift_and_adjust();
    test_json_roundtrip();
    std::cout << "[PASS] test_kernel\n";
    return 0;
}
