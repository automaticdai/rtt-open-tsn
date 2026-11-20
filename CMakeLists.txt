cmake_minimum_required(VERSION 3.20)
project(rtt-open-tsn VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(TSN_BUILD_SIM "Build the C++20 discrete-event simulator" ON)
option(TSN_BUILD_TESTS "Build unit and integration tests" ON)

# ---------------------------------------------------------------------------
# Portable C11 TSN Core Library (RTOS / Bare-Metal Embedded Ready)
# ---------------------------------------------------------------------------
add_library(tsn_core STATIC
    core/src/frame.c
    core/src/queue.c
    core/src/qbv.c
    core/src/qav.c
    core/src/qbu.c
    core/src/frer.c
    core/src/gptp.c
    core/src/bridge.c
)

target_include_directories(tsn_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

if(UNIX)
    target_link_libraries(tsn_core PRIVATE m)
endif()

target_compile_options(tsn_core PRIVATE
    -Wall -Wextra -Wpedantic
)

if(TSN_BUILD_SIM)
    target_compile_definitions(tsn_core PUBLIC TSN_SIM=1)
endif()

# ---------------------------------------------------------------------------
# C++20 Discrete-Event Simulator
# ---------------------------------------------------------------------------
if(TSN_BUILD_SIM)
    add_library(tsn_sim_lib STATIC
        sim/src/kernel.cpp
        sim/src/clock.cpp
        sim/src/link.cpp
        sim/src/node.cpp
        sim/src/cnc.cpp
        sim/src/telemetry.cpp
        sim/src/config.cpp
    )

    target_include_directories(tsn_sim_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/sim/include
    )

    target_link_libraries(tsn_sim_lib PUBLIC tsn_core)

    target_compile_options(tsn_sim_lib PRIVATE
        -Wall -Wextra -Wpedantic
    )

    add_executable(tsn-sim sim/src/main.cpp)
    target_link_libraries(tsn-sim PRIVATE tsn_sim_lib)
    target_compile_options(tsn-sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Tests (CTest)
# ---------------------------------------------------------------------------
if(TSN_BUILD_TESTS AND TSN_BUILD_SIM)
    enable_testing()

    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE tsn_core)
    target_compile_options(test_core PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_core COMMAND test_core)

    add_executable(test_kernel tests/test_kernel.cpp)
    target_link_libraries(test_kernel PRIVATE tsn_sim_lib)
    target_compile_options(test_kernel PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_kernel COMMAND test_kernel)

    add_executable(test_e2e_phase1 tests/test_e2e_phase1.cpp)
    target_link_libraries(test_e2e_phase1 PRIVATE tsn_sim_lib)
    target_compile_options(test_e2e_phase1 PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_e2e_phase1 COMMAND test_e2e_phase1
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_executable(test_phase2 tests/test_phase2.cpp)
    target_link_libraries(test_phase2 PRIVATE tsn_sim_lib)
    target_compile_options(test_phase2 PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_phase2 COMMAND test_phase2
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_executable(test_phase3 tests/test_phase3.cpp)
    target_link_libraries(test_phase3 PRIVATE tsn_sim_lib)
    target_compile_options(test_phase3 PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_phase3 COMMAND test_phase3
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_executable(test_phase4 tests/test_phase4.cpp)
    target_link_libraries(test_phase4 PRIVATE tsn_sim_lib)
    target_compile_options(test_phase4 PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_phase4 COMMAND test_phase4
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_executable(test_phase5 tests/test_phase5.cpp)
    target_link_libraries(test_phase5 PRIVATE tsn_sim_lib)
    target_compile_options(test_phase5 PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_phase5 COMMAND test_phase5
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
