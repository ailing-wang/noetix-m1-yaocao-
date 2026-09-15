cmake_minimum_required(VERSION 3.10)
project(noetix)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_library(MODBUS_LIB modbus)
if(NOT MODBUS_LIB)
    message(FATAL_ERROR "libmodbus library not found")
endif()

find_package(spdlog REQUIRED)

message(STATUS "Found libmodbus: ${MODBUS_LIB}")

add_library(version_lib SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/version_lib.cpp
)

target_include_directories(version_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_compile_definitions(version_lib
    PRIVATE
        VERSION_LIB_EXPORTS
)

file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(noetix-yaocao ${SOURCES})

target_include_directories(noetix-yaocao
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(noetix-yaocao
    PRIVATE
        ${MODBUS_LIB}
        version_lib
        Threads::Threads
        spdlog::spdlog
)

# Standalone, single-servo bench controller. It is intentionally kept out of
# src/ so it cannot be pulled into the production executable by SOURCES glob.
add_executable(noetix-servo-test
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/single_servo_control.cpp
)

target_include_directories(noetix-servo-test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(noetix-servo-test
    PRIVATE
        ${MODBUS_LIB}
        Threads::Threads
)

# Standalone, no-UDP, seven-joint left-arm bench controller.
add_executable(noetix-left-arm-test
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/left_arm_control.cpp
)

target_link_libraries(noetix-left-arm-test
    PRIVATE
        ${MODBUS_LIB}
        Threads::Threads
)

# Formal no-UDP left-arm controller. The core owns both Modbus ports from one
# worker thread; frontends submit timestamped commands and read cached state.
add_library(noetix-left-arm-local-core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Controller.cpp
)

target_include_directories(noetix-left-arm-local-core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(noetix-left-arm-local-core
    PUBLIC
        ${MODBUS_LIB}
        Threads::Threads
)

add_executable(noetix-left-arm-local
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/left_arm_local_cli.cpp
)

target_link_libraries(noetix-left-arm-local
    PRIVATE
        noetix-left-arm-local-core
)

# Standalone, no-UDP, fourteen-joint dual-arm controller. Four RTU ports are
# exclusively owned and serialized by one worker thread. Configured PID values
# are applied with lock=1 at runtime and restored on normal exit.
add_library(noetix-dual-arm-local-core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Calibration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/local_control/DualArmController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Profile.cpp
)

target_include_directories(noetix-dual-arm-local-core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(noetix-dual-arm-local-core
    PUBLIC
        ${MODBUS_LIB}
        Threads::Threads
)

add_executable(noetix-dual-arm-local
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/dual_arm_local_cli.cpp
)

target_link_libraries(noetix-dual-arm-local
    PRIVATE
        noetix-dual-arm-local-core
)

# UDP is only an outer transport adapter. It parses and validates datagrams,
# then submits commands to the same owner-thread controller used by the local
# CLI; it never opens or accesses a Modbus device itself.
add_library(noetix-dual-arm-udp-protocol STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/local_control/UdpProtocol.cpp
)

target_include_directories(noetix-dual-arm-udp-protocol
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(noetix-dual-arm-udp
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/dual_arm_udp_receiver.cpp
)

target_link_libraries(noetix-dual-arm-udp
    PRIVATE
        noetix-dual-arm-local-core
        noetix-dual-arm-udp-protocol
)

add_executable(noetix-dual-arm-limit-recorder
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/dual_arm_limit_recorder.cpp
)

target_link_libraries(noetix-dual-arm-limit-recorder
    PRIVATE
        noetix-dual-arm-local-core
)

# One-shot, no-UDP transition to the legacy encoder-zero definition. It uses
# the formal owner-thread controller and always finishes with STOP ALL.
add_executable(noetix-dual-arm-legacy-zero
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/dual_arm_legacy_zero.cpp
)

target_link_libraries(noetix-dual-arm-legacy-zero
    PRIVATE
        noetix-dual-arm-local-core
)

option(NOETIX_BUILD_LOCAL_CONTROL_TESTS
       "Build the fake-Modbus local controller tests" OFF)

if(NOETIX_BUILD_LOCAL_CONTROL_TESTS)
    enable_testing()
    add_executable(noetix-left-arm-local-test
        ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Controller.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fake_modbus/fake_modbus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/left_arm_local_service_test.cpp
    )
    target_include_directories(noetix-left-arm-local-test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fake_modbus
    )
    target_link_libraries(noetix-left-arm-local-test
        PRIVATE
            Threads::Threads
    )
    add_test(NAME noetix-left-arm-local-test
             COMMAND noetix-left-arm-local-test)

    add_executable(noetix-dual-arm-local-test
        ${CMAKE_CURRENT_SOURCE_DIR}/local_control/DualArmController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Profile.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fake_modbus/fake_modbus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/dual_arm_local_service_test.cpp
    )
    target_include_directories(noetix-dual-arm-local-test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fake_modbus
    )
    target_link_libraries(noetix-dual-arm-local-test
        PRIVATE
            Threads::Threads
    )
    add_test(NAME noetix-dual-arm-local-test
             COMMAND noetix-dual-arm-local-test)

    add_executable(noetix-dual-arm-profile-test
        ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Profile.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/dual_arm_profile_test.cpp
    )
    target_include_directories(noetix-dual-arm-profile-test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    add_test(NAME noetix-dual-arm-profile-test
             COMMAND noetix-dual-arm-profile-test)

    add_executable(noetix-dual-arm-calibration-test
        ${CMAKE_CURRENT_SOURCE_DIR}/local_control/Calibration.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/dual_arm_calibration_test.cpp
    )
    target_include_directories(noetix-dual-arm-calibration-test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    add_test(NAME noetix-dual-arm-calibration-test
             COMMAND noetix-dual-arm-calibration-test)

    add_executable(noetix-dual-arm-udp-protocol-test
        ${CMAKE_CURRENT_SOURCE_DIR}/local_control/UdpProtocol.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/dual_arm_udp_protocol_test.cpp
    )
    target_include_directories(noetix-dual-arm-udp-protocol-test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    add_test(NAME noetix-dual-arm-udp-protocol-test
             COMMAND noetix-dual-arm-udp-protocol-test)
endif()
