cmake_minimum_required(VERSION 3.10)
project(noetix_dual_arm_udp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_library(MODBUS_LIB modbus)
if(NOT MODBUS_LIB)
    message(FATAL_ERROR "libmodbus library not found")
endif()

add_library(noetix-dual-arm-control STATIC
    local_control/DualArmController.cpp
    local_control/Profile.cpp
)
target_include_directories(noetix-dual-arm-control PUBLIC include)
target_link_libraries(noetix-dual-arm-control PUBLIC
    ${MODBUS_LIB}
    Threads::Threads
)

add_library(noetix-dual-arm-udp-protocol STATIC
    local_control/UdpProtocol.cpp
)
target_include_directories(noetix-dual-arm-udp-protocol PUBLIC include)

add_executable(noetix-dual-arm-udp
    tools/dual_arm_udp_receiver.cpp
)
target_link_libraries(noetix-dual-arm-udp PRIVATE
    noetix-dual-arm-control
    noetix-dual-arm-udp-protocol
)
