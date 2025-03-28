cmake_minimum_required(VERSION 3.8)
project(gimbal_status)

set(CMAKE_BUILD_TYPE Debug)

# Set GHADRON
set(GHADRON 1)
message("-- GHADRON: ${GHADRON}")
add_definitions(-DGHADRON=${GHADRON})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

# Set PayloadSDK path
set(PAYLOAD_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../PayloadSdk)
set(PAYLOADSDKLIB_DIR ${PAYLOAD_SDK_ROOT}/libs)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PAYLOADSDKLIB_DIR}
    ${PAYLOADSDKLIB_DIR}/aarch64
    ${PAYLOADSDKLIB_DIR}/aarch64/payload-define
    ${PAYLOADSDKLIB_DIR}/third-party/mavlink/include
    ${PAYLOADSDKLIB_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
    ${PAYLOADSDKLIB_DIR}/gSDK/src
)

# Build PayloadSDK first
execute_process(
  COMMAND bash -c "cd ${PAYLOAD_SDK_ROOT} && mkdir -p build && cd build && cmake .. -DGHADRON=1 && make"
  RESULT_VARIABLE PAYLOAD_SDK_BUILD_RESULT
  OUTPUT_VARIABLE PAYLOAD_SDK_BUILD_OUTPUT
  ERROR_VARIABLE PAYLOAD_SDK_BUILD_ERROR
)

if(NOT PAYLOAD_SDK_BUILD_RESULT EQUAL 0)
  message(FATAL_ERROR "Failed to build PayloadSDK: ${PAYLOAD_SDK_BUILD_ERROR}")
endif()

# Add executable
add_executable(gimbal_status_node src/gimbal_status_node.cpp)

# Add ROS2 dependencies
ament_target_dependencies(gimbal_status_node
    rclcpp
    geometry_msgs
)

# Link libraries
target_link_libraries(gimbal_status_node
    ${PAYLOADSDKLIB_DIR}/aarch64/libPayloadSDK.a
    ${CMAKE_THREAD_LIBS_INIT}
    stdc++fs
)

# Install targets
install(TARGETS
    gimbal_status_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
