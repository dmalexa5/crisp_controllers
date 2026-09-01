cmake_minimum_required(VERSION 3.16)
project(crisp_controllers)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Using ccache")
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable precompiled headers
option(USE_PRECOMPILED_HEADERS "Use precompiled headers" ON)

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

option(ENABLE_NATIVE_OPTIMIZATION
  "Enable CPU-specific optimizations (-march=native)"
  OFF
)

# Default to release build
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Compiler flags by build type
add_library(project_compile_options INTERFACE)
target_compile_options(project_compile_options INTERFACE
  # Warnings (build only)
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>

  # Debug
  $<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:GNU,Clang>>:-O0 -g>

  # Release
  $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU,Clang>>:-O3 -DNDEBUG>
)

# Optional CPU-specific optimizations (non-portable)
if(ENABLE_NATIVE_OPTIMIZATION)
  target_compile_options(project_compile_options INTERFACE
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU,Clang>>:-march=native>
)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Eigen3 REQUIRED)

# Keep Pinocchio and its ABI-sensitive dependencies in the same installation
# as ROS. robotpkg also installs newer copies into /opt/openrobots for qpOASES;
# mixing those libraries with ROS Pinocchio headers corrupts Model destruction.
get_filename_component(ROS_INSTALL_PREFIX "${rclcpp_DIR}/../../.." ABSOLUTE)
set(ROS_ARCH_CMAKE_DIR "${ROS_INSTALL_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake")
if(EXISTS "${ROS_ARCH_CMAKE_DIR}/pinocchio/pinocchioConfig.cmake")
  list(REMOVE_ITEM CMAKE_PREFIX_PATH "${ROS_INSTALL_PREFIX}")
  list(PREPEND CMAKE_PREFIX_PATH "${ROS_INSTALL_PREFIX}")
  list(PREPEND CMAKE_LIBRARY_PATH "${ROS_INSTALL_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
  set(pinocchio_DIR "${ROS_ARCH_CMAKE_DIR}/pinocchio" CACHE PATH "ROS Pinocchio" FORCE)
  set(eigenpy_DIR "${ROS_ARCH_CMAKE_DIR}/eigenpy" CACHE PATH "ROS EigenPy" FORCE)
  set(coal_DIR "${ROS_ARCH_CMAKE_DIR}/coal" CACHE PATH "ROS Coal" FORCE)
endif()
set(CRISP_SAVED_CMAKE_IGNORE_PREFIX_PATH "${CMAKE_IGNORE_PREFIX_PATH}")
list(APPEND CMAKE_IGNORE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED CONFIG)
set(CMAKE_IGNORE_PREFIX_PATH "${CRISP_SAVED_CMAKE_IGNORE_PREFIX_PATH}")
get_target_property(CRISP_PINOCCHIO_LIBRARY pinocchio::pinocchio_default IMPORTED_LOCATION_NONE)
get_target_property(CRISP_COAL_LIBRARY coal::coal IMPORTED_LOCATION_NONE)
message(STATUS "Using Pinocchio library: ${CRISP_PINOCCHIO_LIBRARY}")
message(STATUS "Using Coal library: ${CRISP_COAL_LIBRARY}")
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(qpOASES REQUIRED HINTS /opt/openrobots/lib/cmake/qpOASES)

generate_parameter_library(
  cartesian_controller_parameters
  src/cartesian_controller.yaml
)

generate_parameter_library(
  state_broadcaster_parameters
  src/state_broadcaster.yaml
)

generate_parameter_library(
  torque_feedback_controller_parameters
  src/torque_feedback_controller.yaml
)

generate_parameter_library(
  cartesian_admittance_controller_parameters
  src/cartesian_admittance_controller.yaml
)

generate_parameter_library(
  whole_body_controller_parameters
  src/whole_body_controller.yaml
)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/cartesian_controller.cpp
        src/cartesian_admittance_controller.cpp
        src/state_broadcaster.cpp
        src/whole_body_controller.cpp
        src/torque_feedback_controller.cpp
)

add_executable(
  optimization_tracking_test_node
  tests/optimization_tracking_test_node.cpp
)
target_link_libraries(optimization_tracking_test_node project_compile_options)
ament_target_dependencies(
  optimization_tracking_test_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    project_compile_options
    ${qpOASES_LIBRARY}
)

# qpOASES lives in /opt/openrobots on robotpkg systems. Mark it as a system
# include so that its bundled dependencies cannot shadow ROS-selected headers.
target_compile_options(${PROJECT_NAME} PRIVATE "SHELL:-idirafter ${qpOASES_INCLUDE_DIR}")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(USE_PRECOMPILED_HEADERS)
    target_precompile_headers(${PROJECT_NAME} 
        PRIVATE
        include/crisp_controllers/pch.hpp
    )
endif()

# Link parameter libraries to the main library
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    cartesian_controller_parameters
    cartesian_admittance_controller_parameters
    state_broadcaster_parameters
    whole_body_controller_parameters
    torque_feedback_controller_parameters
)


target_link_libraries(${PROJECT_NAME}
  PUBLIC
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    pinocchio::pinocchio
    realtime_tools::realtime_tools
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${qpOASES_LIBRARY_DIR}"
  INSTALL_RPATH "${qpOASES_LIBRARY_DIR}"
)

# ros2_control did not export namespaced targets until after humble
if(TARGET controller_interface::controller_interface)
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      controller_interface::controller_interface
      hardware_interface::hardware_interface
  )
else()
  ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
      controller_interface
      hardware_interface
  )
endif()

pluginlib_export_plugin_description_file(
        controller_interface crisp_controllers.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    TARGETS optimization_tracking_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
    controller_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    Eigen3
    pinocchio
    realtime_tools
    std_msgs
    geometry_msgs
    sensor_msgs
    generate_parameter_library
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_torque_rate_saturation
    tests/test_torque_rate_saturation.cpp
  )
  target_include_directories(test_torque_rate_saturation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_torque_rate_saturation
    Eigen3::Eigen
  )
  
  ament_add_gtest(test_pseudo_inverse
    tests/test_pseudo_inverse.cpp
  )
  target_include_directories(test_pseudo_inverse PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_pseudo_inverse
    Eigen3::Eigen
  )

  ament_add_gtest(test_filters
    tests/test_filters.cpp
  )
  target_include_directories(test_filters PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_pseudo_inverse
    Eigen3::Eigen
  )
endif()

ament_package()
