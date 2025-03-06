cmake_minimum_required(VERSION 3.16)
project(webviz)

# --------------------------------------------------------------------
# Basic build settings
# --------------------------------------------------------------------
message(STATUS "Compiling Webviz")
message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# We can override the standard if you like:
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")

# Additional flags depending on build type
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(STATUS "Additional Flags for Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  message(STATUS "Additional Flags for Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# --------------------------------------------------------------------
# Determine ROS version from environment variable
# --------------------------------------------------------------------
if(DEFINED ENV{ROS_VERSION})
  if("$ENV{ROS_VERSION}" STREQUAL "1")
    add_definitions(-DUSE_ROS1)
    set(USE_ROS1 TRUE)
    message(STATUS "Compiling for ROS1 (USE_ROS1=ON)")
  elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    add_definitions(-DUSE_ROS2)
    set(USE_ROS1 FALSE)
    message(STATUS "Compiling for ROS2 (USE_ROS2=ON)")
  else()
    message(FATAL_ERROR "Unknown ROS_VERSION: $ENV{ROS_VERSION}. Must be '1' or '2'.")
  endif()
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not defined; must be 1 or 2.")
endif()

# --------------------------------------------------------------------
# ROS1 vs ROS2 setup
# --------------------------------------------------------------------
if(USE_ROS1)
  #------------------------------------------
  # ROS1
  #------------------------------------------
  # The below example uses rosbuild. If you are using catkin, you might
  # do 'find_package(catkin REQUIRED COMPONENTS roscpp ... )' etc. 
  # We'll show rosbuild as your original file did.
  if(NOT DEFINED ENV{ROS_ROOT})
    message(FATAL_ERROR "ROS_ROOT is not set; cannot build for ROS1 with rosbuild.")
  endif()
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

  # Example: we set static & shared libs flags as in your snippet
  set(ROS_BUILD_STATIC_LIBS true)
  set(ROS_BUILD_SHARED_LIBS false)

  message(STATUS "Using rosbuild for ROS1")

else()
  #------------------------------------------
  # ROS2
  #------------------------------------------
  # For ROS2, we use ament_cmake. We can just do:
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(amrl_msgs REQUIRED)

  message(STATUS "Using ament_cmake for ROS2")
endif()

# --------------------------------------------------------------------
# Qt
# --------------------------------------------------------------------
find_package(Qt5 COMPONENTS Core Widgets WebSockets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# --------------------------------------------------------------------
# Output Directories
# --------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# --------------------------------------------------------------------
# Include Directories (example)
# --------------------------------------------------------------------
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR})

# If you have a subdirectory with shared code
add_subdirectory(src/shared)
include_directories(${PROJECT_SOURCE_DIR}/src/shared)

# --------------------------------------------------------------------
# Libraries
# --------------------------------------------------------------------
# If building for ROS1, you might link roslib, roscpp, glog, etc.
# If building for ROS2, you rely on ament, rclcpp, etc.
if(USE_ROS1)
  # Example from your snippet
  set(libs roslib roscpp glog gflags amrl_shared_lib)
else()
  # If needed for ROS2, you might have something else
  set(libs glog gflags amrl_shared_lib rclcpp::rclcpp)
endif()

# --------------------------------------------------------------------
# Add the 'websocket' executable target
# --------------------------------------------------------------------
add_executable(websocket
  src/websocket/websocket_main.cc
  src/websocket/websocket.cc
  src/websocket/ros_adapter.cc   # Or ros1_adapter.cc, ros2_adapter.cc, if you keep them separate
)
target_link_libraries(websocket
  Qt5::Core
  Qt5::Widgets
  Qt5::WebSockets
  ${libs}
)

# If using ROS2, we can do ament dependencies
if(NOT USE_ROS1)
  ament_target_dependencies(websocket
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    amrl_msgs
  )
endif()

# --------------------------------------------------------------------
# Installation
# --------------------------------------------------------------------
install(TARGETS websocket
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# If building for ROS2, finalize with ament_package()
if(NOT USE_ROS1)
  ament_package()
endif()
