cmake_minimum_required(VERSION 3.16)
project(webviz)

# --------------------------------------------------------------------
# Basic build configuration
# --------------------------------------------------------------------
message(STATUS "Compiling Webviz")
message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")

# Extra flags for Debug/Release
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(STATUS "Additional Flags for Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  message(STATUS "Additional Flags for Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# --------------------------------------------------------------------
# Determine ROS version from environment variable
# --------------------------------------------------------------------
if(DEFINED ENV{ROS_VERSION})
  if("$ENV{ROS_VERSION}" STREQUAL "1")
    set(ROS1 TRUE)
    add_definitions(-DROS1)
    message(STATUS "Compiling for ROS1")
  elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    set(ROS2 TRUE)
    add_definitions(-DROS2)
    message(STATUS "Compiling for ROS2")
  else()
    message(FATAL_ERROR "Unknown ROS_VERSION: $ENV{ROS_VERSION}. Must be '1' or '2'.")
  endif()
else()
  message(FATAL_ERROR "ROS_VERSION environment variable not set (should be '1' or '2').")
endif()

# --------------------------------------------------------------------
# ROS1 vs ROS2
# --------------------------------------------------------------------
if(ROS1)
  #------------------------------------------
  # ROS1
  #------------------------------------------
  # Example using catkin or rosbuild. If you truly have only rosbuild, you can do:
  if(NOT DEFINED ENV{ROS_ROOT})
    message(FATAL_ERROR "ROS_ROOT is not set; cannot build for ROS1 with rosbuild.")
  endif()
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

  # If you have message generation, do:
  # rosbuild_genmsg()
  # rosbuild_gensrv()

  message(STATUS "Using rosbuild for ROS1")

else()
  #------------------------------------------
  # ROS2
  #------------------------------------------
  # We use ament_cmake for ROS2
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  # Also find other packages your node uses
  find_package(amrl_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(Qt5 COMPONENTS Core Widgets WebSockets REQUIRED)

  message(STATUS "Using ament_cmake for ROS2")
endif()

# --------------------------------------------------------------------
# Qt
# --------------------------------------------------------------------
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# --------------------------------------------------------------------
# Output directories
# --------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# --------------------------------------------------------------------
# Include directories
# --------------------------------------------------------------------
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR})

# If you have a subdirectory with shared code:
add_subdirectory(src/shared)
include_directories(${PROJECT_SOURCE_DIR}/src/shared)

# --------------------------------------------------------------------
# Libraries (depends on ROS1 vs ROS2)
# --------------------------------------------------------------------
if(ROS1)
  # Example: we link roslib, roscpp, etc.
  set(ros_libs roslib roscpp glog gflags amrl_shared_lib)
  message(STATUS "ROS1 libs: ${ros_libs}")

else()
  # For ROS2 with ament, we’ll rely on ament_target_dependencies to link
  # But we can keep some 3rd-party libs in a variable if needed:
  set(ros_libs glog gflags amrl_shared_lib)
  message(STATUS "ROS2 libs: ${ros_libs}")
endif()

# --------------------------------------------------------------------
# Add Executable
# --------------------------------------------------------------------
# We assume your main is src/websocket/websocket_main.cc
# plus other sources in src/websocket/
set(SOURCES
  src/websocket/websocket_main.cc
  src/websocket/websocket.cc
  src/websocket/ros2_adapter.cc
)

if(ROS1)
  #----------------------------------
  # ROS1 build of "websocket"
  #----------------------------------
  rosbuild_add_executable(websocket ${SOURCES})
  target_link_libraries(websocket ${ros_libs} Qt5::Core Qt5::Widgets Qt5::WebSockets)

  # If you have a catkin or rosbuild approach to message generation, handle it above.

else()
  #----------------------------------
  # ROS2 build of "websocket"
  #----------------------------------
  add_executable(websocket ${SOURCES})
  target_link_libraries(websocket
    ${ros_libs}
    Qt5::Core
    Qt5::Widgets
    Qt5::WebSockets
  )
  # Add ament dependencies so the type-support libs (like amrl_msgs) are properly linked
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

# If we’re building with ROS2, finalize with ament_package()
if(ROS2)
  ament_package()
endif()
