PROJECT(webviz)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Convert AMENT_PREFIX_PATH to CMAKE_PREFIX_PATH (standard ROS2 pattern)
if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" ament_prefix_path_list "$ENV{AMENT_PREFIX_PATH}")
  foreach(path ${ament_prefix_path_list})
    list(APPEND CMAKE_PREFIX_PATH "${path}")
  endforeach()
endif()

find_package(amrl_msgs REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets WebSockets REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LUA REQUIRED lua5.2)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(CMAKE_CXX_FLAGS "-std=c++17 -march=native -Werror -Wall -g")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/config_reader/include)

add_executable(websocket
    src/websocket/websocket_main.cc
    src/websocket/websocket.cc
)

ament_target_dependencies(websocket
  rclcpp
  ament_index_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  amrl_msgs
)

target_link_libraries(websocket
    Qt5::Core
    Qt5::Widgets
    Qt5::WebSockets
    amrl_shared_lib
    glog
    gflags
    ${LUA_LIBRARIES}
    ${OpenCV_LIBS}
)

target_include_directories(websocket PRIVATE ${LUA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

install(TARGETS websocket
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES webviz.html
  DESTINATION share/${PROJECT_NAME}
)

install(FILES config/webviz_config.lua
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rclcpp ament_index_cpp std_msgs geometry_msgs sensor_msgs nav_msgs amrl_msgs)

ament_package()
