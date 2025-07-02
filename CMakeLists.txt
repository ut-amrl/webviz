PROJECT(webviz)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Build Mode: ${CMAKE_BUILD_MODE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# Auto-detect ROS version from environment (amrl_msgs style)
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

if(${ROS_VERSION} EQUAL "1")
  MESSAGE(STATUS "Compiling with ROS1 ROSBUILD")
  
  # ROS1 rosbuild setup
  SET(CMAKE_AUTOMOC ON)
  SET(CMAKE_AUTORCC ON)
  SET(CMAKE_AUTOUIC ON)

  if(CMAKE_VERSION VERSION_LESS "3.7.0")
      set(CMAKE_INCLUDE_CURRENT_DIR ON)
  endif()

  SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g")

  IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
    MESSAGE(STATUS "Additional Flags for Release mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
  ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    MESSAGE(STATUS "Additional Flags for Debug mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
    SET(BUILD_SPECIFIC_LIBRARIES "")
  ENDIF()

  INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  ROSBUILD_INIT()
  SET(ROS_BUILD_STATIC_LIBS true)
  SET(ROS_BUILD_SHARED_LIBS false)

  find_package(Qt5 COMPONENTS Core Widgets WebSockets REQUIRED)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
  MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
  MESSAGE(STATUS "Build-Specific Libraries: ${BUILD_SPECIFIC_LIBRARIES}")

  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

  INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

  ADD_SUBDIRECTORY(src/shared)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared)

  SET(libs roslib roscpp glog gflags amrl_shared_lib
      ${BUILD_SPECIFIC_LIBRARIES})

  ROSBUILD_ADD_EXECUTABLE(websocket
      src/websocket/websocket_main.cc
      src/websocket/websocket.cc
  )
  TARGET_LINK_LIBRARIES(websocket
      Qt5::Core
      Qt5::Widgets
      Qt5::WebSockets
      ${libs} ${OPENGL_LIBRARY})

elseif(${ROS_VERSION} EQUAL "2")
  MESSAGE(STATUS "Compiling with ROS2 COLCON")
  add_compile_definitions(ROS2)
  
  # Qt MOC setup for ROS2
  SET(CMAKE_AUTOMOC ON)
  SET(CMAKE_AUTORCC ON)
  SET(CMAKE_AUTOUIC ON)

  if(CMAKE_VERSION VERSION_LESS "3.7.0")
      set(CMAKE_INCLUDE_CURRENT_DIR ON)
  endif()
  
  # ROS2 ament_cmake setup
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
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
  
  # Qt5 for ROS2
  find_package(Qt5 COMPONENTS Core Widgets WebSockets REQUIRED)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  
  SET(CMAKE_CXX_FLAGS "-std=c++14 -march=native -Werror -Wall -g")
  
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

  # ROS2 executable
  add_executable(websocket
      src/websocket/websocket_main.cc
      src/websocket/websocket.cc
  )
  
  # ROS2 dependencies
  ament_target_dependencies(websocket
    rclcpp
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
  )

  # --- Add install steps ---
  install(TARGETS websocket
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(FILES webviz.html
    DESTINATION share/${PROJECT_NAME}
  )

  # Export runtime dependencies
  ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs amrl_msgs)

  ament_package()

else()
  message(FATAL_ERROR "Unknown ROS_VERSION ${ROS_VERSION}")
endif()
