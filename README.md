# WebViz

A low-bandwidth websocket-based direct robot visualizer for real-time robot data streaming to web browsers.

[![Build Status](https://github.com/ut-amrl/webviz/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/webviz/actions)

## Overview

WebViz is a lightweight, efficient visualization system designed for robotics applications that enables real-time streaming of robot sensor data, localization information, and custom visualizations to web browsers. Unlike traditional desktop-based visualization tools, WebViz operates through WebSockets, making it ideal for remote monitoring, debugging, and demonstration of robotic systems.

### Key Features

- **🌐 Web-Based Interface**: Access robot visualizations from any device with a web browser
- **⚡ Low-Bandwidth Streaming**: Optimized data transmission for efficient remote operation
- **🔄 Real-Time Updates**: Live streaming of sensor data, localization, and custom visualizations
- **📊 Multi-Data Support**: Handles laser scans, localization data, geometric primitives, and text annotations
- **🎯 Interactive Control**: Set initial poses and navigation goals directly from the web interface
- **🧩 Modular Design**: Clean separation between WebSocket server and shared utilities

### Core Components

- **WebSocket Server** (`src/websocket/`): Qt5-based server that handles client connections and data streaming
- **Shared Library** (`src/shared/`): Cross-ROS-compatible utilities including:
  - **Geometry utilities**: 2D/3D transformations, line intersections, collision detection
  - **Math utilities**: Angle operations, statistics, pose handling
  - **ROS abstractions**: Cross-compatible message handling and logging
- **Web Frontend** (`webviz.html`): Browser-based visualization interface
- **Build System**: Cross-compatible build system supporting both ROS1 and ROS2

### Typical Use Cases

- **Remote Robot Monitoring**: Monitor robots in the field from anywhere with internet access
- **Multi-Robot Coordination**: Visualize multiple robots simultaneously
- **Development & Debugging**: Real-time debugging of navigation, perception, and planning algorithms
- **Demonstrations**: Live demos without requiring RViz or desktop access
- **Educational Purposes**: Teaching robotics concepts with accessible web-based visualizations

## ROS Compatibility Status

### ✅ Shared Library (src/shared/) - Fully Compatible
The shared library (`amrl_shared_lib`) is **fully cross-compatible between ROS1 and ROS2**:
- Uses conditional compilation (`#ifdef ROS2`) for API differences
- Provides unified type aliases for message types
- Abstracts logging and timing through cross-compatible macros
- Core functionality is ROS-independent

### ⚠️ Main WebViz Project - ROS1 Only (Migration In Progress)
The main WebSocket server currently uses ROS1 APIs and requires updates for ROS2 compatibility:
- Build system uses rosbuild (ROS1) instead of ament (ROS2)
- Direct usage of ROS1 APIs (`ros::NodeHandle`, `ros::Publisher`, etc.)
- Uses `manifest.xml` instead of `package.xml`

**Migration Status**: See [ROS2 Migration Plan](#ros2-migration-plan) below.


## Dependencies

### Core Dependencies
1. **ROS**: [ROS1 Noetic](http://wiki.ros.org/noetic/Installation) or [ROS2 Humble/Iron](https://docs.ros.org/en/rolling/Installation.html) 
2. **AMRL Messages**: [amrl_msgs](https://github.com/ut-amrl/amrl_msgs) (ROS1/ROS2 compatible)
3. **Qt5 WebSockets**: 
    ```bash
    # Ubuntu/Debian
    sudo apt install qt5-default libqt5websockets5-dev
    
    # For ROS2 environments, you may also need:
    sudo apt install qtbase5-dev qtwebengine5-dev
    ```

### Additional Dependencies
- **Build Tools**: 
  - ROS1: `build-essential cmake`
  - ROS2: `build-essential cmake colcon-common-extensions`
- **Logging**: `libgoogle-glog-dev`
- **Command Line**: `libgflags-dev`
- **Testing** (optional): `libgtest-dev`

### Automatic Dependency Detection
The build system automatically detects your ROS version using the `ROS_VERSION` environment variable and configures dependencies accordingly.

## Build

### Quick Start (Current - ROS1 Only)
1. **Setup ROS Package Path**:
    ```bash
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```

2. **Initialize Submodules**:
    ```bash
    git submodule update --init --recursive
    ```

3. **Build**:
    ```bash
    make
    ```

### When ROS2 Migration Completes
The build process will automatically detect your ROS version:

```bash
# Clone and setup (same for both ROS versions)
git submodule update --init --recursive

# Build (will auto-detect ROS1 vs ROS2)
make

# For ROS2, additional install step will be handled automatically
# No manual AMENT_PREFIX_PATH setup required
```

## Usage

### 1. Start the WebSocket Server
```bash
# Start the server (listens on port 10272 by default)
./bin/websocket

# Optional: Specify custom frame rate and message timeout
./bin/websocket --fps=20 --max_age=5.0
```

### 2. Open Web Interface
1. Open `webviz.html` in any modern web browser
2. Enter the robot's IP address or hostname
3. Click "Connect" to start receiving data

### 3. Interact with the Robot
- **Set Initial Pose**: Click and drag to set robot's initial position and orientation
- **Set Navigation Goal**: Right-click to set navigation targets
- **Reset Goals**: Use the interface to clear all navigation goals

### 4. View Real-Time Data
The interface displays:
- **Laser Scans**: Real-time LiDAR/laser scanner data
- **Robot Localization**: Current pose estimates with uncertainty
- **Custom Visualizations**: Geometry, paths, and annotations from your ROS nodes
- **Multi-Frame Support**: Simultaneous `map` and `base_link` frame visualizations

## ROS2 Migration Plan

To complete ROS2 compatibility, the following updates are needed:

### 📋 Migration Checklist

- [ ] **Update CMakeLists.txt**: Add ROS version detection and dual build system support
- [ ] **Create package.xml**: ROS2 package descriptor alongside existing manifest.xml
- [ ] **Abstract ROS APIs**: Replace direct ROS1 calls with cross-compatible macros
- [ ] **Update Makefile**: Add ROS2 install handling similar to amrl_msgs
- [ ] **Update CI/CD**: Add ROS2 build testing workflows
- [ ] **Documentation**: Update README with final ROS2 instructions

### 🔧 Technical Implementation Plan

1. **Build System Updates** (Following amrl_msgs pattern):
   ```cmake
   # Auto-detect ROS version from environment
   if("$ENV{ROS_VERSION}" STREQUAL "2")
       # ROS2 ament_cmake setup
       find_package(ament_cmake REQUIRED)
       find_package(rclcpp REQUIRED)
       # ... ROS2 dependencies
       add_compile_definitions(ROS2)
   else()
       # ROS1 rosbuild setup  
       INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
       ROSBUILD_INIT()
       # ... ROS1 dependencies
   endif()
   ```

2. **Source Code Abstraction**:
   - Replace `ros::NodeHandle` with abstracted node interface
   - Use ROS-agnostic publisher/subscriber wrappers
   - Leverage existing `ros_helpers.h` and `ros_macros.h` from shared library

3. **Message Compatibility**:
   - Already handled through amrl_msgs ROS1/ROS2 compatibility
   - Shared library provides message type abstractions

### 🎯 Expected Timeline
- **Phase 1**: Build system updates (1-2 days)
- **Phase 2**: Source code abstraction (2-3 days)  
- **Phase 3**: Testing and documentation (1-2 days)

The shared library's existing ROS2 compatibility provides a solid foundation, making this migration straightforward following established patterns from amrl_msgs.
