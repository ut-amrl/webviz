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
- **🔀 ROS1/ROS2 Compatible**: Full cross-compatibility between ROS versions

## ROS Compatibility Status

### ✅ Fully Cross-Compatible (ROS1 & ROS2)

WebViz is **fully compatible with both ROS1 and ROS2**:

- **🎯 Automatic Detection**: Build system automatically detects your ROS version (`$ROS_VERSION`)
- **🔄 Unified Codebase**: Single codebase works seamlessly with both ROS versions
- **📦 Cross-Compatible Messages**: Uses amrl_msgs for ROS1/ROS2 message compatibility
- **🛠️ Abstracted APIs**: ROS-specific APIs are abstracted through cross-compatible macros
- **⚙️ Smart Build System**: CMake automatically configures for the detected ROS version

## Dependencies

- **ROS**: [ROS1 Noetic](http://wiki.ros.org/noetic/Installation) or [ROS2 Humble/Iron](https://docs.ros.org/en/rolling/Installation.html) 
- **AMRL Messages**: [amrl_msgs](https://github.com/ut-amrl/amrl_msgs) (ROS1/ROS2 compatible)
- **System Packages**:
    ```bash
    # Ubuntu/Debian
    sudo apt install build-essential cmake qt5-default libqt5websockets5-dev \
                     qtbase5-dev qtwebengine5-dev libgoogle-glog-dev libgflags-dev \
                     colcon-common-extensions libgtest-dev
    ```

## Usage

1. Clone this repository and initialize submodules:
    ```bash
    git clone https://github.com/ut-amrl/webviz.git
    cd webviz
    git submodule update --init --recursive
    ```

2. **For ROS1**: Add the path to your `~/.bashrc` file for the `ROS_PACKAGE_PATH` environment variable:
    ```bash
    echo "export ROS_PACKAGE_PATH=$(pwd):\$ROS_PACKAGE_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

3. **For ROS2**: Add the path to your `~/.bashrc` file for the `AMENT_PREFIX_PATH` environment variable:
    ```bash
    echo "export AMENT_PREFIX_PATH=$(pwd)/install:\$AMENT_PREFIX_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Run `make`. This will automatically build and install (for ROS2) or just build (for ROS1). No need to run `catkin_*`, `rosbuild`, etc.

## Running WebViz

### Start the WebSocket Server
```bash
# Start the server (listens on port 10272 by default)
./bin/websocket

# Optional: Specify custom frame rate and message timeout
./bin/websocket --fps=20 --max_age=5.0
```

### Open Web Interface
1. Open `webviz.html` in any web browser
2. Enter the robot's IP address or hostname
3. Click "Connect" to start receiving data

### Interact with the Robot
- **Set Initial Pose**: Click and drag to set robot's initial position and orientation
- **Set Navigation Goal**: Right-click to set navigation targets
- **Reset Goals**: Use the interface to clear all navigation goals

### View Real-Time Data
The interface displays:
- **Laser Scans**: Real-time LiDAR/laser scanner data
- **Robot Localization**: Current pose estimates with uncertainty
- **Custom Visualizations**: Geometry, paths, and annotations from your ROS nodes