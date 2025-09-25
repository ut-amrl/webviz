# WebViz

A low-bandwidth websocket-based robot visualizer for real-time data streaming to web browsers.

[![Build Status](https://github.com/ut-amrl/webviz/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/webviz/actions)

## Overview

WebViz provides real-time streaming of robot sensor data, localization, and custom visualizations to web browsers via WebSockets. Supports both ROS1 and ROS2 with the same codebase.

### Key Features

- **Cross-Compatible**: Works with both ROS1 and ROS2
- **Web-Based**: Access from any browser, no desktop apps needed
- **Low-Bandwidth**: Optimized for remote operation
- **Real-Time**: Live streaming of sensor data and visualizations
- **Interactive**: Set poses and navigation goals from the web interface
- **Configurable**: Lua-based configuration system

## Dependencies

- **ROS**: [ROS1 Noetic](http://wiki.ros.org/noetic/Installation) or [ROS2 Humble/Iron](https://docs.ros.org/en/rolling/Installation.html) 
- **AMRL Messages**: [amrl_msgs](https://github.com/ut-amrl/amrl_msgs) (ROS1/ROS2 compatible)
- **System Packages**:
    ```bash
    # Ubuntu/Debian
    sudo apt install build-essential cmake qt5-default libqt5websockets5-dev \
                     python3-colcon-common-extensions libgtest-dev liblua5.1-0-dev \
                     libgoogle-glog-dev libgflags-dev
    ```

## Setup

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

## Usage

1. **Start the server**:
```bash
# Start the server with default configuration
./bin/websocket

# Override configuration file location
./bin/websocket --config_file=config/my_custom_config.lua

# Override other parameters with gflags
./bin/websocket --fps=20 --max_age=5.0 --v=1
```
2. **Open browser**: Load `webviz.html` 
3. **Connect**: Enter robot IP and click Connect

See `config/webviz_config.lua` for complete configuration options.

### Integration with Launch Files

WebViz can be easily integrated into larger ROS stacks using launch files.

#### ROS1 Launch File Example

```xml
<launch>
  <!-- Your existing robot nodes -->
  <include file="$(find your_robot)/launch/robot.launch" />
  
  <!-- WebViz visualization -->
  <node name="webviz" pkg="webviz" type="websocket" output="screen">
    <param name="config_file" value="$(find webviz)/config/webviz_config.lua" />
    <param name="v" value="1" />  <!-- Verbose logging -->
  </node>
</launch>
```

#### ROS2 Launch File Example (Python)

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to webviz config
    webviz_config = os.path.join(
        get_package_share_directory('webviz'),
        'config',
        'webviz_config.lua'
    )
    
    return LaunchDescription([
        # Your existing robot launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('your_robot'), 
                '/launch/robot.launch.py'
            ])
        ),
        
        # WebViz node
        Node(
            package='webviz',
            executable='websocket',
            name='webviz',
            output='screen',
            parameters=[{
                'config_file': webviz_config,
                'v': 1  # Verbose logging
            }]
        )
    ])
```

## Technical Details

For implementation details, protocol specifications, and architecture information, see [Technical Documentation](src/websocket/README.md).