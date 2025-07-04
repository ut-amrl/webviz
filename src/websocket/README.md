# WebSocket Technical Documentation

## Overview

The WebViz WebSocket bridge provides a real-time, low-bandwidth connection between ROS robot systems and web browsers. This document covers the technical implementation details, protocol specifications, and architecture of the WebSocket subsystem.

## Architecture

### Core Components

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Web Browser   │◄───┤  WebSocket Bridge ├───►│   ROS System    │
│   (webviz.html) │    │  (websocket_main) │    │ (Topics/Nodes)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │
                                ▼
                        ┌──────────────────┐
                        │ Config System    │
                        │ (Lua files)      │
                        └──────────────────┘
```

### Key Classes and Files

- **`RobotWebSocket`** (`websocket.h/.cc`): Core WebSocket server implementation
- **`websocket_main.cc`**: Main entry point, ROS integration, and configuration management
- **`ros_compat.h`**: ROS1/ROS2 compatibility macros
- **`webviz_config.lua`**: Default configuration file

## Configuration System
WebViz uses the AMRL config-reader framework for configuration management and supports live reloading of configuration files during runtime.

### Configuration Structure

The configuration uses hierarchical Lua tables:

```lua
-- WebSocket server settings
websocket = {
  port = 10272;                    -- Server port
  update_rate_hz = 10.0;           -- Update frequency
  message_timeout_sec = 2.0;       -- Message aging threshold
  exit_check_interval_ms = 100;    -- Cleanup timing
};

-- ROS integration settings
ros_topics = {
  laser_scan = "/scan";            -- Input: LiDAR data
  visualization = "/visualization"; -- Input: Custom visualizations
  localization = "/localization";  -- Input: Robot pose
  
  initial_pose_std = "/initialpose";        -- Output: Standard nav format
  initial_pose_amrl = "/set_pose";          -- Output: AMRL format
  nav_goal_std = "/move_base_simple/goal";  -- Output: Standard nav format
  nav_goal_amrl = "/set_nav_target";        -- Output: AMRL format
  reset_nav_goals = "/reset_nav_goals";     -- Output: Reset command
};

-- Frame configuration
frames = {
  robot_frame = "base_link";       -- Robot-relative frame
  world_frame = "map";             -- Global/map frame
};
```

### Usage in Code

Configuration parameters are accessed via compile-time macros:

```cpp
// Declaration (usually at file scope)
CONFIG_INT(websocket_port, "websocket.port");
CONFIG_STRING(laser_topic, "ros_topics.laser_scan");
CONFIG_DOUBLE(update_rate_hz, "websocket.update_rate_hz");

// Initialization (in main)
config_reader::ConfigReader config_reader({FLAGS_config_file});

// Usage (anywhere after initialization)
server_ = new RobotWebSocket(CONFIG_websocket_port);
auto laser_sub = CREATE_SUBSCRIBER(node_, LaserScan, CONFIG_laser_topic, 
                                  CONFIG_laser_queue_size, &LaserCallback);
RateLoop loop(CONFIG_update_rate_hz);
```

## ROS Integration

### Cross-Version Compatibility

WebViz supports both ROS1 and ROS2 through a unified abstraction layer:

```cpp
#ifdef ROS2
  // ROS2-specific includes and types
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/laser_scan.hpp"
  using sensor_msgs::msg::LaserScan;
#else  
  // ROS1-specific includes and types
  #include "sensor_msgs/LaserScan.h"
  using sensor_msgs::LaserScan;
#endif

// Unified macros work for both versions
NodePtr node_ = CREATE_NODE(CONFIG_ros_node_name);
auto laser_sub = CREATE_SUBSCRIBER(node_, LaserScan, CONFIG_laser_topic, 
                                  CONFIG_laser_queue_size, &LaserCallback);
```

### Message Flow

1. **Input Subscribers**:
   - `LaserScan`: LiDAR/laser scanner data → converted to web format
   - `VisualizationMsg`: Custom geometry → filtered by frame and merged
   - `Localization2DMsg`: Robot pose → included in updates

2. **Output Publishers**:
   - `PoseWithCovarianceStamped`: Standard ROS navigation initial pose
   - `PoseStamped`: Standard ROS navigation goals
   - `Localization2DMsg`: AMRL-format pose and navigation commands
   - `Empty`: Reset signals

### Message Processing

```cpp
void VisualizationCallback(const VisualizationMsg &msg) {
    // Frame validation
    if (msg.header.frame_id != CONFIG_robot_frame &&
        msg.header.frame_id != CONFIG_world_frame) {
        return; // Ignore unknown frames
    }
    
    // Message deduplication by namespace
    auto prev_msg = std::find_if(vis_msgs_.begin(), vis_msgs_.end(),
                                [&msg](const VisualizationMsg &m) {
                                    return m.ns == msg.ns;
                                });
    
    // Update or append
    if (prev_msg == vis_msgs_.end()) {
        vis_msgs_.push_back(msg);
    } else {
        *prev_msg = msg;
    }
    
    updates_pending_ = true;
}
```

## WebSocket Protocol

### Connection Management

- **Port**: Configurable via `websocket.port` (default: 10272)
- **Protocol**: WebSocket (ws://) with binary message format
- **Threading**: Qt-based event loop with separate ROS processing thread

### Message Format

The protocol uses a compact binary format for efficiency:

```cpp
struct WebSocketMessage {
    uint32_t nonce;           // Protocol identifier (CONFIG_protocol_nonce)
    uint32_t timestamp;       // Message timestamp
    uint32_t laser_count;     // Number of laser points
    uint16_t laser_data[];    // Scaled laser ranges (x CONFIG_laser_range_scale)
    
    // Variable-length sections:
    LocalizationData localization;  // Robot pose and uncertainty
    VisualizationData geometry;     // Points, lines, arcs, text
    MapNameData map_info;          // Current map name
};
```

### Frame Handling

WebViz processes two coordinate frames:

- **Robot Frame** (`CONFIG_robot_frame`): Typically "base_link"
  - Moves with the robot
  - Used for sensor data, local visualizations
  
- **World Frame** (`CONFIG_world_frame`): Typically "map"  
  - Fixed global coordinate system
  - Used for maps, global paths, localization

### Data Transmission

```cpp
void SendUpdate() {
    if (!updates_pending_) return;
    
    // Separate messages by frame
    VisualizationMsg local_msgs, global_msgs;
    for (const auto& msg : vis_msgs_) {
        if (msg.header.frame_id == CONFIG_world_frame) {
            MergeMessage(msg, &global_msgs);
        } else {
            MergeMessage(msg, &local_msgs);
        }
    }
    
    // Send combined update
    server_->Send(local_msgs, global_msgs, laser_scan_, localization_msg_);
    updates_pending_ = false;
}
```

## Performance Considerations

### Message Aging

Optional message aging prevents stale data accumulation:

```cpp
void DropOldMessages() {
    const double max_age = CONFIG_message_timeout_sec;
    const auto now = GET_TIME();
    
    // Remove expired visualizations
    std::remove_if(vis_msgs_.begin(), vis_msgs_.end(),
                  [&](const VisualizationMsg& m) {
                      return (now - m.header.stamp).seconds() > max_age;
                  });
}
```

### Rate Limiting

Update frequency is controlled to prevent network overload:

```cpp
RateLoop loop(CONFIG_update_rate_hz);  // Typically 10-20 Hz
while (ROS_OK() && run_.load()) {
    SendUpdate();
    ROS_SPIN_ONCE(node_);
    loop.Sleep();  // Maintain consistent timing
}
```

### Threading Model

- **Main Thread**: Qt event loop, WebSocket connections
- **ROS Thread**: Message processing, rate-limited updates
- **Config Thread**: File watching, live configuration reloading

## Build System Integration

### CMake Configuration

The build system automatically detects ROS version and configures accordingly:

```cmake
# Auto-detect ROS version
if(${ROS_VERSION} EQUAL "1")
  # ROS1 rosbuild setup
  ROSBUILD_ADD_EXECUTABLE(websocket ...)
  TARGET_LINK_LIBRARIES(websocket Qt5::WebSockets roslib roscpp ...)
  
elseif(${ROS_VERSION} EQUAL "2") 
  # ROS2 ament setup
  add_executable(websocket ...)
  ament_target_dependencies(websocket rclcpp sensor_msgs ...)
  target_link_libraries(websocket Qt5::WebSockets ...)
endif()
```

### Dependencies

- **Qt5**: WebSockets, Core, Widgets
- **ROS**: Version-specific packages (roscpp/rclcpp, sensor_msgs, etc.)
- **AMRL**: amrl_msgs, shared utilities
- **System**: glog, gflags, Lua 5.1