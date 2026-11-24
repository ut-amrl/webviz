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
                     python3-colcon-common-extensions libgtest-dev liblua5.2-dev \
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

The **Bags** badge above the tabs turns green and reads **Recording** when
`/recording` (`std_msgs/Int32`) reports `1`, and gray with **Idle** for `0`.
It shows **Unknown** while disconnected or after three seconds without recorder
feedback. The badge is visible on every tab.

In the `memo_ws` workspace, build the Map tab's offline Austin tiles once with
`scripts/build_map_tiles.sh` inside the container, then serve WebViz with
`scripts/serve_webviz.py`. The map checks that the tiles exist before loading
them and reports offline-rendering failures separately from GPS route status.
The basemap never falls back to public tile servers. Open it through the HTTP
server, including from a laptop using the robot URL or an SSH tunnel for ports
10273 (HTTP) and 10272 (WebSocket). A standalone HTML file cannot access the tiles.

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

## MEMO workspace and bag replay

The Robot tab has a wide policy observation panel, the local point-cloud BEV,
and editable language preferences. The observation shows the sampled plan in cyan
and a heading-up satellite goal route in its top-right corner. The satellite is composited directly into the annotated egocentric JPEG pixels,
with a maximum height of 35% of that image's height. The circular crop has no white corners or outline; the camera stays visible
around it. The browser displays this single composite, and `/memo/plan_image/compressed` publishes the same JPEG. Panel height follows the content width
and is capped at 340 px; panel borders have no gutters or card padding. On narrow
screens the panels scroll horizontally. Rollouts are thin blue lines at 60% opacity. The raw policy prediction is cyan
and the selected tracking path is red. BEV coordinates are robot-local, with
forward up and left to the left; the Map tab handles global GPS goals.

Preference rows save when editing finishes. Use **+** to add a row, the red **−**
to remove it, and the gray **Reset** to restore the planner's configured
`base_preferences`. An empty list intentionally removes all base preferences.
The planner acknowledges the applied list on `/memo/preferences_state` and uses
it on subsequent requests; changing preferences does not trigger robot motion.

`/memo/ui_frame` (`std_msgs/String` JSON, version 1) records the observation stamp,
exact unrounded policy route and waypoints, task, applied preferences and defaults,
GPS placement, and both rendered JPEGs in one message. It is published even without
a browser. Satellite rendering shares MEMO's dataset route renderer and reads only
`$MEMO_WORK_DIR/geolocalizer/labeling/tiles/{z}/{x}/{y}.jpg`; it never downloads tiles.
Missing GPS placement or cached tiles is shown explicitly. Phone GPS heading is in
degrees clockwise from north; stamped odometry carries each fix to the policy
observation before cropping the satellite image.

After building `webviz`, `graph_navigation`, and `memo_deployment`, source the
workspace inside the deployment container and run:

```bash
scripts/replay_memo_ui.sh /path/to/bag
```

Open `http://localhost:10273/webviz.html?autoconnect=1&replay=1#robot`. The helper
starts the HTTP and websocket servers and plays only display topics with the ROS
clock. It does not start inference or robot drivers. `replay=1` makes tasks and
preferences read-only. Existing servers should be stopped before running the
helper, since it uses the standard ports 10272 and 10273.

The recorder inventory includes `/memo/ui_frame`, `/memo/preferences_state`,
`/memo/preferences`, `/visualization`, `/scan`, and `/localization`. Replay uses
recorded image bytes and navigation geometry, so it needs neither imagery caches
nor a policy checkpoint. `/memo/preferences_state` is transient-local, with a
recorder/playback QoS override to recover defaults even when recording starts late.
Bags made before these display artifacts were recorded cannot reproduce the
original sampled policy output exactly by rerunning stochastic inference.


### Offline satellite coverage

The citywide cache covers the full Austin offline-map rectangle:
west **-98.05**, south **30.05**, east **-97.45**, north **30.60**. This includes
East Austin and Mueller. Populate or resume it from the workspace:

```bash
scripts/cache_memo_satellite.py --austin --workers 24
scripts/cache_memo_satellite.py --central-austin --workers 24
```

The citywide layer uses zoom 17 (about 1 m/pixel); the additional Central Austin
layer uses zoom 19 (about 0.26 m/pixel). Both live under
`data/geolocalizer/labeling/tiles` by default, or the same relative path under
`MEMO_WORK_DIR`. The shared MEMO renderer prefers native zoom-19 tiles and crops
cached parent tiles elsewhere, without downloading or writing enlarged copies.
Existing images are checked and reused, corrupt entries are repaired, and failed
requests are retried. The `austin_z17.json` and `central_austin_z19.json` manifests
report verified coverage and any remaining gaps. These files persist across
sessions; the live robot does not need Wi-Fi after the cache is populated.

The camera overlay uses MEMO's ground-plane projection of predicted XY, matching
evaluation instead of projecting noise in the predicted Z channel. Raw XYZ policy
waypoints remain intact. The satellite route uses MEMO's shared `route_inset`
renderer at the actual inset size, preserving line and marker widths.
