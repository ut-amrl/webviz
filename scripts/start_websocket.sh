#!/bin/bash
colcon build --packages-select amrl_msgs webviz
source install/setup.bash
ros2 run webviz websocket