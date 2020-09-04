# WebViz

A low-bandwidth websocket-based direct robot visualizer.

[![Build Status](https://travis-ci.com/ut-amrl/webviz.svg?branch=master)](https://travis-ci.com/ut-amrl/webviz)


## Dependencies

1. [ROS](http://wiki.ros.org/Installation/)
1. [AMRL ROS Messages](https://github.com/ut-amrl/amrl_msgs)
1. QT5 and QT5 WebSockets
    ```
    sudo apt install qt5-default libqt5websockets5-dev
    ```

## Build

1. Add the project directory to the `ROS_PACKAGE_PATH` environment variable.
    ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```
1. Run `make`.

## Usage

1. Start the websocket server
    ```
    ./bin/websocket
    ```
1. Open the file `webviz.html` in a web browser on the remote computer
1. Enter the IP address or hostname of the robot, and click on `Connect`.