//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 * \file    websocket_main.cpp
 * \brief   Main entry point for websocket bridge, ROS1/ROS2 cross-compat.
 * \author  (original) Joydeep Biswas, (C) 2019
 * \author  (refactor) Your Name
 */
//========================================================================

#include <QtCore/QCoreApplication>
#include <csignal>
#include <pthread.h>
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "websocket.h"
#include "ros_adapter.h"

#ifdef USE_ROS1
  #include <ros/ros.h>
  #include "util/timer.h"  // For RateLoop
#endif

#ifdef USE_ROS2
  #include "rclcpp/rclcpp.hpp"
#endif

DEFINE_double(fps, 10.0, "Max visualization frame rate.");
DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");
DECLARE_int32(v);

static bool run_ = true;
static std::shared_ptr<RobotWebSocket> server_;
static std::shared_ptr<RosAdapter> ros_adapter_;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

/**
 * RosThread
 * A single thread function that sets up and spins either ROS1 or ROS2.
 * This function is invoked by pthread_create(...) in main().
 */
void* RosThread(void*) {
  pthread_detach(pthread_self());
  CHECK_NOTNULL(server_.get());  // Just a safety check

#ifdef USE_ROS1
  // ROS1: Create NodeHandle, init the RosAdapter for ROS1
  ros::NodeHandle nh;
  ros_adapter_->InitRos1(nh);

  RateLoop loop(FLAGS_fps);
  while (ros::ok() && run_) {
    ros::spinOnce();
    // You could call server_->SendUpdate() or similar, if needed
    loop.Sleep();
  }
  // If we exit the loop, thread terminates
  pthread_exit(nullptr);
  return nullptr;
#endif

#ifdef USE_ROS2
  // ROS2: Create node, init the RosAdapter for ROS2
  auto node = rclcpp::Node::make_shared("websocket_node");
  ros_adapter_->InitRos2(node);

  rclcpp::Rate loop(FLAGS_fps);
  while (rclcpp::ok() && run_) {
    rclcpp::spin_some(node);
    // Possibly call server_->SendUpdate() or something similar
    loop.sleep();
  }
  // If done, shutdown ROS2 before exiting
  rclcpp::shutdown();
  pthread_exit(nullptr);
  return nullptr;
#endif

  // If neither USE_ROS1 nor USE_ROS2 is defined, we do nothing 
  pthread_exit(nullptr);
  return nullptr;
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, false);

#ifdef USE_ROS1
  // Initialize ROS1 but disable the default SIGINT handler
  ros::init(argc, argv, "websocket", ros::init_options::NoSigintHandler);
#endif

#ifdef USE_ROS2
  // Initialize ROS2
  rclcpp::init(argc, argv);
#endif

  // Set up Qt application (for your WebSocket server in the main thread)
  QCoreApplication qt_app(argc, argv);

  // Create the RobotWebSocket in a shared_ptr so RosAdapter can hold a weak_ptr
  server_ = std::make_shared<RobotWebSocket>(10272);

  // When the WebSocket server closes, quit the Qt event loop
  QObject::connect(server_.get(), &RobotWebSocket::closed,
                   &qt_app, &QCoreApplication::quit);

  // Construct the RosAdapter, passing it a weak_ptr to the server
  ros_adapter_ = std::make_shared<RosAdapter>(server_);

  // Install our custom SIGINT handler
  signal(SIGINT, SignalHandler);

  // Start the single RosThread for either ROS1 or ROS2
  pthread_t ros_thread_id = 0;
  pthread_create(&ros_thread_id, nullptr, &RosThread, nullptr);

  // Now run the Qt event loop in this main thread
  const int retval = qt_app.exec();

  // Once Qt event loop ends, signal the ROS thread to stop
  run_ = false;

  // Join the thread so we cleanly shut down
  pthread_join(ros_thread_id, nullptr);

  return retval;
}
