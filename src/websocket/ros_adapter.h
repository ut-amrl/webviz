#pragma once

#include <memory>

// Forward-declare RobotWebSocket so we can store a (weak) pointer to it.
class RobotWebSocket;

// If building ROS1:
#ifdef USE_ROS1
  #include <ros/ros.h>
  #include "amrl_msgs/Localization2DMsg.h"
  #include "amrl_msgs/VisualizationMsg.h"
  #include "amrl_msgs/GPSArrayMsg.h"
  #include "amrl_msgs/GPSMsg.h"
  #include "amrl_msgs/graphNavGPSSrv.h"
  #include "geometry_msgs/PoseWithCovarianceStamped.h"
  #include "geometry_msgs/PoseStamped.h"
  #include "sensor_msgs/LaserScan.h"
  #include "std_msgs/Empty.h"
#endif

// If building ROS2:
#ifdef USE_ROS2
  #include "rclcpp/rclcpp.hpp"
  #include "amrl_msgs/msg/localization2_d_msg.hpp"
  #include "amrl_msgs/msg/visualization_msg.hpp"
  #include "amrl_msgs/msg/gps_array_msg.hpp"
  #include "amrl_msgs/msg/gps_msg.hpp"
  #include "amrl_msgs/srv/graph_nav_gps_srv.hpp"
  #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
  #include "geometry_msgs/msg/pose_stamped.hpp"
  #include "sensor_msgs/msg/laser_scan.hpp"
  #include "std_msgs/msg/empty.hpp"
#endif

///
/// RosAdapter interface that can be used by websocket_main.cc to set up
/// publishers/subscribers/services in either ROS1 or ROS2, depending on #ifdef.
///
class RosAdapter {
 public:
  explicit RosAdapter(std::weak_ptr<RobotWebSocket> ws);

#ifdef USE_ROS1
  // Called by the ROS1 version of the code to initialize everything.
  void InitRos1(ros::NodeHandle& nh);
#endif

#ifdef USE_ROS2
  // Called by the ROS2 version of the code to initialize everything.
  void InitRos2(std::shared_ptr<rclcpp::Node> node);
#endif

 private:
  // We keep a weak reference to the main RobotWebSocket (or whichever class
  // you want to notify/modify) so we can call its methods from callbacks.
  std::weak_ptr<RobotWebSocket> ws_;

#ifdef USE_ROS1
  // ROS1 Publishers/Subscribers/Services
  ros::Subscriber laser_sub_;
  ros::Subscriber laser_lowbeam_sub_;
  ros::Subscriber vis_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber gps_pose_sub_;

  ros::Publisher init_loc_pub_;
  ros::Publisher amrl_init_loc_pub_;
  ros::Publisher nav_goal_pub_;
  ros::Publisher amrl_nav_goal_pub_;
  ros::Publisher reset_nav_goals_pub_;
  ros::ServiceClient gps_service_client_;

  // ROS1 callbacks:
  void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg);
  void LaserCallback(const sensor_msgs::LaserScan& msg);
  void LaserLowBeamCallback(const sensor_msgs::LaserScan& msg);
  void VisualizationCallback(const amrl_msgs::VisualizationMsg& msg);
  void GPSPoseCallback(const amrl_msgs::GPSMsg& msg);
#endif

#ifdef USE_ROS2
  // ROS2 Publishers/Subscribers/Services
  rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_lowbeam_sub_;
  rclcpp::Subscription<amrl_msgs::msg::VisualizationMsg>::SharedPtr vis_sub_;
  rclcpp::Subscription<amrl_msgs::msg::GPSMsg>::SharedPtr gps_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_loc_pub_;
  rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr amrl_init_loc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
  rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr amrl_nav_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_nav_goals_pub_;
  rclcpp::Client<amrl_msgs::srv::GraphNavGPSSrv>::SharedPtr gps_service_client_;

  // ROS2 callbacks:
  void LocalizationCallbackRos2(const amrl_msgs::msg::Localization2DMsg::SharedPtr msg);
  void LaserCallbackRos2(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void LaserLowBeamCallbackRos2(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void VisualizationCallbackRos2(const amrl_msgs::msg::VisualizationMsg::SharedPtr msg);
  void GPSPoseCallbackRos2(const amrl_msgs::msg::GPSMsg::SharedPtr msg);
#endif
};

