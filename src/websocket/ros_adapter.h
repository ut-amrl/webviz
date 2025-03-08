#pragma once

#include <memory>
#include <mutex> // For std::mutex if needed

#include <QtCore/QObject>
#include "websocket_flags.h"

// Forward-declare RobotWebSocket so we can store a (weak) pointer to it.
class RobotWebSocket;

/* ------------------------------------------------------------------
 * Conditionally include ROS1 vs. ROS2 headers and define type aliases.
 * ------------------------------------------------------------------
 */
#ifdef ROS1
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

  // ---------------------
  // Using-declarations
  // ---------------------
  using LaserScan                   = sensor_msgs::LaserScan;
  using Localization2DMsg           = amrl_msgs::Localization2DMsg;
  using VisualizationMsg            = amrl_msgs::VisualizationMsg;
  using GPSArrayMsg                 = amrl_msgs::GPSArrayMsg;
  using GPSMsg                      = amrl_msgs::GPSMsg;
  using GraphNavGPSSrv              = amrl_msgs::graphNavGPSSrv; 
  using PoseWithCovarianceStamped   = geometry_msgs::PoseWithCovarianceStamped;
  using PoseStamped                 = geometry_msgs::PoseStamped;
  using Empty                       = std_msgs::Empty;
#else
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

  // ---------------------
  // Using-declarations
  // ---------------------
  using LaserScan                  = sensor_msgs::msg::LaserScan;
  using Localization2DMsg          = amrl_msgs::msg::Localization2DMsg;
  using VisualizationMsg           = amrl_msgs::msg::VisualizationMsg;
  using GPSArrayMsg                = amrl_msgs::msg::GPSArrayMsg;
  using GPSMsg                     = amrl_msgs::msg::GPSMsg;
  using GraphNavGPSSrv             = amrl_msgs::srv::GraphNavGPSSrv;
  using PoseWithCovarianceStamped  = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PoseStamped                = geometry_msgs::msg::PoseStamped;
  using Empty                      = std_msgs::msg::Empty;
#endif

///
/// RosAdapter interface that can be used by websocket_main.cc to set up
/// publishers/subscribers/services in either ROS1 or ROS2, depending on #ifdef.
///
class RosAdapter {
 public:
  explicit RosAdapter(std::weak_ptr<RobotWebSocket> ws);

  std::weak_ptr<RobotWebSocket> ws_;

#ifdef ROS1
  // Called by the ROS1 version of the code to initialize everything.
  void InitRos1(ros::NodeHandle& nh);

  // ROS1 callbacks:
 private:
  void LocalizationCallback(const Localization2DMsg& msg);
  void LaserCallback(const LaserScan& msg);
  void LaserLowBeamCallback(const LaserScan& msg);
  void VisualizationCallback(const VisualizationMsg& msg);
  void GPSPoseCallback(const GPSMsg& msg);

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
#else 
 public:
  // Called by the ROS2 version of the code to initialize everything.
  void InitRos2(std::shared_ptr<rclcpp::Node> node);

  // Provide a public method to push updates to the WebSocket
  // e.g. SendUpdate(), if you want to call it from main.
  void SendUpdate();

 private:
  rclcpp::Node::SharedPtr node_;
  // ROS2 callbacks:
  void LocalizationCallbackRos2(const Localization2DMsg::SharedPtr msg);
  void LaserCallbackRos2(const LaserScan::SharedPtr msg);
  void LaserLowBeamCallbackRos2(const LaserScan::SharedPtr msg);
  void VisualizationCallbackRos2(const VisualizationMsg::SharedPtr msg);
  void GPSPoseCallbackRos2(const GPSMsg::SharedPtr msg);
  void SetInitialPose(float x, float y, float theta, QString map);
  void ResetNavGoals();
  void SetNavGoal(float x, float y, float theta, QString map);
  void SetGPSGoal(float start_lat, float start_lon, float end_lat, float end_lon);

  // Data storage for ROS2
  LaserScan laser_scan_;
  LaserScan laser_lowbeam_scan_;
  Localization2DMsg localization_msg_;
  VisualizationMsg vis_msg_;
  GPSMsg robot_gps_msg_;
  GPSArrayMsg gps_goals_msg_;
  bool updates_pending_ = false;
  std::mutex mutex_;

  // Data storage for QT signals
  PoseWithCovarianceStamped initial_pose_msg_;
  Localization2DMsg amrl_initial_pose_msg_;
  PoseStamped nav_goal_msg_;
  Localization2DMsg amrl_nav_goal_msg_;
  Empty reset_nav_goals_msg_;

  // ROS2 Publishers/Subscribers/Services
  rclcpp::Subscription<Localization2DMsg>::SharedPtr localization_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_lowbeam_sub_;
  rclcpp::Subscription<VisualizationMsg>::SharedPtr vis_sub_;
  rclcpp::Subscription<GPSMsg>::SharedPtr gps_pose_sub_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr init_loc_pub_;
  rclcpp::Publisher<Localization2DMsg>::SharedPtr amrl_init_loc_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr nav_goal_pub_;
  rclcpp::Publisher<Localization2DMsg>::SharedPtr amrl_nav_goal_pub_;
  rclcpp::Publisher<Empty>::SharedPtr reset_nav_goals_pub_;

  rclcpp::Client<GraphNavGPSSrv>::SharedPtr gps_service_client_;
  GraphNavGPSSrv::Request gps_service_request_;
#endif // ROS2
};
