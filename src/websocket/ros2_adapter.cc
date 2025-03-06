#include "ros_adapter.h"

#ifdef USE_ROS2

#include <QtCore/QObject>
#include "websocket.h"

using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::VisualizationMsg;
using amrl_msgs::msg::GPSArrayMsg;
using amrl_msgs::msg::GPSMsg;
using amrl_msgs::srv::GraphNavGPSSrv;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Empty;

RosAdapter::RosAdapter(std::weak_ptr<RobotWebSocket> ws)
  : ws_(ws) {
}

void RosAdapter::InitRos2(std::shared_ptr<rclcpp::Node> node) {
  // Service client
  gps_service_client_ = node->create_client<GraphNavGPSSrv>("graphNavGPSSrv");

  // Subscribers (ROS2)
  laser_sub_ = node->create_subscription<LaserScan>(
      "/velodyne_2dscan_high_beams", 5,
      std::bind(&RosAdapter::LaserCallbackRos2, this, std::placeholders::_1));

  laser_lowbeam_sub_ = node->create_subscription<LaserScan>(
      "/velodyne_2dscan_lowbeam", 5,
      std::bind(&RosAdapter::LaserLowBeamCallbackRos2, this, std::placeholders::_1));

  vis_sub_ = node->create_subscription<VisualizationMsg>(
      "/visualization", 10,
      std::bind(&RosAdapter::VisualizationCallbackRos2, this, std::placeholders::_1));

  localization_sub_ = node->create_subscription<Localization2DMsg>(
      "/localization", 10,
      std::bind(&RosAdapter::LocalizationCallbackRos2, this, std::placeholders::_1));

  gps_pose_sub_ = node->create_subscription<GPSMsg>(
      "/vectornav/GPSHeading", 10,
      std::bind(&RosAdapter::GPSPoseCallbackRos2, this, std::placeholders::_1));

  // Publishers (ROS2)
  init_loc_pub_ = node->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);
  nav_goal_pub_ = node->create_publisher<PoseStamped>("/move_base_simple/goal", 10);
  amrl_init_loc_pub_ = node->create_publisher<Localization2DMsg>("/set_pose", 10);
  amrl_nav_goal_pub_ = node->create_publisher<Localization2DMsg>("/set_nav_target", 10);
  reset_nav_goals_pub_ = node->create_publisher<Empty>("/reset_nav_goals", 10);

  // Connect RobotWebSocket signals if necessary
  if (auto shared_ws = ws_.lock()) {
    // e.g. 
    // QObject::connect(shared_ws.get(), &RobotWebSocket::SetInitialPoseSignal,
    //                  [this](float x, float y, float theta, QString map){
    //                    // do something
    //                  });
  }
}

// ROS2 callbacks:

void RosAdapter::LocalizationCallbackRos2(const Localization2DMsg::SharedPtr msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnLocalizationUpdate(*msg);
  }
}

void RosAdapter::LaserCallbackRos2(const LaserScan::SharedPtr msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnLaserScan(*msg);
  }
}

void RosAdapter::LaserLowBeamCallbackRos2(const LaserScan::SharedPtr msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnLaserLowBeam(*msg);
  }
}

void RosAdapter::VisualizationCallbackRos2(const VisualizationMsg::SharedPtr msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnVisualization(*msg);
  }
}

void RosAdapter::GPSPoseCallbackRos2(const GPSMsg::SharedPtr msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnGPSPose(*msg);
  }
}

#endif // USE_ROS2
