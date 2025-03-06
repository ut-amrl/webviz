#include "ros_adapter.h"

#ifdef USE_ROS1

#include <QtCore/QObject>
#include "websocket.h"  // where RobotWebSocket is declared

RosAdapter::RosAdapter(std::weak_ptr<RobotWebSocket> ws)
  : ws_(ws) {
}

void RosAdapter::InitRos1(ros::NodeHandle& nh) {
  // Service client
  gps_service_client_ = nh.serviceClient<amrl_msgs::graphNavGPSSrv>("graphNavGPSSrv");

  // Subscribers
  laser_sub_ = nh.subscribe("/velodyne_2dscan_high_beams", 5,
                            &RosAdapter::LaserCallback, this);
  laser_lowbeam_sub_ = nh.subscribe("/velodyne_2dscan_lowbeam", 5,
                                    &RosAdapter::LaserLowBeamCallback, this);
  vis_sub_ = nh.subscribe("/visualization", 10,
                          &RosAdapter::VisualizationCallback, this);
  localization_sub_ = nh.subscribe("/localization", 10,
                                   &RosAdapter::LocalizationCallback, this);
  gps_pose_sub_ = nh.subscribe("/vectornav/GPSHeading", 10,
                               &RosAdapter::GPSPoseCallback, this);

  // Publishers
  init_loc_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  nav_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  amrl_init_loc_pub_ = nh.advertise<amrl_msgs::Localization2DMsg>("/set_pose", 10);
  amrl_nav_goal_pub_ = nh.advertise<amrl_msgs::Localization2DMsg>("/set_nav_target", 10);
  reset_nav_goals_pub_ = nh.advertise<std_msgs::Empty>("/reset_nav_goals", 10);

  // If you need to connect RobotWebSocket signals to setX methods in RosAdapter, do so here.
  if (auto shared_ws = ws_.lock()) {
    // e.g. 
    // QObject::connect(shared_ws.get(), &RobotWebSocket::SetInitialPoseSignal,
    //                  [this](float x, float y, float theta, QString map){
    //                    // do something
    //                  });
  }
}

// ROS1 callbacks:

void RosAdapter::LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  if (auto shared_ws = ws_.lock()) {
    // Forward the data to your RobotWebSocket
    shared_ws->OnLocalizationUpdate(msg);
  }
}

void RosAdapter::LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnLaserScan(msg);
  }
}

void RosAdapter::LaserLowBeamCallback(const sensor_msgs::LaserScan& msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnLaserLowBeam(msg);
  }
}

void RosAdapter::VisualizationCallback(const amrl_msgs::VisualizationMsg& msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnVisualization(msg);
  }
}

void RosAdapter::GPSPoseCallback(const amrl_msgs::GPSMsg& msg) {
  if (auto shared_ws = ws_.lock()) {
    shared_ws->OnGPSPose(msg);
  }
}

#endif // USE_ROS1
