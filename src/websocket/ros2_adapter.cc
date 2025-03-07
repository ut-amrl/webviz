#include "ros_adapter.h"

#ifdef ROS2

#include <QtCore/QObject>
#include "websocket.h"
#include "shared/math/math_util.h"

// For brevity, alias the messages:
using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::VisualizationMsg;
using amrl_msgs::msg::GPSArrayMsg;
using amrl_msgs::msg::GPSMsg;
using amrl_msgs::srv::GraphNavGPSSrv;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Empty;

// Utility to merge visualization messages
static void MergeVector(const std::vector<amrl_msgs::msg::ColoredPoint2D> &v1,
                        std::vector<amrl_msgs::msg::ColoredPoint2D> *v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}
static void MergeVectorLines(const std::vector<amrl_msgs::msg::ColoredLine2D> &v1,
                             std::vector<amrl_msgs::msg::ColoredLine2D> *v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}
static void MergeVectorArcs(const std::vector<amrl_msgs::msg::ColoredArc2D> &v1,
                            std::vector<amrl_msgs::msg::ColoredArc2D> *v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}
static void MergeVectorTexts(const std::vector<amrl_msgs::msg::ColoredText> &v1,
                             std::vector<amrl_msgs::msg::ColoredText> *v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}

// Merge one VisualizationMsg into another
static void MergeMessage(const VisualizationMsg &src, VisualizationMsg *dest) {
  MergeVector(src.points, &dest->points);
  MergeVectorLines(src.lines, &dest->lines);
  MergeVectorArcs(src.arcs, &dest->arcs);
  MergeVectorTexts(src.text_annotations, &dest->text_annotations);
}

RosAdapter::RosAdapter(std::weak_ptr<RobotWebSocket> ws)
  : ws_(ws), updates_pending_(false) {
}

// Initialize all ROS2 pubs/subs/services
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

  // If you want to connect signals from RobotWebSocket, do so here. e.g.:
  if (auto server = ws_.lock()) {
    QObject::connect(server.get(), &RobotWebSocket::SetInitialPoseSignal,
                    [this](float x, float y, float theta, QString map) {
                      this->SetInitialPose(x, y, theta, map);
                    });
    QObject::connect(server.get(), &RobotWebSocket::SetNavGoalSignal, 
                    [this](float x, float y, float theta, QString map) {
                      this->SetNavGoal(x, y, theta, map);
                    });
    QObject::connect(server.get(), &RobotWebSocket::ResetNavGoalsSignal,
                    [this]() {
                      this->ResetNavGoals();
                    });
    QObject::connect(server.get(), &RobotWebSocket::SetGPSGoalSignal, 
                    [this](float start_lat, float start_lon, float end_lat, float end_lon) {
                      this->SetGPSGoal(start_lat, start_lon, end_lat, end_lon);
                    });
  }
}

//
// Qt signals to call ROS2 functions
//

void RosAdapter::SetInitialPose(float x, float y, float theta, QString map) {
  // Debug output if verbose flag is set
  if (FLAGS_v > 0) {
    printf("Set initial pose: %s %f, %f, %f\n",
           map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
  }
  
  // Get current time from ROS2 clock
  auto now = rclcpp::Clock().now();
  
  // Fill out the initial pose message (using ROS2 types)
  initial_pose_msg_.header.stamp = now;
  // Assuming initial_pose_msg_ is of type PoseWithCovarianceStamped.
  initial_pose_msg_.pose.pose.position.x = x;
  initial_pose_msg_.pose.pose.position.y = y;
  initial_pose_msg_.pose.pose.orientation.w = std::cos(0.5 * theta);
  initial_pose_msg_.pose.pose.orientation.z = std::sin(0.5 * theta);
  init_loc_pub_->publish(initial_pose_msg_);

  // Fill out the amrl initial pose message (of type Localization2DMsg)
  amrl_initial_pose_msg_.header.stamp = now;
  amrl_initial_pose_msg_.map = map.toStdString();
  amrl_initial_pose_msg_.pose.x = x;
  amrl_initial_pose_msg_.pose.y = y;
  amrl_initial_pose_msg_.pose.theta = theta;
  amrl_init_loc_pub_->publish(amrl_initial_pose_msg_);
}

void RosAdapter::ResetNavGoals() {
  if (FLAGS_v > 0) {
    printf("Reset nav goals.\n");
  }
  // Clear the GPS goals message (assumed to have a 'data' vector)
  gps_goals_msg_.data.clear();
  reset_nav_goals_pub_->publish(reset_nav_goals_msg_);
}

void RosAdapter::SetNavGoal(float x, float y, float theta, QString map) {
  if (FLAGS_v > 0) {
    printf("Set nav goal: %s %f, %f, %f\n",
           map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
  }
  
  auto now = rclcpp::Clock().now();
  
  // Fill out the navigation goal (PoseStamped)
  nav_goal_msg_.header.stamp = now;
  nav_goal_msg_.pose.position.x = x;
  nav_goal_msg_.pose.position.y = y;
  nav_goal_msg_.pose.orientation.w = std::cos(0.5 * theta);
  nav_goal_msg_.pose.orientation.z = std::sin(0.5 * theta);
  nav_goal_pub_->publish(nav_goal_msg_);

  // Fill out the amrl nav goal message (Localization2DMsg)
  amrl_nav_goal_msg_.header.stamp = now;
  amrl_nav_goal_msg_.map = map.toStdString();
  amrl_nav_goal_msg_.pose.x = x;
  amrl_nav_goal_msg_.pose.y = y;
  amrl_nav_goal_msg_.pose.theta = theta;
  amrl_nav_goal_pub_->publish(amrl_nav_goal_msg_);
}

void RosAdapter::SetGPSGoal(float start_lat, float start_lon, float end_lat, float end_lon) {
  if (FLAGS_v > 0) {
    printf("Set gps goal: %f, %f\n", end_lat, end_lon);
  }

  // Create and populate the service request.
  // Here we assume you have a member variable 'srv_' of type GraphNavGPSSrv::Request.
  auto now = rclcpp::Clock().now();
  


  gps_service_request_.start.header.stamp = now;
  gps_service_request_.start.latitude = start_lat;
  gps_service_request_.start.longitude = start_lon;

  gps_goals_msg_.header.stamp = now;
  gps_goals_msg_.data.clear();
  
  // Create a GPSMsg goal and add it to gps_goals_msg_
  GPSMsg goal_msg;
  goal_msg.header.stamp = now;
  goal_msg.latitude = end_lat;
  goal_msg.longitude = end_lon;
  gps_goals_msg_.data = { goal_msg };

  gps_service_request_.goals = gps_goals_msg_;

  // Synchronous service call (for demonstration)
  // In a production system, consider using async_send_request and handling the future.
  auto result_future = gps_service_client_->async_send_request(std::make_shared<GraphNavGPSSrv::Request>(gps_service_request_));
  
  // Wait for the result (you might add a timeout in production)
  if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared("temp_node"), result_future)
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result_future.get();
    gps_goals_msg_ = response->plan;
    if (FLAGS_v > 0) {
      for (const auto &gps_goal : gps_goals_msg_.data) {
        RCLCPP_INFO(rclcpp::get_logger("websocket"), "Path point: %f, %f",
                    gps_goal.latitude, gps_goal.longitude);
      }
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("websocket"), "Service call failed");
  }
}

//
// ROS2 callbacks - store messages in private variables
//
void RosAdapter::LocalizationCallbackRos2(const Localization2DMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  localization_msg_ = *msg;
  updates_pending_ = true;
}

void RosAdapter::LaserCallbackRos2(const LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  laser_scan_ = *msg;
  updates_pending_ = true;
}

void RosAdapter::LaserLowBeamCallbackRos2(const LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  laser_lowbeam_scan_ = *msg;
  updates_pending_ = true;
}

void RosAdapter::VisualizationCallbackRos2(const VisualizationMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  // We can store multiple VisualizationMsgs in a buffer, or just keep the latest.
  // If you want to combine them, you could push_back them in a vector.
  // For demonstration, let's say we combine them into one global `vis_msg_`.
  // If you want local vs global frames, you'd do more checks.
  // For now, let's just merge them all.
  MergeMessage(*msg, &vis_msg_);
  updates_pending_ = true;
}

void RosAdapter::GPSPoseCallbackRos2(const GPSMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  robot_gps_msg_ = *msg;
  updates_pending_ = true;
}

//
// Called from outside (e.g. after spin_some in websocket_main.cc) to
// push updates to the RobotWebSocket if anything changed.
//
void RosAdapter::SendUpdate() {
  // Acquire lock for thread-safe access
  std::lock_guard<std::mutex> lock(mutex_);

  // If we haven't gotten any updates, skip
  if (!updates_pending_) {
    return;
  }

  // Attempt to lock the shared pointer to the WebSocket server
  auto server = ws_.lock();
  if (!server) {
    // The WebSocket is gone
    return;
  }

  // Now we can break down the single `vis_msg_` if you want local vs global.
  // Let's assume all are global except for some that have frame_id == "base_link".
  VisualizationMsg local_msgs, global_msgs;
  // We'll do a very naive approach: if frame_id == "map", treat as global, else local
  // If your messages store the frame in `vis_msg_.header.frame_id`, you might do:
  // But we combined multiple messages, so let's just pretend it's one big bag:
  if (vis_msg_.header.frame_id == "map") {
    global_msgs = vis_msg_;
  } else {
    local_msgs = vis_msg_;
  }

  // Send everything to the server
  server->Send(local_msgs,
               global_msgs,
               laser_scan_,
               laser_lowbeam_scan_,
               localization_msg_,
               gps_goals_msg_,
               robot_gps_msg_);

  // Clear updates_pending_ until new data arrives
  updates_pending_ = false;
}

#endif // ROS2
