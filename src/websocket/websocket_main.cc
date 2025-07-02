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
 * \brief   Main entry point for websocket bridge.
 * \author  Joydeep Biswas, (C) 2019
 */
//========================================================================
#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>
#include <algorithm>
#include <vector>
#include <signal.h>
#include <atomic>
#include <unistd.h>
#include <cstdlib>

#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::VisualizationMsg;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Empty;
#else
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "amrl_msgs/Localization2DMsg.h"
using amrl_msgs::Localization2DMsg;
using amrl_msgs::VisualizationMsg;
using geometry_msgs::PoseStamped;
using geometry_msgs::PoseWithCovarianceStamped;
using sensor_msgs::LaserScan;
using std_msgs::Empty;
#endif

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/math_util.h"
#include "util/timer.h"
#include "websocket.h"
#include "ros_compat.h"

using std::vector;

DEFINE_double(fps, 10.0, "Max visualization frames rate.");
DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");
DECLARE_int32(v);

namespace {
std::atomic<bool> run_(true);
vector<VisualizationMsg> vis_msgs_;
PoseWithCovarianceStamped initial_pose_msg_;
PoseStamped nav_goal_msg_;
Localization2DMsg amrl_initial_pose_msg_;
Localization2DMsg amrl_nav_goal_msg_;
Empty reset_nav_goals_msg_;
Localization2DMsg localization_msg_;
LaserScan laser_scan_;
NodePtr node_;
PublisherPtr<PoseWithCovarianceStamped> init_loc_pub_;
PublisherPtr<Localization2DMsg> amrl_init_loc_pub_;
PublisherPtr<PoseStamped> nav_goal_pub_;
PublisherPtr<Localization2DMsg> amrl_nav_goal_pub_;
PublisherPtr<Empty> reset_nav_goals_pub_;
bool updates_pending_ = false;
RobotWebSocket *server_ = nullptr;
}  // namespace

void LocalizationCallback(const Localization2DMsg &msg) {
    localization_msg_ = msg;
}

void LaserCallback(const LaserScan &msg) {
    laser_scan_ = msg;
    updates_pending_ = true;
}

void VisualizationCallback(const VisualizationMsg &msg) {
    static bool warning_showed_ = false;
    if (msg.header.frame_id != "base_link" &&
        msg.header.frame_id != "map") {
        if (!warning_showed_) {
            fprintf(stderr,
                    "WARNING: Ignoring visualization for unknown frame '%s'."
                    " This message prints only once.\n",
                    msg.header.frame_id.c_str());
            warning_showed_ = true;
        }
        return;
    }
    auto prev_msg =
        std::find_if(vis_msgs_.begin(),
                     vis_msgs_.end(),
                     [&msg](const VisualizationMsg &m) {
                         return m.ns == msg.ns;
                     });
    if (prev_msg == vis_msgs_.end()) {
        vis_msgs_.push_back(msg);
    } else {
        *prev_msg = msg;
    }
    updates_pending_ = true;
}

template <typename T>
void MergeVector(const std::vector<T> &v1, std::vector<T> *v2) {
    v2->insert(v2->end(), v1.begin(), v1.end());
}

// Merge message m1 into m2.
void MergeMessage(const VisualizationMsg &m1,
                  VisualizationMsg *m2_ptr) {
    VisualizationMsg &m2 = *m2_ptr;
    MergeVector(m1.points, &m2.points);
    MergeVector(m1.lines, &m2.lines);
    MergeVector(m1.arcs, &m2.arcs);
    MergeVector(m1.text_annotations, &m2.text_annotations);
}

void DropOldMessages() {
    const auto now = GET_TIME();
#ifdef ROS2
    if ((now - rclcpp::Time(laser_scan_.header.stamp)).seconds() > FLAGS_max_age) {
        laser_scan_.header.stamp = ZERO_TIME();
    }
    std::remove_if(
        vis_msgs_.begin(),
        vis_msgs_.end(),
        [&now](const VisualizationMsg &m) {
            return ((now - rclcpp::Time(m.header.stamp)).seconds() > FLAGS_max_age);
        });
#else
    if ((now - laser_scan_.header.stamp).toSec() > FLAGS_max_age) {
        laser_scan_.header.stamp = ZERO_TIME();
    }
    std::remove_if(
        vis_msgs_.begin(),
        vis_msgs_.end(),
        [&now](const VisualizationMsg &m) {
            return ((now - m.header.stamp).toSec() > FLAGS_max_age);
        });
#endif
}

void SendUpdate() {
    if (server_ == nullptr || !updates_pending_) {
        return;
    }
    // DropOldMessages();
    updates_pending_ = false;
#ifdef ROS2
    if (laser_scan_.header.stamp.sec == 0 && vis_msgs_.empty()) {
        return;
    }
#else
    if (laser_scan_.header.stamp.toSec() == 0 && vis_msgs_.empty()) {
        return;
    }
#endif
    VisualizationMsg local_msgs;
    VisualizationMsg global_msgs;
    for (const VisualizationMsg &m : vis_msgs_) {
        // std::cout << m << std::endl;
        if (m.header.frame_id == "map") {
            MergeMessage(m, &global_msgs);
        } else {
            MergeMessage(m, &local_msgs);
        }
    }
    server_->Send(local_msgs,
                  global_msgs,
                  laser_scan_,
                  localization_msg_);
}

void SetInitialPose(float x, float y, float theta, QString map) {
    if (FLAGS_v > 0) {
        printf("Set initial pose: %s %f,%f, %f\n",
               map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
    }
    initial_pose_msg_.header.stamp = GET_TIME();
    initial_pose_msg_.pose.pose.position.x = x;
    initial_pose_msg_.pose.pose.position.y = y;
    initial_pose_msg_.pose.pose.orientation.w = cos(0.5 * theta);
    initial_pose_msg_.pose.pose.orientation.z = sin(0.5 * theta);
    PUBLISH(init_loc_pub_, initial_pose_msg_);
    amrl_initial_pose_msg_.header.stamp = GET_TIME();
    amrl_initial_pose_msg_.map = map.toStdString();
    amrl_initial_pose_msg_.pose.x = x;
    amrl_initial_pose_msg_.pose.y = y;
    amrl_initial_pose_msg_.pose.theta = theta;
    PUBLISH(amrl_init_loc_pub_, amrl_initial_pose_msg_);
}

void ResetNavGoals() {
    if (FLAGS_v > 0) {
        printf("Reset nav goals.\n");
    }
    PUBLISH(reset_nav_goals_pub_, reset_nav_goals_msg_);
}

void SetNavGoal(float x, float y, float theta, QString map) {
    if (FLAGS_v > 0) {
        printf("Set nav goal: %s %f,%f, %f\n",
               map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
    }
    nav_goal_msg_.header.stamp = GET_TIME();
    nav_goal_msg_.pose.position.x = x;
    nav_goal_msg_.pose.position.y = y;
    nav_goal_msg_.pose.orientation.w = cos(0.5 * theta);
    nav_goal_msg_.pose.orientation.z = sin(0.5 * theta);
    PUBLISH(nav_goal_pub_, nav_goal_msg_);
    amrl_nav_goal_msg_.header.stamp = GET_TIME();
    amrl_nav_goal_msg_.map = map.toStdString();
    amrl_nav_goal_msg_.pose.x = x;
    amrl_nav_goal_msg_.pose.y = y;
    amrl_nav_goal_msg_.pose.theta = theta;
    PUBLISH(amrl_nav_goal_pub_, amrl_nav_goal_msg_);
}

void *RosThread(void *arg) {
    // Don't detach - we need to join this thread for clean shutdown
    CHECK_NOTNULL(server_);
    QObject::connect(
        server_, &RobotWebSocket::SetInitialPoseSignal, &SetInitialPose);
    QObject::connect(
        server_, &RobotWebSocket::SetNavGoalSignal, &SetNavGoal);
    QObject::connect(
        server_, &RobotWebSocket::ResetNavGoalsSignal, &ResetNavGoals);

    node_ = CREATE_NODE("websocket");

#ifdef ROS2
    // ROS2 callback setup with lambda wrappers
    auto laser_callback = [](const LaserScan::SharedPtr msg) {
        LaserCallback(*msg);
    };
    auto vis_callback = [](const VisualizationMsg::SharedPtr msg) {
        VisualizationCallback(*msg);
    };
    auto loc_callback = [](const Localization2DMsg::SharedPtr msg) {
        LocalizationCallback(*msg);
    };

    auto laser_sub = CREATE_SUBSCRIBER(node_, LaserScan, "/scan", 5, laser_callback);
    auto vis_sub = CREATE_SUBSCRIBER(node_, VisualizationMsg, "/visualization", 10, vis_callback);
    auto localization_sub = CREATE_SUBSCRIBER(node_, Localization2DMsg, "/localization", 10, loc_callback);
#else
    auto laser_sub = CREATE_SUBSCRIBER(node_, LaserScan, "/scan", 5, &LaserCallback);
    auto vis_sub = CREATE_SUBSCRIBER(node_, VisualizationMsg, "/visualization", 10, &VisualizationCallback);
    auto localization_sub = CREATE_SUBSCRIBER(node_, Localization2DMsg, "/localization", 10, &LocalizationCallback);
#endif

    init_loc_pub_ = CREATE_PUBLISHER(node_, PoseWithCovarianceStamped, "/initialpose", 10);
    nav_goal_pub_ = CREATE_PUBLISHER(node_, PoseStamped, "/move_base_simple/goal", 10);
    amrl_init_loc_pub_ = CREATE_PUBLISHER(node_, Localization2DMsg, "/set_pose", 10);
    amrl_nav_goal_pub_ = CREATE_PUBLISHER(node_, Localization2DMsg, "/set_nav_target", 10);
    reset_nav_goals_pub_ = CREATE_PUBLISHER(node_, Empty, "/reset_nav_goals", 10);

    RateLoop loop(FLAGS_fps);
    while (ROS_OK() && run_.load()) {
        SendUpdate();
        ROS_SPIN_ONCE(node_);
        loop.Sleep();
    }
    return nullptr;
}

void SignalHandler(int) {
    if (!run_.load()) {
        printf("Force Exit.\n");
        exit(0);
    }
    printf("Exiting.\n");
    run_.store(false);
}

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);
    // Initialize Qt application.
    QCoreApplication app(argc, argv);
    // Initialize ROS.
    ROS_INIT(argc, argv, "websocket");
    signal(SIGINT, SignalHandler);
    signal(SIGALRM, SignalHandler);

    laser_scan_.header.stamp = ZERO_TIME();
    localization_msg_.header.stamp = ZERO_TIME();

    server_ = new RobotWebSocket(10272);

    // Setup timer to check for exit signal
    QTimer exitTimer;
    QObject::connect(&exitTimer, &QTimer::timeout, [&app]() {
        if (!run_.load()) {
            app.quit();
        }
    });
    exitTimer.start(100);  // Check every 100ms

    pthread_t ros_thread;
    pthread_create(&ros_thread, NULL, &RosThread, NULL);

    // Run Qt event loop
    app.exec();

    // Cleanup: ensure ROS thread stops
    run_.store(false);

    // Give ROS thread time to exit gracefully
    usleep(100000);  // 100ms

    // Wait for ROS thread to finish
    pthread_join(ros_thread, NULL);

    // Cleanup server
    delete server_;
    server_ = nullptr;

    // Shutdown ROS
    ROS_SHUTDOWN();

    // Use _exit() to bypass global destructors that cause segfaults
    // This is a known issue with ROS2 + Qt cleanup order
    _exit(0);
}
