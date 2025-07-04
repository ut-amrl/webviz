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
#include "config_reader/config_reader.h"
#include "websocket.h"
#include "ros_compat.h"

using std::vector;

DEFINE_double(fps, 10.0, "Max visualization frames rate.");
DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");
DEFINE_string(config_file, "config/webviz_config.lua", "Name of config file to use");
DECLARE_int32(v);

// Configuration variables using config-reader macros
CONFIG_INT(websocket_port, "websocket.port");
CONFIG_DOUBLE(update_rate_hz, "websocket.update_rate_hz");
CONFIG_DOUBLE(message_timeout_sec, "websocket.message_timeout_sec");
CONFIG_INT(exit_check_interval_ms, "websocket.exit_check_interval_ms");

CONFIG_STRING(ros_node_name, "ros_node.name");
CONFIG_INT(laser_queue_size, "ros_node.queue_sizes.laser_scan");
CONFIG_INT(viz_queue_size, "ros_node.queue_sizes.visualization");
CONFIG_INT(loc_queue_size, "ros_node.queue_sizes.localization");
CONFIG_INT(pub_queue_size, "ros_node.queue_sizes.publishers");

CONFIG_STRING(laser_topic, "ros_topics.laser_scan");
CONFIG_STRING(viz_topic, "ros_topics.visualization");
CONFIG_STRING(loc_topic, "ros_topics.localization");
CONFIG_STRING(init_pose_std_topic, "ros_topics.initial_pose_std");
CONFIG_STRING(nav_goal_std_topic, "ros_topics.nav_goal_std");
CONFIG_STRING(init_pose_amrl_topic, "ros_topics.initial_pose_amrl");
CONFIG_STRING(nav_goal_amrl_topic, "ros_topics.nav_goal_amrl");
CONFIG_STRING(reset_goals_topic, "ros_topics.reset_nav_goals");

CONFIG_STRING(robot_frame, "frames.robot_frame");
CONFIG_STRING(world_frame, "frames.world_frame");

CONFIG_DOUBLE(laser_range_scale, "data_processing.laser_range_scale");
CONFIG_INT(protocol_nonce, "data_processing.protocol_nonce");
CONFIG_INT(text_buffer_size, "data_processing.text_buffer_size");
CONFIG_INT(map_name_buffer_size, "data_processing.map_name_buffer_size");

CONFIG_INT(verbosity, "logging.verbosity");

CONFIG_BOOL(enable_message_aging, "performance.enable_message_aging");
CONFIG_BOOL(enable_rate_limiting, "performance.enable_rate_limiting");
CONFIG_INT(thread_sleep_usec, "performance.thread_sleep_usec");

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

// Track current subscriptions for dynamic reconfiguration
#ifdef ROS2
SubscriberPtr<LaserScan> laser_sub_;
SubscriberPtr<VisualizationMsg> vis_sub_;
SubscriberPtr<Localization2DMsg> localization_sub_;
#else
SubscriberPtr<LaserScan> laser_sub_;
SubscriberPtr<VisualizationMsg> vis_sub_;
SubscriberPtr<Localization2DMsg> localization_sub_;
#endif

// Track current topic names to detect changes
std::string current_laser_topic_;
std::string current_viz_topic_;
std::string current_loc_topic_;

// Track current configuration for comprehensive monitoring
struct CurrentConfig {
    // Subscriber topics
    std::string laser_topic;
    std::string viz_topic;
    std::string loc_topic;

    // Publisher topics
    std::string init_pose_std_topic;
    std::string nav_goal_std_topic;
    std::string init_pose_amrl_topic;
    std::string nav_goal_amrl_topic;
    std::string reset_goals_topic;

    // Frames
    std::string robot_frame;
    std::string world_frame;

    // WebSocket settings
    int websocket_port;
    double update_rate_hz;
    double message_timeout_sec;

    // Queue sizes
    int laser_queue_size;
    int viz_queue_size;
    int loc_queue_size;
    int pub_queue_size;
};

CurrentConfig current_config_;
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
    if (msg.header.frame_id != CONFIG_robot_frame &&
        msg.header.frame_id != CONFIG_world_frame) {
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
    const double max_age = CONFIG_message_timeout_sec;
#ifdef ROS2
    if ((now - rclcpp::Time(laser_scan_.header.stamp)).seconds() > max_age) {
        laser_scan_.header.stamp = ZERO_TIME();
    }
    std::remove_if(
        vis_msgs_.begin(),
        vis_msgs_.end(),
        [&now, max_age](const VisualizationMsg &m) {
            return ((now - rclcpp::Time(m.header.stamp)).seconds() > max_age);
        });
#else
    if ((now - laser_scan_.header.stamp).toSec() > max_age) {
        laser_scan_.header.stamp = ZERO_TIME();
    }
    std::remove_if(
        vis_msgs_.begin(),
        vis_msgs_.end(),
        [&now, max_age](const VisualizationMsg &m) {
            return ((now - m.header.stamp).toSec() > max_age);
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
        if (m.header.frame_id == CONFIG_world_frame) {
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

// Function to create or recreate ROS subscriptions
void CreateSubscriptions() {
    if (FLAGS_v > 0) {
        printf("Creating ROS subscriptions:\n");
        printf("  Laser: %s\n", CONFIG_laser_topic.c_str());
        printf("  Visualization: %s\n", CONFIG_viz_topic.c_str());
        printf("  Localization: %s\n", CONFIG_loc_topic.c_str());
    }

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

    laser_sub_ = CREATE_SUBSCRIBER(node_, LaserScan, CONFIG_laser_topic, CONFIG_laser_queue_size, laser_callback);
    vis_sub_ = CREATE_SUBSCRIBER(node_, VisualizationMsg, CONFIG_viz_topic, CONFIG_viz_queue_size, vis_callback);
    localization_sub_ = CREATE_SUBSCRIBER(node_, Localization2DMsg, CONFIG_loc_topic, CONFIG_loc_queue_size, loc_callback);
#else
    laser_sub_ = CREATE_SUBSCRIBER(node_, LaserScan, CONFIG_laser_topic, CONFIG_laser_queue_size, &LaserCallback);
    vis_sub_ = CREATE_SUBSCRIBER(node_, VisualizationMsg, CONFIG_viz_topic, CONFIG_viz_queue_size, &VisualizationCallback);
    localization_sub_ = CREATE_SUBSCRIBER(node_, Localization2DMsg, CONFIG_loc_topic, CONFIG_loc_queue_size, &LocalizationCallback);
#endif

    // Update tracked topic names
    current_laser_topic_ = CONFIG_laser_topic;
    current_viz_topic_ = CONFIG_viz_topic;
    current_loc_topic_ = CONFIG_loc_topic;
}

// Function to create or recreate ROS publishers
void CreatePublishers() {
    if (FLAGS_v > 0) {
        printf("Creating ROS publishers:\n");
        printf("  Initial pose (std): %s\n", CONFIG_init_pose_std_topic.c_str());
        printf("  Nav goal (std): %s\n", CONFIG_nav_goal_std_topic.c_str());
        printf("  Initial pose (AMRL): %s\n", CONFIG_init_pose_amrl_topic.c_str());
        printf("  Nav goal (AMRL): %s\n", CONFIG_nav_goal_amrl_topic.c_str());
        printf("  Reset goals: %s\n", CONFIG_reset_goals_topic.c_str());
    }

    init_loc_pub_ = CREATE_PUBLISHER(node_, PoseWithCovarianceStamped, CONFIG_init_pose_std_topic, CONFIG_pub_queue_size);
    nav_goal_pub_ = CREATE_PUBLISHER(node_, PoseStamped, CONFIG_nav_goal_std_topic, CONFIG_pub_queue_size);
    amrl_init_loc_pub_ = CREATE_PUBLISHER(node_, Localization2DMsg, CONFIG_init_pose_amrl_topic, CONFIG_pub_queue_size);
    amrl_nav_goal_pub_ = CREATE_PUBLISHER(node_, Localization2DMsg, CONFIG_nav_goal_amrl_topic, CONFIG_pub_queue_size);
    reset_nav_goals_pub_ = CREATE_PUBLISHER(node_, Empty, CONFIG_reset_goals_topic, CONFIG_pub_queue_size);
}

// Function to capture current configuration state
void CaptureCurrentConfig() {
    current_config_.laser_topic = CONFIG_laser_topic;
    current_config_.viz_topic = CONFIG_viz_topic;
    current_config_.loc_topic = CONFIG_loc_topic;

    current_config_.init_pose_std_topic = CONFIG_init_pose_std_topic;
    current_config_.nav_goal_std_topic = CONFIG_nav_goal_std_topic;
    current_config_.init_pose_amrl_topic = CONFIG_init_pose_amrl_topic;
    current_config_.nav_goal_amrl_topic = CONFIG_nav_goal_amrl_topic;
    current_config_.reset_goals_topic = CONFIG_reset_goals_topic;

    current_config_.robot_frame = CONFIG_robot_frame;
    current_config_.world_frame = CONFIG_world_frame;

    current_config_.websocket_port = CONFIG_websocket_port;
    current_config_.update_rate_hz = CONFIG_update_rate_hz;
    current_config_.message_timeout_sec = CONFIG_message_timeout_sec;

    current_config_.laser_queue_size = CONFIG_laser_queue_size;
    current_config_.viz_queue_size = CONFIG_viz_queue_size;
    current_config_.loc_queue_size = CONFIG_loc_queue_size;
    current_config_.pub_queue_size = CONFIG_pub_queue_size;
}

// Comprehensive configuration monitoring and updating
bool CheckAndUpdateConfiguration() {
    bool subscribers_changed = false;
    bool publishers_changed = false;
    bool other_changed = false;

    // Check subscriber topics
    if (current_config_.laser_topic != CONFIG_laser_topic ||
        current_config_.viz_topic != CONFIG_viz_topic ||
        current_config_.loc_topic != CONFIG_loc_topic) {
        subscribers_changed = true;
    }

    // Check publisher topics
    if (current_config_.init_pose_std_topic != CONFIG_init_pose_std_topic ||
        current_config_.nav_goal_std_topic != CONFIG_nav_goal_std_topic ||
        current_config_.init_pose_amrl_topic != CONFIG_init_pose_amrl_topic ||
        current_config_.nav_goal_amrl_topic != CONFIG_nav_goal_amrl_topic ||
        current_config_.reset_goals_topic != CONFIG_reset_goals_topic) {
        publishers_changed = true;
    }

    // Check other parameters (frames, rates, etc.)
    if (current_config_.robot_frame != CONFIG_robot_frame ||
        current_config_.world_frame != CONFIG_world_frame ||
        current_config_.update_rate_hz != CONFIG_update_rate_hz ||
        current_config_.message_timeout_sec != CONFIG_message_timeout_sec ||
        current_config_.websocket_port != CONFIG_websocket_port) {
        other_changed = true;
    }

    if (subscribers_changed || publishers_changed || other_changed) {
        if (FLAGS_v > 0) {
            printf("=== WebViz Configuration Change Detected ===\n");
        }

        // Handle subscriber changes
        if (subscribers_changed) {
            if (FLAGS_v > 0) {
                printf("Updating ROS topic subscriptions:\n");
                if (current_config_.laser_topic != CONFIG_laser_topic) {
                    printf("  Laser scan topic: '%s' -> '%s'\n", current_config_.laser_topic.c_str(), CONFIG_laser_topic.c_str());
                }
                if (current_config_.viz_topic != CONFIG_viz_topic) {
                    printf("  Visualization topic: '%s' -> '%s'\n", current_config_.viz_topic.c_str(), CONFIG_viz_topic.c_str());
                }
                if (current_config_.loc_topic != CONFIG_loc_topic) {
                    printf("  Localization topic: '%s' -> '%s'\n", current_config_.loc_topic.c_str(), CONFIG_loc_topic.c_str());
                }
            }

            // Reset and recreate subscriptions
#ifdef ROS2
            laser_sub_.reset();
            vis_sub_.reset();
            localization_sub_.reset();
#else
            laser_sub_.shutdown();
            vis_sub_.shutdown();
            localization_sub_.shutdown();
#endif
            CreateSubscriptions();
        }

        // Handle publisher changes
        if (publishers_changed) {
            if (FLAGS_v > 0) {
                printf("Updating ROS topic publishers:\n");
                if (current_config_.init_pose_std_topic != CONFIG_init_pose_std_topic) {
                    printf("  Initial pose (std): '%s' -> '%s'\n", current_config_.init_pose_std_topic.c_str(), CONFIG_init_pose_std_topic.c_str());
                }
                if (current_config_.nav_goal_std_topic != CONFIG_nav_goal_std_topic) {
                    printf("  Nav goal (std): '%s' -> '%s'\n", current_config_.nav_goal_std_topic.c_str(), CONFIG_nav_goal_std_topic.c_str());
                }
                if (current_config_.init_pose_amrl_topic != CONFIG_init_pose_amrl_topic) {
                    printf("  Initial pose (AMRL): '%s' -> '%s'\n", current_config_.init_pose_amrl_topic.c_str(), CONFIG_init_pose_amrl_topic.c_str());
                }
                if (current_config_.nav_goal_amrl_topic != CONFIG_nav_goal_amrl_topic) {
                    printf("  Nav goal (AMRL): '%s' -> '%s'\n", current_config_.nav_goal_amrl_topic.c_str(), CONFIG_nav_goal_amrl_topic.c_str());
                }
                if (current_config_.reset_goals_topic != CONFIG_reset_goals_topic) {
                    printf("  Reset goals: '%s' -> '%s'\n", current_config_.reset_goals_topic.c_str(), CONFIG_reset_goals_topic.c_str());
                }
            }

            // Reset and recreate publishers (ROS handles cleanup automatically)
            CreatePublishers();
        }

        // Handle other parameter changes
        if (other_changed) {
            if (FLAGS_v > 0) {
                printf("Other configuration updates:\n");
                if (current_config_.robot_frame != CONFIG_robot_frame) {
                    printf("  Robot frame: '%s' -> '%s'\n", current_config_.robot_frame.c_str(), CONFIG_robot_frame.c_str());
                }
                if (current_config_.world_frame != CONFIG_world_frame) {
                    printf("  World frame: '%s' -> '%s'\n", current_config_.world_frame.c_str(), CONFIG_world_frame.c_str());
                }
                if (current_config_.update_rate_hz != CONFIG_update_rate_hz) {
                    printf("  Update rate: %.1f Hz -> %.1f Hz\n", current_config_.update_rate_hz, CONFIG_update_rate_hz);
                }
                if (current_config_.message_timeout_sec != CONFIG_message_timeout_sec) {
                    printf("  Message timeout: %.1f s -> %.1f s\n", current_config_.message_timeout_sec, CONFIG_message_timeout_sec);
                }
                if (current_config_.websocket_port != CONFIG_websocket_port) {
                    printf("  WebSocket port: %d -> %d (requires restart)\n", current_config_.websocket_port, CONFIG_websocket_port);
                }
            }
        }

        // Update our tracking of current config
        CaptureCurrentConfig();

        if (FLAGS_v > 0) {
            printf("Configuration update completed successfully!\n");
            printf("==========================================\n");
        }

        return true;
    }

    return false;
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

    node_ = CREATE_NODE(CONFIG_ros_node_name);

    // Create initial subscriptions and publishers
    CreateSubscriptions();
    CreatePublishers();

    // Capture initial configuration state for monitoring
    CaptureCurrentConfig();

    RateLoop loop(CONFIG_update_rate_hz);
    int config_check_counter = 0;
    const int config_check_interval = 10;  // Check config every 10 loops (~1 second at 10Hz)

    while (ROS_OK() && run_.load()) {
        // Periodically check for configuration changes
        if (++config_check_counter >= config_check_interval) {
            CheckAndUpdateConfiguration();
            config_check_counter = 0;
        }

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
    // Set usage message for gflags help
    google::SetUsageMessage(
        "WebViz WebSocket Server - Real-time robot visualization bridge\n"
        "Usage: " +
        std::string(argv[0]) +
        " [options]\n"
        "For more information, see README.md");

    // Parse command line flags - gflags will handle --help/--helpshort automatically and exit
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Initialize the configuration system with config-reader
    config_reader::ConfigReader config_reader({FLAGS_config_file});

    // Initialize Qt application.
    QCoreApplication app(argc, argv);
    // Initialize ROS.
    ROS_INIT(argc, argv, CONFIG_ros_node_name);
    signal(SIGINT, SignalHandler);
    signal(SIGALRM, SignalHandler);

    laser_scan_.header.stamp = ZERO_TIME();
    localization_msg_.header.stamp = ZERO_TIME();

    server_ = new RobotWebSocket(CONFIG_websocket_port);

    // Setup timer to check for exit signal
    QTimer exitTimer;
    QObject::connect(&exitTimer, &QTimer::timeout, [&app]() {
        if (!run_.load()) {
            app.quit();
        }
    });
    exitTimer.start(CONFIG_exit_check_interval_ms);  // Check based on config

    pthread_t ros_thread;
    pthread_create(&ros_thread, NULL, &RosThread, NULL);

    // Run Qt event loop
    app.exec();

    // Cleanup: ensure ROS thread stops
    run_.store(false);

    // Give ROS thread time to exit gracefully
    usleep(CONFIG_thread_sleep_usec);

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
