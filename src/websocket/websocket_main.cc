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
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <signal.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/foresight_planner_msg.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/math_util.h"
#include "util/timer.h"
#include "config_reader/config_reader.h"
#include "websocket.h"

using amrl_msgs::msg::ForesightPlannerMsg;
using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::VisualizationMsg;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::LaserScan;
using std::vector;
using std_msgs::msg::Empty;
using std_msgs::msg::String;

DEFINE_double(fps, 10.0, "Max visualization frames rate.");
DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");
DEFINE_string(config_file, "", "Path to config file; defaults to the installed share/webviz/config/webviz_config.lua");
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

// Image panel configuration. Two panels (left/right) defined in lua.
CONFIG_STRING(left_image_topic, "image_panels.left.topic");
CONFIG_STRING(left_image_msg_type, "image_panels.left.msg_type");
CONFIG_INT(left_image_queue_size, "image_panels.left.queue_size");
CONFIG_STRING(right_image_topic, "image_panels.right.topic");
CONFIG_STRING(right_image_msg_type, "image_panels.right.msg_type");
CONFIG_INT(right_image_queue_size, "image_panels.right.queue_size");

CONFIG_DOUBLE(image_max_rate_hz, "image_streaming.max_rate_hz");
CONFIG_INT(image_jpeg_quality, "image_streaming.jpeg_quality");

// Foresight planner topic configuration. The webviz UI publishes the typed
// goal_command on ``command_topic`` and listens for ``ForesightPlannerMsg``
// updates on ``status_topic`` (verdict + reason + reflection_id) which it
// forwards to the browser.
CONFIG_STRING(foresight_command_topic, "foresight_planner.command_topic");
CONFIG_INT(foresight_command_topic_qos, "foresight_planner.command_topic_qos");
CONFIG_STRING(foresight_status_topic, "foresight_planner.status_topic");
CONFIG_INT(foresight_status_topic_qos, "foresight_planner.status_topic_qos");

namespace
{
    std::atomic<bool> run_(true);
    vector<VisualizationMsg> vis_msgs_;
    PoseWithCovarianceStamped initial_pose_msg_;
    PoseStamped nav_goal_msg_;
    Localization2DMsg amrl_initial_pose_msg_;
    Localization2DMsg amrl_nav_goal_msg_;
    Empty reset_nav_goals_msg_;
    Localization2DMsg localization_msg_;
    LaserScan laser_scan_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr init_loc_pub_;
    rclcpp::Publisher<Localization2DMsg>::SharedPtr amrl_init_loc_pub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr nav_goal_pub_;
    rclcpp::Publisher<Localization2DMsg>::SharedPtr amrl_nav_goal_pub_;
    rclcpp::Publisher<Empty>::SharedPtr reset_nav_goals_pub_;
    bool updates_pending_ = false;
    RobotWebSocket *server_ = nullptr;

    rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<VisualizationMsg>::SharedPtr vis_sub_;
    rclcpp::Subscription<Localization2DMsg>::SharedPtr localization_sub_;

    // Track current configuration for comprehensive monitoring
    struct CurrentConfig
    {
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

        // Image panels and foresight planner.
        std::string left_image_topic;
        std::string left_image_msg_type;
        int left_image_queue_size;
        std::string right_image_topic;
        std::string right_image_msg_type;
        int right_image_queue_size;
        double image_max_rate_hz;
        int image_jpeg_quality;
        std::string foresight_command_topic;
        int foresight_command_topic_qos;
        std::string foresight_status_topic;
        int foresight_status_topic_qos;
    };

    CurrentConfig current_config_;

    // Per-panel image streaming state.
    struct ImagePanel
    {
        std::string id;    // "left" / "right"; matches lua key
        uint32_t panel_id; // 0 / 1; sent over the wire
        std::string topic;
        std::string msg_type; // "compressed" or "raw"
        int queue_size = 1;
        rclcpp::SubscriptionBase::SharedPtr sub;
        double last_send_sec = 0.0;
    };
    std::vector<ImagePanel> image_panels_;

    // Foresight planner topic interfaces. Webviz only publishes the goal_command
    // String onto ``foresight_command_pub_``; status updates flow back through
    // ``foresight_status_sub_`` from the planner / graph_navigation.
    rclcpp::Publisher<String>::SharedPtr foresight_command_pub_;
    rclcpp::Subscription<ForesightPlannerMsg>::SharedPtr foresight_status_sub_;
} // namespace

void LocalizationCallback(const Localization2DMsg &msg)
{
    localization_msg_ = msg;
}

void LaserCallback(const LaserScan &msg)
{
    laser_scan_ = msg;
    updates_pending_ = true;
}

void VisualizationCallback(const VisualizationMsg &msg)
{
    static bool warning_showed_ = false;
    if (msg.header.frame_id != CONFIG_robot_frame &&
        msg.header.frame_id != CONFIG_world_frame)
    {
        if (!warning_showed_)
        {
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
                     [&msg](const VisualizationMsg &m)
                     {
                         return m.ns == msg.ns;
                     });
    if (prev_msg == vis_msgs_.end())
    {
        vis_msgs_.push_back(msg);
    }
    else
    {
        *prev_msg = msg;
    }
    updates_pending_ = true;
}

template <typename T>
void MergeVector(const std::vector<T> &v1, std::vector<T> *v2)
{
    v2->insert(v2->end(), v1.begin(), v1.end());
}

// Merge message m1 into m2.
void MergeMessage(const VisualizationMsg &m1,
                  VisualizationMsg *m2_ptr)
{
    VisualizationMsg &m2 = *m2_ptr;
    MergeVector(m1.points, &m2.points);
    MergeVector(m1.lines, &m2.lines);
    MergeVector(m1.arcs, &m2.arcs);
    MergeVector(m1.text_annotations, &m2.text_annotations);
}

void DropOldMessages()
{
    const auto now = rclcpp::Clock().now();
    const double max_age = CONFIG_message_timeout_sec;
    if ((now - rclcpp::Time(laser_scan_.header.stamp)).seconds() > max_age)
    {
        laser_scan_.header.stamp = rclcpp::Time(0, 0);
    }
    std::remove_if(
        vis_msgs_.begin(),
        vis_msgs_.end(),
        [&now, max_age](const VisualizationMsg &m)
        {
            return ((now - rclcpp::Time(m.header.stamp)).seconds() > max_age);
        });
}

void SendUpdate()
{
    if (server_ == nullptr || !updates_pending_)
    {
        return;
    }
    updates_pending_ = false;
    if (laser_scan_.header.stamp.sec == 0 && vis_msgs_.empty())
    {
        return;
    }
    VisualizationMsg local_msgs;
    VisualizationMsg global_msgs;
    for (const VisualizationMsg &m : vis_msgs_)
    {
        if (m.header.frame_id == CONFIG_world_frame)
        {
            MergeMessage(m, &global_msgs);
        }
        else
        {
            MergeMessage(m, &local_msgs);
        }
    }
    server_->Send(local_msgs,
                  global_msgs,
                  laser_scan_,
                  localization_msg_);
}

void SetInitialPose(float x, float y, float theta, QString map)
{
    if (FLAGS_v > 0)
    {
        printf("Set initial pose: %s %f,%f, %f\n",
               map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
    }
    initial_pose_msg_.header.stamp = rclcpp::Clock().now();
    initial_pose_msg_.pose.pose.position.x = x;
    initial_pose_msg_.pose.pose.position.y = y;
    initial_pose_msg_.pose.pose.orientation.w = cos(0.5 * theta);
    initial_pose_msg_.pose.pose.orientation.z = sin(0.5 * theta);
    init_loc_pub_->publish(initial_pose_msg_);
    amrl_initial_pose_msg_.header.stamp = rclcpp::Clock().now();
    amrl_initial_pose_msg_.map = map.toStdString();
    amrl_initial_pose_msg_.pose.x = x;
    amrl_initial_pose_msg_.pose.y = y;
    amrl_initial_pose_msg_.pose.theta = theta;
    amrl_init_loc_pub_->publish(amrl_initial_pose_msg_);
}

void ResetNavGoals()
{
    if (FLAGS_v > 0)
    {
        printf("Reset nav goals.\n");
    }
    reset_nav_goals_pub_->publish(reset_nav_goals_msg_);
}

void SetNavGoal(float x, float y, float theta, QString map)
{
    if (FLAGS_v > 0)
    {
        printf("Set nav goal: %s %f,%f, %f\n",
               map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
    }
    nav_goal_msg_.header.stamp = rclcpp::Clock().now();
    nav_goal_msg_.pose.position.x = x;
    nav_goal_msg_.pose.position.y = y;
    nav_goal_msg_.pose.orientation.w = cos(0.5 * theta);
    nav_goal_msg_.pose.orientation.z = sin(0.5 * theta);
    nav_goal_pub_->publish(nav_goal_msg_);
    amrl_nav_goal_msg_.header.stamp = rclcpp::Clock().now();
    amrl_nav_goal_msg_.map = map.toStdString();
    amrl_nav_goal_msg_.pose.x = x;
    amrl_nav_goal_msg_.pose.y = y;
    amrl_nav_goal_msg_.pose.theta = theta;
    amrl_nav_goal_pub_->publish(amrl_nav_goal_msg_);
}

// Encode a sensor_msgs::Image into a JPEG QByteArray. Supports rgb8, bgr8,
// mono8 encodings. Other encodings are rejected with a one-time warning.
static bool EncodeImageToJpeg(const Image &msg, QByteArray *out)
{
    static bool warned_unknown_encoding = false;
    int cv_type;
    bool need_rgb_to_bgr = false;
    if (msg.encoding == "bgr8")
    {
        cv_type = CV_8UC3;
    }
    else if (msg.encoding == "rgb8")
    {
        cv_type = CV_8UC3;
        need_rgb_to_bgr = true;
    }
    else if (msg.encoding == "mono8")
    {
        cv_type = CV_8UC1;
    }
    else
    {
        if (!warned_unknown_encoding)
        {
            fprintf(stderr,
                    "WARNING: Unsupported image encoding '%s' on raw image "
                    "topic. Supported: rgb8, bgr8, mono8. This message "
                    "prints only once.\n",
                    msg.encoding.c_str());
            warned_unknown_encoding = true;
        }
        return false;
    }

    cv::Mat view(msg.height, msg.width, cv_type,
                 const_cast<uint8_t *>(msg.data.data()), msg.step);
    cv::Mat encoded_input;
    if (need_rgb_to_bgr)
    {
        cv::cvtColor(view, encoded_input, cv::COLOR_RGB2BGR);
    }
    else
    {
        encoded_input = view;
    }

    std::vector<uint8_t> jpeg_bytes;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,
                                     CONFIG_image_jpeg_quality};
    if (!cv::imencode(".jpg", encoded_input, jpeg_bytes, params))
    {
        fprintf(stderr, "WARNING: cv::imencode failed for raw image.\n");
        return false;
    }
    out->resize(static_cast<int>(jpeg_bytes.size()));
    memcpy(out->data(), jpeg_bytes.data(), jpeg_bytes.size());
    return true;
}

// Drop frames that arrive faster than CONFIG_image_max_rate_hz. Returns true
// if the frame should be forwarded to the websocket.
static bool ShouldForwardImage(ImagePanel *panel)
{
    if (CONFIG_image_max_rate_hz <= 0.0)
        return true;
    const double now_sec =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    const double min_interval = 1.0 / CONFIG_image_max_rate_hz;
    if (now_sec - panel->last_send_sec < min_interval)
        return false;
    panel->last_send_sec = now_sec;
    return true;
}

static void OnCompressedImage(ImagePanel *panel,
                              const CompressedImage::SharedPtr msg)
{
    if (server_ == nullptr)
        return;
    if (!ShouldForwardImage(panel))
        return;
    QByteArray jpeg(reinterpret_cast<const char *>(msg->data.data()),
                    static_cast<int>(msg->data.size()));
    const double stamp =
        msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    server_->SendImage(panel->panel_id, panel->topic, jpeg, stamp);
}

static void OnRawImage(ImagePanel *panel, const Image::SharedPtr msg)
{
    if (server_ == nullptr)
        return;
    if (!ShouldForwardImage(panel))
        return;
    QByteArray jpeg;
    if (!EncodeImageToJpeg(*msg, &jpeg))
        return;
    const double stamp =
        msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    server_->SendImage(panel->panel_id, panel->topic, jpeg, stamp);
}

// (Re)build all image-panel subscriptions from the current config.
static void CreateImagePanels()
{
    image_panels_.clear();
    struct Cfg
    {
        std::string id;
        uint32_t panel_id;
        std::string topic;
        std::string msg_type;
        int qsize;
    };
    const std::vector<Cfg> cfg = {
        {"left", 0, CONFIG_left_image_topic, CONFIG_left_image_msg_type,
         CONFIG_left_image_queue_size},
        {"right", 1, CONFIG_right_image_topic, CONFIG_right_image_msg_type,
         CONFIG_right_image_queue_size},
    };
    image_panels_.reserve(cfg.size());
    for (const auto &c : cfg)
    {
        if (c.topic.empty())
            continue;
        image_panels_.push_back({c.id, c.panel_id, c.topic, c.msg_type,
                                 c.qsize, nullptr, 0.0});
        ImagePanel *panel = &image_panels_.back();
        if (c.msg_type == "compressed")
        {
            panel->sub = node_->create_subscription<CompressedImage>(
                c.topic, c.qsize,
                [panel](const CompressedImage::SharedPtr msg)
                {
                    OnCompressedImage(panel, msg);
                });
        }
        else if (c.msg_type == "raw")
        {
            panel->sub = node_->create_subscription<Image>(
                c.topic, c.qsize,
                [panel](const Image::SharedPtr msg)
                {
                    OnRawImage(panel, msg);
                });
        }
        else
        {
            fprintf(stderr,
                    "ERROR: image_panels.%s.msg_type='%s' must be "
                    "'compressed' or 'raw'.\n",
                    c.id.c_str(), c.msg_type.c_str());
            image_panels_.pop_back();
            continue;
        }
        if (FLAGS_v > 0)
        {
            printf("  Image panel '%s' subscribed: %s [%s]\n",
                   c.id.c_str(), c.topic.c_str(), c.msg_type.c_str());
        }
    }
}

// Forward a ForesightPlannerMsg from ROS to all websocket clients.
static void OnForesightStatus(const ForesightPlannerMsg::SharedPtr msg)
{
    fprintf(stderr, "[fs] cb verdict=%d reason='%s' refl=%u server=%p\n",
            (int)msg->verdict.data, msg->reason.data.c_str(),
            msg->reflection_id, (void *)server_);
    if (server_ == nullptr)
        return;
    const QString verdict = msg->verdict.data ? "true" : "false";
    const QString reason = QString::fromStdString(msg->reason.data);
    const quint32 reflection_id = static_cast<quint32>(msg->reflection_id);
    const QString state = msg->verdict.data ? "complete" : "planning";
    server_->SendForesightStatus(state, verdict, reason, reflection_id);
}

// Build (or rebuild) the foresight publisher + status subscription.
static void CreateForesightPlannerInterfaces()
{
    foresight_command_pub_ = node_->create_publisher<String>(
        CONFIG_foresight_command_topic, CONFIG_foresight_command_topic_qos);
    foresight_status_sub_ = node_->create_subscription<ForesightPlannerMsg>(
        CONFIG_foresight_status_topic, CONFIG_foresight_status_topic_qos,
        [](const ForesightPlannerMsg::SharedPtr msg)
        {
            OnForesightStatus(msg);
        });
    if (FLAGS_v > 0)
    {
        printf("  Foresight command topic: %s\n",
               CONFIG_foresight_command_topic.c_str());
        printf("  Foresight status topic:  %s\n",
               CONFIG_foresight_status_topic.c_str());
    }
}

// Slot wired to RobotWebSocket::ForesightCommandSignal. Runs on the ROS
// thread (Qt::QueuedConnection is set up in RosThread). Publishes the
// goal_command as a String on ``foresight_command_topic``; replies flow back
// asynchronously via the foresight_status subscription.
static void HandleForesightCommand(QString text)
{
    if (foresight_command_pub_)
    {
        String msg;
        msg.data = text.toStdString();
        foresight_command_pub_->publish(msg);
    }
    if (server_)
    {
        // Immediately echo a "sent" status to acknowledge receipt; downstream
        // status updates from the planner will overwrite it.
        server_->SendForesightStatus("sent", "", "", 0);
    }
}

void CreateSubscriptions()
{
    if (FLAGS_v > 0)
    {
        printf("Creating ROS subscriptions:\n");
        printf("  Laser: %s\n", CONFIG_laser_topic.c_str());
        printf("  Visualization: %s\n", CONFIG_viz_topic.c_str());
        printf("  Localization: %s\n", CONFIG_loc_topic.c_str());
    }

    laser_sub_ = node_->create_subscription<LaserScan>(
        CONFIG_laser_topic, CONFIG_laser_queue_size,
        [](const LaserScan::SharedPtr msg)
        { LaserCallback(*msg); });
    vis_sub_ = node_->create_subscription<VisualizationMsg>(
        CONFIG_viz_topic, CONFIG_viz_queue_size,
        [](const VisualizationMsg::SharedPtr msg)
        { VisualizationCallback(*msg); });
    localization_sub_ = node_->create_subscription<Localization2DMsg>(
        CONFIG_loc_topic, CONFIG_loc_queue_size,
        [](const Localization2DMsg::SharedPtr msg)
        { LocalizationCallback(*msg); });
}

void CreatePublishers()
{
    if (FLAGS_v > 0)
    {
        printf("Creating ROS publishers:\n");
        printf("  Initial pose (std): %s\n", CONFIG_init_pose_std_topic.c_str());
        printf("  Nav goal (std): %s\n", CONFIG_nav_goal_std_topic.c_str());
        printf("  Initial pose (AMRL): %s\n", CONFIG_init_pose_amrl_topic.c_str());
        printf("  Nav goal (AMRL): %s\n", CONFIG_nav_goal_amrl_topic.c_str());
        printf("  Reset goals: %s\n", CONFIG_reset_goals_topic.c_str());
    }

    init_loc_pub_ = node_->create_publisher<PoseWithCovarianceStamped>(
        CONFIG_init_pose_std_topic, CONFIG_pub_queue_size);
    nav_goal_pub_ = node_->create_publisher<PoseStamped>(
        CONFIG_nav_goal_std_topic, CONFIG_pub_queue_size);
    amrl_init_loc_pub_ = node_->create_publisher<Localization2DMsg>(
        CONFIG_init_pose_amrl_topic, CONFIG_pub_queue_size);
    amrl_nav_goal_pub_ = node_->create_publisher<Localization2DMsg>(
        CONFIG_nav_goal_amrl_topic, CONFIG_pub_queue_size);
    reset_nav_goals_pub_ = node_->create_publisher<Empty>(
        CONFIG_reset_goals_topic, CONFIG_pub_queue_size);
}

void CaptureCurrentConfig()
{
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

    current_config_.left_image_topic = CONFIG_left_image_topic;
    current_config_.left_image_msg_type = CONFIG_left_image_msg_type;
    current_config_.left_image_queue_size = CONFIG_left_image_queue_size;
    current_config_.right_image_topic = CONFIG_right_image_topic;
    current_config_.right_image_msg_type = CONFIG_right_image_msg_type;
    current_config_.right_image_queue_size = CONFIG_right_image_queue_size;
    current_config_.image_max_rate_hz = CONFIG_image_max_rate_hz;
    current_config_.image_jpeg_quality = CONFIG_image_jpeg_quality;
    current_config_.foresight_command_topic = CONFIG_foresight_command_topic;
    current_config_.foresight_command_topic_qos =
        CONFIG_foresight_command_topic_qos;
    current_config_.foresight_status_topic = CONFIG_foresight_status_topic;
    current_config_.foresight_status_topic_qos =
        CONFIG_foresight_status_topic_qos;
}

bool CheckAndUpdateConfiguration()
{
    bool subscribers_changed = false;
    bool publishers_changed = false;
    bool other_changed = false;

    if (current_config_.laser_topic != CONFIG_laser_topic ||
        current_config_.viz_topic != CONFIG_viz_topic ||
        current_config_.loc_topic != CONFIG_loc_topic)
    {
        subscribers_changed = true;
    }

    if (current_config_.init_pose_std_topic != CONFIG_init_pose_std_topic ||
        current_config_.nav_goal_std_topic != CONFIG_nav_goal_std_topic ||
        current_config_.init_pose_amrl_topic != CONFIG_init_pose_amrl_topic ||
        current_config_.nav_goal_amrl_topic != CONFIG_nav_goal_amrl_topic ||
        current_config_.reset_goals_topic != CONFIG_reset_goals_topic)
    {
        publishers_changed = true;
    }

    if (current_config_.robot_frame != CONFIG_robot_frame ||
        current_config_.world_frame != CONFIG_world_frame ||
        current_config_.update_rate_hz != CONFIG_update_rate_hz ||
        current_config_.message_timeout_sec != CONFIG_message_timeout_sec ||
        current_config_.websocket_port != CONFIG_websocket_port)
    {
        other_changed = true;
    }

    if (current_config_.left_image_topic != CONFIG_left_image_topic ||
        current_config_.left_image_msg_type != CONFIG_left_image_msg_type ||
        current_config_.left_image_queue_size != CONFIG_left_image_queue_size ||
        current_config_.right_image_topic != CONFIG_right_image_topic ||
        current_config_.right_image_msg_type != CONFIG_right_image_msg_type ||
        current_config_.right_image_queue_size != CONFIG_right_image_queue_size ||
        current_config_.image_max_rate_hz != CONFIG_image_max_rate_hz ||
        current_config_.image_jpeg_quality != CONFIG_image_jpeg_quality ||
        current_config_.foresight_command_topic != CONFIG_foresight_command_topic ||
        current_config_.foresight_command_topic_qos !=
            CONFIG_foresight_command_topic_qos ||
        current_config_.foresight_status_topic !=
            CONFIG_foresight_status_topic ||
        current_config_.foresight_status_topic_qos !=
            CONFIG_foresight_status_topic_qos)
    {
        other_changed = true;
    }

    if (subscribers_changed || publishers_changed || other_changed)
    {
        if (FLAGS_v > 0)
        {
            printf("=== WebViz Configuration Change Detected ===\n");
        }

        if (subscribers_changed)
        {
            if (FLAGS_v > 0)
            {
                printf("Updating ROS topic subscriptions:\n");
                if (current_config_.laser_topic != CONFIG_laser_topic)
                {
                    printf("  Laser scan topic: '%s' -> '%s'\n", current_config_.laser_topic.c_str(), CONFIG_laser_topic.c_str());
                }
                if (current_config_.viz_topic != CONFIG_viz_topic)
                {
                    printf("  Visualization topic: '%s' -> '%s'\n", current_config_.viz_topic.c_str(), CONFIG_viz_topic.c_str());
                }
                if (current_config_.loc_topic != CONFIG_loc_topic)
                {
                    printf("  Localization topic: '%s' -> '%s'\n", current_config_.loc_topic.c_str(), CONFIG_loc_topic.c_str());
                }
            }

            laser_sub_.reset();
            vis_sub_.reset();
            localization_sub_.reset();
            CreateSubscriptions();
        }

        if (publishers_changed)
        {
            if (FLAGS_v > 0)
            {
                printf("Updating ROS topic publishers:\n");
                if (current_config_.init_pose_std_topic != CONFIG_init_pose_std_topic)
                {
                    printf("  Initial pose (std): '%s' -> '%s'\n", current_config_.init_pose_std_topic.c_str(), CONFIG_init_pose_std_topic.c_str());
                }
                if (current_config_.nav_goal_std_topic != CONFIG_nav_goal_std_topic)
                {
                    printf("  Nav goal (std): '%s' -> '%s'\n", current_config_.nav_goal_std_topic.c_str(), CONFIG_nav_goal_std_topic.c_str());
                }
                if (current_config_.init_pose_amrl_topic != CONFIG_init_pose_amrl_topic)
                {
                    printf("  Initial pose (AMRL): '%s' -> '%s'\n", current_config_.init_pose_amrl_topic.c_str(), CONFIG_init_pose_amrl_topic.c_str());
                }
                if (current_config_.nav_goal_amrl_topic != CONFIG_nav_goal_amrl_topic)
                {
                    printf("  Nav goal (AMRL): '%s' -> '%s'\n", current_config_.nav_goal_amrl_topic.c_str(), CONFIG_nav_goal_amrl_topic.c_str());
                }
                if (current_config_.reset_goals_topic != CONFIG_reset_goals_topic)
                {
                    printf("  Reset goals: '%s' -> '%s'\n", current_config_.reset_goals_topic.c_str(), CONFIG_reset_goals_topic.c_str());
                }
            }

            CreatePublishers();
        }

        if (other_changed)
        {
            if (FLAGS_v > 0)
            {
                printf("Other configuration updates:\n");
                if (current_config_.robot_frame != CONFIG_robot_frame)
                {
                    printf("  Robot frame: '%s' -> '%s'\n", current_config_.robot_frame.c_str(), CONFIG_robot_frame.c_str());
                }
                if (current_config_.world_frame != CONFIG_world_frame)
                {
                    printf("  World frame: '%s' -> '%s'\n", current_config_.world_frame.c_str(), CONFIG_world_frame.c_str());
                }
                if (current_config_.update_rate_hz != CONFIG_update_rate_hz)
                {
                    printf("  Update rate: %.1f Hz -> %.1f Hz\n", current_config_.update_rate_hz, CONFIG_update_rate_hz);
                }
                if (current_config_.message_timeout_sec != CONFIG_message_timeout_sec)
                {
                    printf("  Message timeout: %.1f s -> %.1f s\n", current_config_.message_timeout_sec, CONFIG_message_timeout_sec);
                }
                if (current_config_.websocket_port != CONFIG_websocket_port)
                {
                    printf("  WebSocket port: %d -> %d (requires restart)\n", current_config_.websocket_port, CONFIG_websocket_port);
                }
            }
            // Rebuild image panels if any of their fields changed.
            if (current_config_.left_image_topic != CONFIG_left_image_topic ||
                current_config_.left_image_msg_type !=
                    CONFIG_left_image_msg_type ||
                current_config_.left_image_queue_size !=
                    CONFIG_left_image_queue_size ||
                current_config_.right_image_topic != CONFIG_right_image_topic ||
                current_config_.right_image_msg_type !=
                    CONFIG_right_image_msg_type ||
                current_config_.right_image_queue_size !=
                    CONFIG_right_image_queue_size)
            {
                if (FLAGS_v > 0)
                    printf("  Rebuilding image panel subscriptions.\n");
                CreateImagePanels();
            }
            if (current_config_.foresight_command_topic !=
                    CONFIG_foresight_command_topic ||
                current_config_.foresight_command_topic_qos !=
                    CONFIG_foresight_command_topic_qos ||
                current_config_.foresight_status_topic !=
                    CONFIG_foresight_status_topic ||
                current_config_.foresight_status_topic_qos !=
                    CONFIG_foresight_status_topic_qos)
            {
                if (FLAGS_v > 0)
                    printf("  Rebuilding foresight planner interfaces.\n");
                CreateForesightPlannerInterfaces();
            }
        }

        CaptureCurrentConfig();

        if (FLAGS_v > 0)
        {
            printf("Configuration update completed successfully!\n");
            printf("==========================================\n");
        }

        return true;
    }

    return false;
}

void *RosThread(void *arg)
{
    (void)arg;
    CHECK_NOTNULL(server_);
    QObject::connect(
        server_, &RobotWebSocket::SetInitialPoseSignal, &SetInitialPose);
    QObject::connect(
        server_, &RobotWebSocket::SetNavGoalSignal, &SetNavGoal);
    QObject::connect(
        server_, &RobotWebSocket::ResetNavGoalsSignal, &ResetNavGoals);
    QObject::connect(
        server_, &RobotWebSocket::ForesightCommandSignal,
        server_,
        [](const QString &text)
        { HandleForesightCommand(text); },
        Qt::QueuedConnection);

    node_ = rclcpp::Node::make_shared(CONFIG_ros_node_name);

    CreateSubscriptions();
    CreatePublishers();
    CreateImagePanels();
    CreateForesightPlannerInterfaces();

    CaptureCurrentConfig();

    RateLoop loop(CONFIG_update_rate_hz);
    int config_check_counter = 0;
    const int config_check_interval = 10; // Check config every 10 loops (~1 second at 10Hz)

    while (rclcpp::ok() && run_.load())
    {
        if (++config_check_counter >= config_check_interval)
        {
            CheckAndUpdateConfiguration();
            config_check_counter = 0;
        }

        SendUpdate();
        rclcpp::spin_some(node_);
        loop.Sleep();
    }
    return nullptr;
}

void SignalHandler(int)
{
    if (!run_.load())
    {
        printf("Force Exit.\n");
        exit(0);
    }
    printf("Exiting.\n");
    run_.store(false);
}

int main(int argc, char *argv[])
{
    google::SetUsageMessage(
        "WebViz WebSocket Server - Real-time robot visualization bridge\n"
        "Usage: " +
        std::string(argv[0]) +
        " [options]\n"
        "For more information, see README.md");

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    std::string config_path = FLAGS_config_file;
    if (config_path.empty())
    {
        config_path =
            ament_index_cpp::get_package_share_directory("webviz") +
            "/config/webviz_config.lua";
    }
    config_reader::ConfigReader config_reader({config_path});

    QCoreApplication app(argc, argv);
    rclcpp::init(argc, argv);
    signal(SIGINT, SignalHandler);
    signal(SIGALRM, SignalHandler);

    laser_scan_.header.stamp = rclcpp::Time(0, 0);
    localization_msg_.header.stamp = rclcpp::Time(0, 0);

    server_ = new RobotWebSocket(CONFIG_websocket_port);

    QTimer exitTimer;
    QObject::connect(&exitTimer, &QTimer::timeout, [&app]()
                     {
        if (!run_.load()) {
            app.quit();
        } });
    exitTimer.start(CONFIG_exit_check_interval_ms);

    pthread_t ros_thread;
    pthread_create(&ros_thread, NULL, &RosThread, NULL);

    app.exec();

    run_.store(false);
    usleep(CONFIG_thread_sleep_usec);
    pthread_join(ros_thread, NULL);

    delete server_;
    server_ = nullptr;

    rclcpp::shutdown();

    // Use _exit() to bypass global destructors that cause segfaults
    // (known issue with ROS2 + Qt cleanup order).
    _exit(0);
}
