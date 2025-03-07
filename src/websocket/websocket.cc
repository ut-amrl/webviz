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
 * \file    websocket.cc
 * \brief   Lightweight interface to send data from a robot
 *          to a web-based visualization page.
 * \author  Joydeep Biswas, (C) 2020
 */
//========================================================================
#include "websocket.h"

#include <QtCore/QDebug>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtWebSockets/QWebSocket>
#include <QtWebSockets/QWebSocketServer>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "gflags/gflags.h"
#include "glog/logging.h"

#ifdef USE_ROS1
  #include <ros/ros.h>  // for ros::Time(0), if used
#else
  // In ROS2, you might handle time differently, but omitted for brevity
#endif

DEFINE_uint64(max_connections, 4, "Maximum number of websocket connections");

QT_USE_NAMESPACE

// Helper to write elements into a buffer
template <typename T>
char* WriteElement(const T& x, char* const buf) {
  *reinterpret_cast<T*>(buf) = x;
  return (buf + sizeof(x));
}

template <typename T>
char* WriteElementVector(const std::vector<T>& v, char* const buf) {
  const size_t len = v.size() * sizeof(T);
  memcpy(buf, v.data(), len);
  return (buf + len);
}

// -----------------------------------------------------------
// DataMessage Implementation
// -----------------------------------------------------------
QByteArray DataMessage::ToByteArray() const {
  QByteArray data;
  data.resize(header.GetByteLength());
  char* buf = data.data();

  // 1) Write the header
  buf = WriteElement(header, buf);

  // 2) Write laser scan data
  buf = WriteElementVector(laser_scan, buf);

  // 3) Write points, lines, arcs, texts
  buf = WriteElementVector(points, buf);
  buf = WriteElementVector(lines, buf);
  buf = WriteElementVector(arcs, buf);
  buf = WriteElementVector(text_annotations, buf);

  return data;
}

DataMessage DataMessage::FromRosMessages(
    const LaserScan& laser_msg,
    const LaserScan& laser_lowbeam_msg,
    const VisualizationMsg& local_msg,
    const VisualizationMsg& global_msg,
    const Localization2DMsg& localization_msg,
    const GPSArrayMsg& gps_goals_msg,
    const GPSMsg& gps_msg) {

  DataMessage msg;

  // Initialize map chars to zero
  memset(msg.header.map, 0, sizeof(msg.header.map));

  // Fill in localization pose
  msg.header.loc_x = localization_msg.pose.x;
  msg.header.loc_y = localization_msg.pose.y;
  msg.header.loc_r = localization_msg.pose.theta;

  // Fill in GPS pose
  msg.header.lat = gps_msg.latitude;
  msg.header.lng = gps_msg.longitude;
  msg.header.heading = gps_msg.heading;

  // Fill in route
  msg.header.route[0] = gps_goals_msg.data.size();
  for (size_t i = 0; i < gps_goals_msg.data.size(); i++) {
    const auto& route_node = gps_goals_msg.data[i];
    msg.header.route[ROUTE_HEADER_SIZE + i*NODE_SIZE + 0] = route_node.latitude;
    msg.header.route[ROUTE_HEADER_SIZE + i*NODE_SIZE + 1] = route_node.longitude;
    msg.header.route[ROUTE_HEADER_SIZE + i*NODE_SIZE + 2] = route_node.heading;
  }

  // Copy the map name (truncate if longer than 31 chars)
  strncpy(msg.header.map, localization_msg.map.c_str(),
          std::min(sizeof(msg.header.map) - 1, localization_msg.map.size()));

  // Laser info
  msg.header.laser_min_angle = laser_msg.angle_min;
  msg.header.laser_max_angle = laser_msg.angle_max;
  msg.header.num_laser_rays =
      laser_msg.ranges.size() + laser_lowbeam_msg.ranges.size();
  msg.laser_scan.resize(msg.header.num_laser_rays);

  // Convert LaserScan data to millimeters
  for (size_t i = 0; i < laser_msg.ranges.size(); ++i) {
    if (laser_msg.ranges[i] <= laser_msg.range_min ||
        laser_msg.ranges[i] >= laser_msg.range_max) {
      msg.laser_scan[i] = 0;
    } else {
      msg.laser_scan[i] = static_cast<uint32_t>(laser_msg.ranges[i] * 1000.0f);
    }
  }
  // Lowbeam
  for (size_t i = 0; i < laser_lowbeam_msg.ranges.size(); ++i) {
    const size_t idx = i + laser_msg.ranges.size();
    if (laser_lowbeam_msg.ranges[i] <= laser_lowbeam_msg.range_min ||
        laser_lowbeam_msg.ranges[i] >= laser_lowbeam_msg.range_max) {
      msg.laser_scan[idx] = 0;
    } else {
      msg.laser_scan[idx] =
          static_cast<uint32_t>(laser_lowbeam_msg.ranges[i] * 1000.0f);
    }
  }

  // Merge local and global points
  msg.points = local_msg.points;
  msg.header.num_local_points = local_msg.points.size();
  msg.points.insert(msg.points.end(),
                    global_msg.points.begin(), global_msg.points.end());

  // Merge local and global lines
  msg.lines = local_msg.lines;
  msg.header.num_local_lines = local_msg.lines.size();
  msg.lines.insert(msg.lines.end(),
                   global_msg.lines.begin(), global_msg.lines.end());

  // Merge arcs
  msg.arcs = local_msg.arcs;
  msg.header.num_local_arcs = local_msg.arcs.size();
  msg.arcs.insert(msg.arcs.end(),
                  global_msg.arcs.begin(), global_msg.arcs.end());

  // Text annotations
  const size_t local_text_count  = local_msg.text_annotations.size();
  const size_t global_text_count = global_msg.text_annotations.size();

  msg.header.num_local_text_annotations = local_text_count;
  msg.header.num_text_annotations = local_text_count + global_text_count;

  // Convert ColoredText -> ColoredTextNative
  for (auto& text : local_msg.text_annotations) {
    ColoredTextNative t;
    t.start = text.start;
    t.color = text.color;
    t.size_em = text.size_em;
    // Copy the text with truncation
    size_t copy_len = std::min(sizeof(t.text) - 1, text.text.size());
    strncpy(t.text, text.text.data(), copy_len);
    t.text[copy_len] = '\0';
    msg.text_annotations.push_back(t);
  }
  for (auto& text : global_msg.text_annotations) {
    ColoredTextNative t;
    t.start = text.start;
    t.color = text.color;
    t.size_em = text.size_em;
    size_t copy_len = std::min(sizeof(t.text) - 1, text.text.size());
    strncpy(t.text, text.text.data(), copy_len);
    t.text[copy_len] = '\0';
    msg.text_annotations.push_back(t);
  }

  // Summaries
  msg.header.num_points = msg.points.size();
  msg.header.num_lines  = msg.lines.size();
  msg.header.num_arcs   = msg.arcs.size();

  return msg;
}

// -----------------------------------------------------------
// RobotWebSocket Implementation
// -----------------------------------------------------------
RobotWebSocket::RobotWebSocket(uint16_t port)
    : QObject(nullptr),
      ws_server_(new QWebSocketServer(
          QStringLiteral("Robot Websocket Server"),
          QWebSocketServer::NonSecureMode,
          this)) {
#ifdef USE_ROS1
  // For ROS1, you might do something like:
  localization_.header.stamp = ros::Time(0); // This is a ROS1 call
  localization_.header.seq = 0;
#else
  // In ROS2, you might use builtin_interfaces::msg::Time etc.
  localization_.header.stamp.sec = 0;
  localization_.header.stamp.nanosec = 0;
#endif

  // Initialize some default values for demonstration:
  localization_.pose.x = 0;
  localization_.pose.y = 0;
  localization_.pose.theta = 0;
  gps_pose_.latitude  = 30.2861;   // UT Austin
  gps_pose_.longitude = -97.7394;
  gps_pose_.heading   = 0.0;

  // Connect the Qt signal that triggers sending data
  connect(this, &RobotWebSocket::SendDataSignal, this, &RobotWebSocket::SendDataSlot);

  // Listen on the given port
  if (ws_server_->listen(QHostAddress::Any, port)) {
    qDebug() << "Listening on port" << port;
    connect(ws_server_, &QWebSocketServer::newConnection,
            this, &RobotWebSocket::onNewConnection);
    connect(ws_server_, &QWebSocketServer::closed,
            this, &RobotWebSocket::closed);
  }
}

RobotWebSocket::~RobotWebSocket() {
  ws_server_->close();
  for (auto* c : clients_) {
    delete c;
  }
  clients_.clear();
}

void RobotWebSocket::onNewConnection() {
  QWebSocket* new_client = ws_server_->nextPendingConnection();
  if (clients_.size() >= FLAGS_max_connections) {
    new_client->sendTextMessage("{ \"error\": \"Too many clients\" }");
    qInfo() << "Ignoring new client" << new_client
            << ", too many existing clients:" << clients_.size();
    delete new_client;
    return;
  }
  clients_.push_back(new_client);
  qInfo() << "New client:" << new_client << ", total:" << clients_.size() << "/"
          << FLAGS_max_connections;

  connect(new_client, &QWebSocket::textMessageReceived,
          this, &RobotWebSocket::processTextMessage);
  connect(new_client, &QWebSocket::binaryMessageReceived,
          this, &RobotWebSocket::processBinaryMessage);
  connect(new_client, &QWebSocket::disconnected,
          this, &RobotWebSocket::socketDisconnected);
}

bool AllNumericalKeysPresent(const QStringList& expected, const QJsonObject& json) {
  for (const QString& key : expected) {
    if (!json.contains(key)) return false;
    if (!json.value(key).isDouble()) return false;
  }
  return true;
}

bool StringKeyPresent(const QString& key, const QJsonObject& json) {
  if (!json.contains(key)) return false;
  return json.value(key).isString();
}

void RobotWebSocket::ProcessCallback(const QJsonObject& json) {
  if (!json.contains("type")) {
    SendError("Malformed request");
    return;
  }
  const auto type = json.value("type").toString();

  // set_initial_pose
  if (type == "set_initial_pose") {
    if (!AllNumericalKeysPresent({"x","y","theta"}, json) ||
        !StringKeyPresent("map", json)) {
      SendError("Invalid set_initial_pose parameters");
      return;
    }
    emit SetInitialPoseSignal(json.value("x").toDouble(),
                              json.value("y").toDouble(),
                              json.value("theta").toDouble(),
                              json.value("map").toString());
  }
  // set_nav_goal
  else if (type == "set_nav_goal") {
    if (!AllNumericalKeysPresent({"x","y","theta"}, json) ||
        !StringKeyPresent("map", json)) {
      SendError("Invalid set_nav_goal parameters");
      return;
    }
    emit SetNavGoalSignal(json.value("x").toDouble(),
                          json.value("y").toDouble(),
                          json.value("theta").toDouble(),
                          json.value("map").toString());
  }
  // set_gps_goal
  else if (type == "set_gps_goal") {
    if (!AllNumericalKeysPresent({"start_lat","start_lon","end_lat","end_lon"}, json)) {
      SendError("Invalid set_gps_goal parameters");
      return;
    }
    emit SetGPSGoalSignal(json.value("start_lat").toDouble(),
                          json.value("start_lon").toDouble(),
                          json.value("end_lat").toDouble(),
                          json.value("end_lon").toDouble());
  }
  // reset_nav_goals
  else if (type == "reset_nav_goals") {
    emit ResetNavGoalsSignal();
  }
  else {
    SendError("Unrecognized request type");
  }
}

void RobotWebSocket::processTextMessage(QString message) {
  QWebSocket* client = qobject_cast<QWebSocket*>(sender());
  if (!client) {
    return;
  }
  QJsonDocument doc = QJsonDocument::fromJson(message.toLocal8Bit());
  QJsonObject json = doc.object();
  ProcessCallback(json);
}

void RobotWebSocket::processBinaryMessage(QByteArray message) {
  QWebSocket* client = qobject_cast<QWebSocket*>(sender());
  if (!client) {
    return;
  }
  qDebug() << "Binary Message received, echoing to client size=" << message.size();
  // Echo it back (if desired)
  client->sendBinaryMessage(message);
}

void RobotWebSocket::socketDisconnected() {
  QWebSocket* client = qobject_cast<QWebSocket*>(sender());
  auto it = std::find(clients_.begin(), clients_.end(), client);
  if (it == clients_.end()) {
    qWarning() << "Unknown socket disconnected!";
    delete client;
    return;
  }
  qDebug() << "Socket disconnected:" << client;
  delete client;
  clients_.erase(it);
}

void RobotWebSocket::SendDataSlot() {
  if (clients_.empty()) return;

  data_mutex_.lock();
  DataMessage data = DataMessage::FromRosMessages(
      laser_scan_, laser_lowbeam_scan_,
      local_vis_, global_vis_,
      localization_, gps_goals_msg_, gps_pose_);
  QByteArray buffer = data.ToByteArray();
  // Confirm the size matches the header's declared length
  CHECK_EQ(data.header.GetByteLength(), static_cast<size_t>(buffer.size()));
  // Send to all clients
  for (auto c : clients_) {
    c->sendBinaryMessage(buffer);
  }
  data_mutex_.unlock();
}

void RobotWebSocket::SendError(const QString& error_val) {
  for (auto c : clients_) {
    c->sendTextMessage("{ \"error\": \"" + error_val + "\" }");
  }
}

void RobotWebSocket::Send(const VisualizationMsg& local_vis,
                          const VisualizationMsg& global_vis,
                          const LaserScan& laser_scan,
                          const LaserScan& laser_lowbeam_scan,
                          const Localization2DMsg& localization,
                          const GPSArrayMsg& gps_goals_msg,
                          const GPSMsg& gps_pose) {
  QMutexLocker lock(&data_mutex_);
  local_vis_       = local_vis;
  global_vis_      = global_vis;
  laser_scan_      = laser_scan;
  laser_lowbeam_scan_ = laser_lowbeam_scan;
  localization_    = localization;
  gps_goals_msg_   = gps_goals_msg;
  gps_pose_        = gps_pose;

  // Emit the signal to send the data on the next event loop cycle
  emit SendDataSignal();
}
