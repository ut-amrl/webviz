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
 * \file    websocket.h
 * \brief   Lightweight interface to send data from a robot
 *          to a web-based visualization page.
 * \author  Joydeep Biswas, (C) 2020
 */
//========================================================================
#ifndef ECHOSERVER_H
#define ECHOSERVER_H

#include <stdint.h>

#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QMutex>
#include <QtCore/QObject>
#include <vector>

// --------------------
// Conditional Includes for ROS1 vs ROS2
// --------------------
#ifdef USE_ROS1
  #include "amrl_msgs/ColoredArc2D.h"
  #include "amrl_msgs/ColoredLine2D.h"
  #include "amrl_msgs/ColoredPoint2D.h"
  #include "amrl_msgs/ColoredText.h"
  #include "amrl_msgs/Localization2DMsg.h"
  #include "amrl_msgs/Point2D.h"
  #include "amrl_msgs/VisualizationMsg.h"
  #include "amrl_msgs/GPSArrayMsg.h"
  #include "amrl_msgs/GPSMsg.h"
  #include "sensor_msgs/LaserScan.h"
  #include "std_msgs/Float64MultiArray.h"

  // Type aliases for ROS1 messages
  using ColoredArc2D          = amrl_msgs::ColoredArc2D;
  using ColoredLine2D         = amrl_msgs::ColoredLine2D;
  using ColoredPoint2D        = amrl_msgs::ColoredPoint2D;
  using ColoredText           = amrl_msgs::ColoredText;
  using Localization2DMsg     = amrl_msgs::Localization2DMsg;
  using Point2D               = amrl_msgs::Point2D;
  using VisualizationMsg      = amrl_msgs::VisualizationMsg;
  using GPSArrayMsg           = amrl_msgs::GPSArrayMsg;
  using GPSMsg                = amrl_msgs::GPSMsg;
  using LaserScan             = sensor_msgs::LaserScan;
#else
  // Assume USE_ROS2
  #include "amrl_msgs/msg/colored_arc2_d.hpp"
  #include "amrl_msgs/msg/colored_line2_d.hpp"
  #include "amrl_msgs/msg/colored_point2_d.hpp"
  #include "amrl_msgs/msg/colored_text.hpp"
  #include "amrl_msgs/msg/localization2_d_msg.hpp"
  #include "amrl_msgs/msg/point2_d.hpp"
  #include "amrl_msgs/msg/visualization_msg.hpp"
  #include "amrl_msgs/msg/gps_array_msg.hpp"
  #include "amrl_msgs/msg/gps_msg.hpp"
  #include "sensor_msgs/msg/laser_scan.hpp"
  #include "std_msgs/msg/float64_multi_array.hpp"

  // Type aliases for ROS2 messages
  using ColoredArc2D          = amrl_msgs::msg::ColoredArc2D;
  using ColoredLine2D         = amrl_msgs::msg::ColoredLine2D;
  using ColoredPoint2D        = amrl_msgs::msg::ColoredPoint2D;
  using ColoredText           = amrl_msgs::msg::ColoredText;
  using Localization2DMsg     = amrl_msgs::msg::Localization2DMsg;
  using Point2D               = amrl_msgs::msg::Point2D;
  using VisualizationMsg      = amrl_msgs::msg::VisualizationMsg;
  using GPSArrayMsg           = amrl_msgs::msg::GPSArrayMsg;
  using GPSMsg                = amrl_msgs::msg::GPSMsg;
  using LaserScan             = sensor_msgs::msg::LaserScan;
#endif

// Forward declares for Qt classes
class QWebSocketServer;
class QWebSocket;

#define ROUTE_HEADER_SIZE (size_t) 1
#define MAX_ROUTE_NODES (size_t) 120
#define NODE_SIZE (size_t) 3
#define MAX_ROUTE_SIZE (ROUTE_HEADER_SIZE+MAX_ROUTE_NODES*NODE_SIZE)

struct MessageHeader {
  MessageHeader() : nonce(42) {}
  uint32_t nonce;                       // 1
  uint32_t num_points;                  // 2
  uint32_t num_lines;                   // 3
  uint32_t num_arcs;                    // 4
  uint32_t num_text_annotations;        // 5
  uint32_t num_laser_rays;              // 6
  uint32_t num_local_points;            // 7
  uint32_t num_local_lines;             // 8
  uint32_t num_local_arcs;              // 9
  uint32_t num_local_text_annotations;  // 10
  float laser_min_angle;                // 11
  float laser_max_angle;                // 12
  float loc_x;                          // 13
  float loc_y;                          // 14
  float loc_r;                          // 15
  float lat;                            // 16
  float lng;                            // 17
  float heading;                        // 18
  float route[MAX_ROUTE_SIZE];          // 19:19+MAX_ROUTE_SIZE
  char map[32];                         //
  size_t GetByteLength() const {
    // Just returns the total length in bytes for data serialization
    const size_t len =
        18 * 4 + MAX_ROUTE_SIZE * 4 + 32 +  // header fields + map data
        num_laser_rays * 4 +               // each ray is uint32_t
        num_points * 3 * 4 +               // x, y, color
        num_lines * 5 * 4 +                // x1, y1, x2, y2, color
        num_arcs * 6 * 4 +                 // x, y, radius, start_angle, end_angle, color
        num_text_annotations * 4 * 4 * 32; // x, y, color, size, msg
    return len;
  }
};

struct ColoredTextNative {
  Point2D start;
  uint32_t color;
  float size_em;
  char text[32];
};

// ---------------------------------------------------------------------------
// DataMessage is a native struct that merges all visualization data into
// one contiguous binary buffer we can send over WebSocket.
// ---------------------------------------------------------------------------
struct DataMessage {
  MessageHeader header;
  std::vector<uint32_t> laser_scan;
  std::vector<ColoredPoint2D> points;
  std::vector<ColoredLine2D> lines;
  std::vector<ColoredArc2D> arcs;
  std::vector<ColoredTextNative> text_annotations;

  QByteArray ToByteArray() const;

  // Create a DataMessage from the ROS messages (LaserScan, Visualization, etc.)
  static DataMessage FromRosMessages(
      const LaserScan& laser_msg,
      const LaserScan& laser_lowbeam_msg,
      const VisualizationMsg& local_msg,
      const VisualizationMsg& global_msg,
      const Localization2DMsg& localization_msg,
      const GPSArrayMsg& gps_route_msg,
      const GPSMsg& gps_pose_msg);
};

class RobotWebSocket : public QObject {
  Q_OBJECT
 public:
  explicit RobotWebSocket(uint16_t port);
  ~RobotWebSocket();

  // Send data out to all connected clients
  void Send(const VisualizationMsg& local_vis,
            const VisualizationMsg& global_vis,
            const LaserScan& laser_scan,
            const LaserScan& laser_lowbeam_scan,
            const Localization2DMsg& localization,
            const GPSArrayMsg& gps_goals_msg,
            const GPSMsg& gps_pose);

 Q_SIGNALS:
  // Signals that the socket can emit
  void closed();
  void SendDataSignal();
  // The following are signals corresponding to user commands, e.g. from JS
  void SetInitialPoseSignal(float x, float y, float theta, QString map);
  void SetNavGoalSignal(float x, float y, float theta, QString map);
  void SetGPSGoalSignal(float start_lat, float start_lon,
                        float end_lat, float end_lon);
  void ResetNavGoalsSignal();

 private Q_SLOTS:
  void onNewConnection();
  void processTextMessage(QString message);
  void processBinaryMessage(QByteArray message);
  void socketDisconnected();
  void SendDataSlot();

 private:
  void ProcessCallback(const QJsonObject& json);
  void SendError(const QString& error_val);

 private:
  QWebSocketServer* ws_server_;
  std::vector<QWebSocket*> clients_;

  QMutex data_mutex_;
  VisualizationMsg local_vis_;
  VisualizationMsg global_vis_;
  LaserScan laser_scan_;
  LaserScan laser_lowbeam_scan_;
  Localization2DMsg localization_;
  GPSArrayMsg gps_goals_msg_;
  GPSMsg gps_pose_;
};

#endif  // ECHOSERVER_H
