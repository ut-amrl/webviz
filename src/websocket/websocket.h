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
#include <QtCore/QMutex>
#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>
#include <QtCore/QString>
#include <deque>
#include <string>
#include <vector>

#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/point2_d.hpp"
#include "amrl_msgs/msg/colored_point2_d.hpp"
#include "amrl_msgs/msg/colored_line2_d.hpp"
#include "amrl_msgs/msg/colored_arc2_d.hpp"
#include "amrl_msgs/msg/colored_text.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using amrl_msgs::msg::ColoredArc2D;
using amrl_msgs::msg::ColoredLine2D;
using amrl_msgs::msg::ColoredPoint2D;
using amrl_msgs::msg::ColoredText;
using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::Point2D;
using amrl_msgs::msg::VisualizationMsg;
using sensor_msgs::msg::LaserScan;

class QWebSocketServer;
class QWebSocket;

struct MessageHeader
{
    MessageHeader();                     // Move implementation to .cc file
    uint32_t nonce;                      // 1
    uint32_t num_points;                 // 2
    uint32_t num_lines;                  // 3
    uint32_t num_arcs;                   // 4
    uint32_t num_text_annotations;       // 5
    uint32_t num_laser_rays;             // 6
    uint32_t num_local_points;           // 7
    uint32_t num_local_lines;            // 8
    uint32_t num_local_arcs;             // 9
    uint32_t num_local_text_annotations; // 10
    float laser_min_angle;               // 11
    float laser_max_angle;               // 12
    float loc_x;                         // 13
    float loc_y;                         // 14
    float loc_r;                         // 15
    char map[32];                        //
    size_t GetByteLength() const
    {
        const size_t len = 15 * 4 + 32 +                      // header fields + map data
                           num_laser_rays * 4 +               // each ray is uint32_t
                           num_points * 3 * 4 +               // x, y, color
                           num_lines * 5 * 4 +                // x1, y1, x2, y2, color
                           num_arcs * 6 * 4 +                 // x, y, radius, start_angle, end_angle, color
                           num_text_annotations * 4 * 4 * 32; // x, y, color, size, msg
        return len;
    }
};

struct ColoredTextNative
{
    Point2D start;
    uint32_t color;
    float size_em;
    char text[32];
};

// Binary frame sent to the browser for each image panel update. The browser
// dispatches on the leading nonce: kVisNonce (42) -> existing DataMessage,
// kImageNonce (43) -> ImageFrame.
struct ImageFrame
{
    uint32_t panel_id; // 0 = left, 1 = right (extensible)
    std::string topic; // source ROS topic name
    double stamp_sec;  // ROS timestamp in seconds
    QByteArray jpeg;   // raw JPEG bytes (browser-decodable)
    QByteArray ToByteArray() const;
};

struct DataMessage
{
    MessageHeader header;
    std::vector<uint32_t> laser_scan;
    std::vector<ColoredPoint2D> points;
    std::vector<ColoredLine2D> lines;
    std::vector<ColoredArc2D> arcs;
    std::vector<ColoredTextNative> text_annotations;
    QByteArray ToByteArray() const;
    static DataMessage FromRosMessages(
        const LaserScan &laser_msg,
        const VisualizationMsg &local_msg,
        const VisualizationMsg &global_msg,
        const Localization2DMsg &localization_msg);
};

class RobotWebSocket : public QObject
{
    Q_OBJECT
public:
    explicit RobotWebSocket(uint16_t port);
    ~RobotWebSocket();
    void Send(const VisualizationMsg &local_vis,
              const VisualizationMsg &global_vis,
              const LaserScan &laser_scan,
              const Localization2DMsg &localization);
    // Enqueue an image frame for transmission to all connected clients.
    // Safe to call from any thread.
    void SendImage(uint32_t panel_id,
                   const std::string &topic,
                   const QByteArray &jpeg,
                   double stamp_sec);
    // Send a foresight planner status update as a JSON text frame to all
    // connected clients. ``payload_json`` is a pre-serialized JSON document
    // whose root must be an object containing at minimum a ``type`` field
    // (e.g. "foresight_response"). Building the JSON in the caller keeps the
    // signal/slot signature small as the payload schema grows (state,
    // verdict, reason, reflection_id, thinking_text, motion_text,
    // critic_text, motion_image_b64, ...).
    void SendForesightStatus(const QString &payload_json);

Q_SIGNALS:
    void closed();
    void SendDataSignal();
    void SendImageSignal();
    void SendForesightStatusSignal(QString payload_json);
    void SetInitialPoseSignal(float x, float y, float theta, QString map);
    void SetNavGoalSignal(float x, float y, float theta, QString map);
    void ResetNavGoalsSignal();
    void ForesightCommandSignal(QString text);

private Q_SLOTS:
    void onNewConnection();
    void processTextMessage(QString message);
    void processBinaryMessage(QByteArray message);
    void socketDisconnected();
    void SendDataSlot();
    void SendImageSlot();
    void SendForesightStatusSlot(QString payload_json);

private:
    void ProcessCallback(const QJsonObject &json);
    void SendError(const QString &error_val);

private:
    QWebSocketServer *ws_server_;
    std::vector<QWebSocket *> clients_;

    QMutex data_mutex_;
    VisualizationMsg local_vis_;
    VisualizationMsg global_vis_;
    LaserScan laser_scan_;
    Localization2DMsg localization_;

    QMutex image_mutex_;
    std::deque<ImageFrame> image_queue_;
};

#endif // ECHOSERVER_H
