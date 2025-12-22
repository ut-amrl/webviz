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

#include <string.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "config_reader/config_reader.h"

#include <QtWebSockets/qwebsocketserver.h>
#include <QtWebSockets/qwebsocket.h>
#include <QtCore/QDebug>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QMetaType>

#ifdef ROS2
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/point2_d.hpp"
#include "amrl_msgs/msg/colored_point2_d.hpp"
#include "amrl_msgs/msg/colored_line2_d.hpp"
#include "amrl_msgs/msg/colored_arc2_d.hpp"
#include "amrl_msgs/msg/colored_text.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
using amrl_msgs::msg::ColoredArc2D;
using amrl_msgs::msg::ColoredLine2D;
using amrl_msgs::msg::ColoredPoint2D;
using amrl_msgs::msg::ColoredText;
using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::Point2D;
using amrl_msgs::msg::VisualizationMsg;
#else
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/Point2D.h"
#include "amrl_msgs/ColoredPoint2D.h"
#include "amrl_msgs/ColoredLine2D.h"
#include "amrl_msgs/ColoredArc2D.h"
#include "amrl_msgs/ColoredText.h"
#include "amrl_msgs/VisualizationMsg.h"
using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::ColoredText;
using amrl_msgs::Localization2DMsg;
using amrl_msgs::Point2D;
using amrl_msgs::VisualizationMsg;
#endif

#include "ros_compat.h"

using std::vector;

DECLARE_int32(v);

// Configuration variables used in websocket.cc
CONFIG_INT(protocol_nonce, "data_processing.protocol_nonce");
CONFIG_INT(map_name_buffer_size, "data_processing.map_name_buffer_size");
CONFIG_INT(text_buffer_size, "data_processing.text_buffer_size");

MessageHeader::MessageHeader() {
    nonce = CONFIG_protocol_nonce;
}

RobotWebSocket::RobotWebSocket(uint16_t port) : ws_server_(new QWebSocketServer(QStringLiteral("WebViz"),
                                                                                QWebSocketServer::NonSecureMode)),
                                                local_vis_(),
                                                global_vis_(),
                                                localization_() {
    qRegisterMetaType<uint8_t>("uint8_t");
    if (ws_server_->listen(QHostAddress::Any, port)) {
        printf("WebViz listening on port %d\n", port);
        connect(ws_server_, &QWebSocketServer::newConnection,
                this, &RobotWebSocket::onNewConnection);
    }
    // Initialize localization time stamp to zero.
    localization_.header.stamp = ZERO_TIME();
}

RobotWebSocket::~RobotWebSocket() {
    ws_server_->close();
    qDeleteAll(clients_.begin(), clients_.end());
}

void RobotWebSocket::onNewConnection() {
    auto* socket = ws_server_->nextPendingConnection();
    if (FLAGS_v > 0) {
        printf("WebSocket client connected: %s\n",
               socket->peerAddress().toString().toStdString().c_str());
    }

    connect(socket, &QWebSocket::textMessageReceived,
            this, &RobotWebSocket::processTextMessage);
    connect(socket, &QWebSocket::binaryMessageReceived,
            this, &RobotWebSocket::processBinaryMessage);
    connect(socket, &QWebSocket::disconnected,
            this, &RobotWebSocket::socketDisconnected);
    connect(this, &RobotWebSocket::SendDataSignal,
            this, &RobotWebSocket::SendDataSlot);
    connect(this, &RobotWebSocket::NavStatusSignal,
            this, &RobotWebSocket::NavStatusSlot);

    clients_.push_back(socket);
}

void RobotWebSocket::processTextMessage(QString message) {
    if (FLAGS_v > 1) {
        printf("WebSocket message received: %s\n", message.toStdString().c_str());
    }
    QJsonParseError error;
    QJsonDocument json_doc = QJsonDocument::fromJson(message.toUtf8(), &error);
    if (error.error == QJsonParseError::NoError) {
        QJsonObject json_obj = json_doc.object();
        ProcessCallback(json_obj);
    } else {
        printf("ERROR: Ignoring websocket message, JSON parsing error: %s\n",
               error.errorString().toStdString().c_str());
    }
}

void RobotWebSocket::processBinaryMessage(QByteArray message) {
    if (FLAGS_v > 1) {
        printf("WebSocket Binary message received: %s\n", message.data());
    }
}

void RobotWebSocket::socketDisconnected() {
    auto* client = qobject_cast<QWebSocket*>(sender());
    if (FLAGS_v > 0) {
        printf("WebSocket client disconnected: %s\n",
               client->peerAddress().toString().toStdString().c_str());
    }
    if (client) {
        clients_.erase(std::find(clients_.begin(), clients_.end(), client));
        client->deleteLater();
    }
}

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

QByteArray DataMessage::ToByteArray() const {
    QByteArray data;
    data.resize(header.GetByteLength());
    char* buf = data.data();
    buf = WriteElement(header, buf);
    buf = WriteElementVector(points, buf);
    buf = WriteElementVector(lines, buf);
    buf = WriteElementVector(arcs, buf);
    buf = WriteElementVector(text_annotations, buf);
    return data;
}

DataMessage DataMessage::FromRosMessages(
    const VisualizationMsg& local_msg,
    const VisualizationMsg& global_msg,
    const Localization2DMsg& localization_msg) {
    static const bool kDebug = false;
    DataMessage msg;
    for (size_t i = 0; i < sizeof(msg.header.map); ++i) {
        msg.header.map[i] = 0;
    }
    msg.header.loc_x = localization_msg.pose.x;
    msg.header.loc_y = localization_msg.pose.y;
    msg.header.loc_r = localization_msg.pose.theta;
    strncpy(msg.header.map,
            localization_msg.map.data(),
            std::min(CONFIG_map_name_buffer_size - 1, static_cast<int>(localization_msg.map.size())));
    msg.points = local_msg.points;
    msg.header.num_local_points = local_msg.points.size();
    msg.points.insert(msg.points.end(),
                      global_msg.points.begin(),
                      global_msg.points.end());

    msg.lines = local_msg.lines;
    msg.header.num_local_lines = local_msg.lines.size();
    msg.lines.insert(msg.lines.end(),
                     global_msg.lines.begin(),
                     global_msg.lines.end());

    msg.arcs = local_msg.arcs;
    msg.header.num_local_arcs = local_msg.arcs.size();
    msg.arcs.insert(msg.arcs.end(),
                    global_msg.arcs.begin(),
                    global_msg.arcs.end());

    msg.header.num_points = msg.points.size();
    msg.header.num_lines = msg.lines.size();
    msg.header.num_arcs = msg.arcs.size();
    msg.header.num_local_text_annotations = local_msg.text_annotations.size();
    msg.header.num_text_annotations = local_msg.text_annotations.size() + global_msg.text_annotations.size();
    for (ColoredText text : local_msg.text_annotations) {
        ColoredTextNative localText;
        localText.start = text.start;
        localText.color = text.color;
        localText.size_em = text.size_em;
        size_t size = std::min(CONFIG_text_buffer_size - 1, static_cast<int>(text.text.size()));
        strncpy(localText.text, text.text.data(), size);
        localText.text[size] = 0;
        msg.text_annotations.push_back(localText);
    }
    for (ColoredText text : global_msg.text_annotations) {
        ColoredTextNative localText;
        localText.start = text.start;
        localText.color = text.color;
        localText.size_em = text.size_em;
        size_t size = std::min(CONFIG_text_buffer_size - 1, static_cast<int>(text.text.size()));
        strncpy(localText.text, text.text.data(), size);
        localText.text[size] = 0;
        msg.text_annotations.push_back(localText);
    }

    if (kDebug) {
        printf(
            "nonce: %d "
            "num_points: %d "
            "num_lines: %d "
            "num_arcs: %d "
            "num_text_annotations: %d "
            "num_local_points: %d "
            "num_local_lines: %d "
            "num_local_arcs: %d "
            "num_local_text_annotations: %d\n",
            msg.header.nonce,
            msg.header.num_points,
            msg.header.num_lines,
            msg.header.num_arcs,
            msg.header.num_text_annotations,
            msg.header.num_local_points,
            msg.header.num_local_lines,
            msg.header.num_local_arcs,
            msg.header.num_local_text_annotations);
    }
    return msg;
}

void RobotWebSocket::SendError(const QString& error_val) {
    for (auto c : clients_) {
        CHECK_NOTNULL(c);
        c->sendTextMessage("{ \"error\": \"" + error_val + "\" }");
    }
}

void RobotWebSocket::SendNavStatus(uint8_t status) {
    QString json = QString("{ \"type\": \"nav_status\", \"status\": %1 }").arg(status);
    for (auto c : clients_) {
        CHECK_NOTNULL(c);
        c->sendTextMessage(json);
    }
}

void RobotWebSocket::NavStatusSlot(uint8_t status) {
    SendNavStatus(status);
}

bool AllNumericalKeysPresent(const QStringList& expected,
                             const QJsonObject& json) {
    for (const QString& key : expected) {
        if (!json.contains(key)) return false;
        const QJsonValue val = json.value(key);
        if (!val.isDouble()) return false;
    }
    return true;
}

bool StringKeyPresent(const QString& key,
                      const QJsonObject& json) {
    if (!json.contains(key)) return false;
    const QJsonValue val = json.value(key);
    return val.isString();
}

void RobotWebSocket::ProcessCallback(const QJsonObject& json) {
    static const bool kDebug = false;
    if (kDebug) {
        qInfo() << "Callback JSON:\n"
                << json;
    }
    if (!json.contains("type")) {
        SendError("Malformed request");
        return;
    }
    const auto type = json.value("type");
    if (type == "set_initial_pose") {
        if (!AllNumericalKeysPresent({"x", "y", "theta"}, json) ||
            !StringKeyPresent("map", json)) {
            SendError("Invalid set_initial_pose parameters");
            return;
        }
        SetInitialPoseSignal(json.value("x").toDouble(),
                             json.value("y").toDouble(),
                             json.value("theta").toDouble(),
                             json.value("map").toString());
    } else if (type == "set_nav_goal") {
        if (!AllNumericalKeysPresent({"x", "y", "theta"}, json) ||
            !StringKeyPresent("map", json)) {
            SendError("Invalid set_nav_goal parameters");
            return;
        }
        SetNavGoalSignal(json.value("x").toDouble(),
                         json.value("y").toDouble(),
                         json.value("theta").toDouble(),
                         json.value("map").toString());
    } else if (type == "change_map") {
        if (!StringKeyPresent("map", json)) {
            SendError("Invalid change_map parameters");
            return;
        }
        ChangeMapSignal(json.value("map").toString());
    } else if (type == "reset_nav_goals") {
        ResetNavGoalsSignal();
    } else {
        SendError("Unrecognized request type");
    }
}

void RobotWebSocket::SendDataSlot() {
    if (clients_.empty()) return;
    data_mutex_.lock();
    const auto data = DataMessage::FromRosMessages(
        local_vis_, global_vis_, localization_);
    const auto buffer = data.ToByteArray();
    CHECK_EQ(data.header.GetByteLength(), buffer.size());
    for (auto c : clients_) {
        c->sendBinaryMessage(buffer);
    }
    data_mutex_.unlock();
}

void RobotWebSocket::Send(const VisualizationMsg& local_vis,
                          const VisualizationMsg& global_vis,
                          const Localization2DMsg& localization) {
    data_mutex_.lock();
    localization_ = localization;
    local_vis_ = local_vis;
    global_vis_ = global_vis;
    data_mutex_.unlock();
    SendDataSignal();
}
