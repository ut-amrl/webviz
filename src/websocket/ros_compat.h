#pragma once

// Webviz-specific ROS1/ROS2 compatibility layer
// This provides comprehensive abstractions for cross-ROS compatibility

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <memory>

// Webviz-specific logger
#define WEBVIZ_LOGGER rclcpp::get_logger("webviz")

#define LOG_ERROR(...) RCLCPP_ERROR(WEBVIZ_LOGGER, __VA_ARGS__)
#define LOG_WARN(...) RCLCPP_WARN(WEBVIZ_LOGGER, __VA_ARGS__)
#define LOG_INFO(...) RCLCPP_INFO(WEBVIZ_LOGGER, __VA_ARGS__)
#define GET_TIME() rclcpp::Clock().now()
#define DURATION(secs) rclcpp::Duration::from_seconds(secs)

// Node abstractions
using NodePtr = std::shared_ptr<rclcpp::Node>;

// Publisher/Subscriber type abstractions
template <typename MessageType>
using PublisherPtr = typename rclcpp::Publisher<MessageType>::SharedPtr;

template <typename MessageType>
using SubscriberPtr = typename rclcpp::Subscription<MessageType>::SharedPtr;

// Time abstractions
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;

// ROS system macros
#define ROS_OK() rclcpp::ok()
#define ROS_SPIN_ONCE(node) rclcpp::spin_some(node)
#define ROS_INIT(argc, argv, name) rclcpp::init(argc, argv)
#define ROS_SHUTDOWN() rclcpp::shutdown()

// Create zero time
#define ZERO_TIME() rclcpp::Time(0)

// Node creation macro
#define CREATE_NODE(name) rclcpp::Node::make_shared(name)

// Publisher creation macro
#define CREATE_PUBLISHER(node, msg_type, topic, queue_size) \
    node->create_publisher<msg_type>(topic, queue_size)

// Subscriber creation macro
#define CREATE_SUBSCRIBER(node, msg_type, topic, queue_size, callback) \
    node->create_subscription<msg_type>(topic, queue_size, callback)

// Publish macro
#define PUBLISH(publisher, msg) publisher->publish(msg)

#else
#include <ros/ros.h>

#define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define GET_TIME() ros::Time::now()
#define DURATION(secs) ros::Duration(secs)

// Node abstractions
using NodePtr = std::shared_ptr<ros::NodeHandle>;

// Publisher/Subscriber type abstractions
template <typename MessageType>
using PublisherPtr = ros::Publisher;

template <typename MessageType>
using SubscriberPtr = ros::Subscriber;

// Time abstractions
using Time = ros::Time;
using Duration = ros::Duration;

// ROS system macros
#define ROS_OK() ros::ok()
#define ROS_SPIN_ONCE(node) ros::spinOnce()
#define ROS_INIT(argc, argv, name) ros::init(argc, argv, name, ros::init_options::NoSigintHandler)
#define ROS_SHUTDOWN() ros::shutdown()

// Create zero time
#define ZERO_TIME() ros::Time(0)

// Node creation macro
#define CREATE_NODE(name) std::make_shared<ros::NodeHandle>()

// Publisher creation macro
#define CREATE_PUBLISHER(node, msg_type, topic, queue_size) \
    node->advertise<msg_type>(topic, queue_size)

// Subscriber creation macro
#define CREATE_SUBSCRIBER(node, msg_type, topic, queue_size, callback) \
    node->subscribe(topic, queue_size, callback)

// Publish macro
#define PUBLISH(publisher, msg) publisher.publish(msg)

#endif