// Copyright 2025 miyajimad0620
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GEOMETRY_TRANSFORMER__ROS2__TWIST_TRANSFORMER_NODE_HPP_
#define GEOMETRY_TRANSFORMER__ROS2__TWIST_TRANSFORMER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "create_tf_listener.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visibility.hpp"

namespace geometry_transformer::ros2
{

class TwistTransformerNode : public rclcpp::Node
{
  using TwistMsg = geometry_msgs::msg::TwistStamped;
  using TwistRawMsg = geometry_msgs::msg::Twist;

public:
  static constexpr auto kDefaultNodeName = "twist_transformer";

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  inline TwistTransformerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", std::string("map"))),
    only_rotation_(this->declare_parameter<bool>("only_rotation", false)),
    tf_buffer_(this->get_clock()),
    tf_listener_(create_tf_listener(this, tf_buffer_)),
    twist_transformed_publisher_(this->create_twist_transformed_publisher()),
    twist_raw_transformed_publisher_(
      this->declare_parameter("publish_raw",
      false) ? this->create_twist_raw_transformed_publisher() :
      nullptr),
    twist_subscription_(this->create_twist_subscription())
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline TwistTransformerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : TwistTransformerNode(node_name, "", options)
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline TwistTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : TwistTransformerNode(kDefaultNodeName, "", options)
  {
  }

  ~TwistTransformerNode() override {}

private:
  rclcpp::Publisher<TwistMsg>::SharedPtr create_twist_transformed_publisher()
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<TwistMsg>(
      "twist/transformed", rclcpp::QoS(10), publisher_options);
  }

  rclcpp::Publisher<TwistRawMsg>::SharedPtr create_twist_raw_transformed_publisher()
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<TwistRawMsg>(
      "twist_raw/transformed", rclcpp::QoS(10), publisher_options);
  }

  rclcpp::Subscription<TwistMsg>::SharedPtr create_twist_subscription()
  {
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<TwistMsg>(
      "twist", rclcpp::QoS(10),
      [this](TwistMsg::ConstSharedPtr msg) {this->grant(std::move(msg));}, subscription_options);
  }

  void grant(TwistMsg::ConstSharedPtr twist_msg)
  {
    geometry_msgs::msg::TransformStamped twist_base_tf_msg;
    try {
      twist_base_tf_msg =
        tf_buffer_.lookupTransform(frame_id_, twist_msg->header.frame_id, twist_msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }

    // convert to tf2::Transform
    tf2::Transform twist_base_tf;
    tf2::fromMsg(twist_base_tf_msg.transform, twist_base_tf);
    tf2::Vector3 twist_relative_linear_vec;
    tf2::fromMsg(twist_msg->twist.linear, twist_relative_linear_vec);
    tf2::Vector3 twist_relative_angular_vec;
    tf2::fromMsg(twist_msg->twist.angular, twist_relative_angular_vec);

    // calculate the new twist in the target frame
    if (!only_rotation_) {
      twist_relative_linear_vec +=
        twist_relative_angular_vec.cross(twist_base_tf.inverse().getOrigin());
    }
    const auto twist_base_rotation_mat = tf2::Matrix3x3(twist_base_tf.getRotation());
    const auto twist_linear_vec = twist_base_rotation_mat * twist_relative_linear_vec;
    const auto twist_angular_vec = twist_base_rotation_mat * twist_relative_angular_vec;

    // create a new TwistMsg with the twist in the target frame
    auto twist_transformed_msg = std::make_unique<TwistMsg>();
    twist_transformed_msg->header.stamp = twist_msg->header.stamp;
    twist_transformed_msg->header.frame_id = frame_id_;
    twist_transformed_msg->twist.linear = tf2::toMsg(twist_linear_vec);
    twist_transformed_msg->twist.angular = tf2::toMsg(twist_angular_vec);

    // publish the twist in the target frame
    twist_transformed_publisher_->publish(std::move(twist_transformed_msg));

    // if raw twist is requested, publish the raw transformed twist
    if (twist_raw_transformed_publisher_) {
      // create a new TwistRawMsg with the raw twist in the target frame
      auto twist_raw_transformed_msg = std::make_unique<TwistRawMsg>();
      twist_raw_transformed_msg->linear = tf2::toMsg(twist_relative_linear_vec);
      twist_raw_transformed_msg->angular = tf2::toMsg(twist_relative_angular_vec);

      // publish the raw twist in the target frame
      twist_raw_transformed_publisher_->publish(std::move(twist_raw_transformed_msg));
    }
  }

  // parameter
  std::string frame_id_;
  bool only_rotation_;

  // tf buffer & listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // transformed publisher
  rclcpp::Publisher<TwistMsg>::SharedPtr twist_transformed_publisher_;
  rclcpp::Publisher<TwistRawMsg>::SharedPtr twist_raw_transformed_publisher_;

  // original subscription
  rclcpp::Subscription<TwistMsg>::SharedPtr twist_subscription_;
};

}  // namespace geometry_transformer::ros2

#endif  // GEOMETRY_TRANSFORMER__ROS2__TWIST_TRANSFORMER_NODE_HPP_
