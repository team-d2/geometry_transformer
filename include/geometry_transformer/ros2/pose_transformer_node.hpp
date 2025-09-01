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

#ifndef GEOMETRY_TRANSFORMER__ROS2__POSE_TRANSFORMER_NODE_HPP_
#define GEOMETRY_TRANSFORMER__ROS2__POSE_TRANSFORMER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "create_tf_listener.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visibility.hpp"

namespace geometry_transformer::ros2
{

class PoseTransformerNode : public rclcpp::Node
{
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using PoseRawMsg = geometry_msgs::msg::Pose;

public:
  static constexpr auto kDefaultNodeName = "pose_transformer";

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  inline PoseTransformerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", std::string("map"))),
    tf_buffer_(this->get_clock()),
    tf_listener_(create_tf_listener(this, tf_buffer_)),
    pose_transformed_publisher_(this->create_pose_transformed_publisher()),
    pose_subscription_(this->create_pose_subscription())
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PoseTransformerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PoseTransformerNode(node_name, "", options)
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PoseTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PoseTransformerNode(kDefaultNodeName, "", options)
  {
  }

  ~PoseTransformerNode() override {}

private:
  rclcpp::Publisher<PoseMsg>::SharedPtr create_pose_transformed_publisher()
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<PoseMsg>("pose/transformed", rclcpp::QoS(10), publisher_options);
  }

  rclcpp::Subscription<PoseMsg>::SharedPtr create_pose_subscription()
  {
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<PoseMsg>(
      "pose", rclcpp::QoS(10), [this](PoseMsg::ConstSharedPtr msg) {this->transform(std::move(msg));},
      subscription_options);
  }

  void transform(PoseMsg::ConstSharedPtr pose_msg)
  {
    geometry_msgs::msg::TransformStamped pose_base_tf_msg;
    try {
      pose_base_tf_msg =
        tf_buffer_.lookupTransform(frame_id_, pose_msg->header.frame_id, pose_msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }

    // convert to tf2::Transform
    tf2::Transform pose_base_tf;
    tf2::fromMsg(pose_base_tf_msg.transform, pose_base_tf);
    tf2::Transform pose_relative_tf;
    tf2::fromMsg(pose_msg->pose, pose_relative_tf);

    // calculate the new pose in the target frame
    const auto pose_tf = pose_base_tf * pose_relative_tf;

    // create a new PoseMsg with the pose in the target frame
    auto pose_transformed_msg = std::make_unique<PoseMsg>();
    pose_transformed_msg->header.stamp = pose_msg->header.stamp;
    pose_transformed_msg->header.frame_id = frame_id_;
    tf2::toMsg(pose_tf, pose_transformed_msg->pose);

    // publish the pose in the target frame
    pose_transformed_publisher_->publish(std::move(pose_transformed_msg));
  }

  // parameter
  std::string frame_id_;

  // tf buffer & listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // transformed publisher
  rclcpp::Publisher<PoseMsg>::SharedPtr pose_transformed_publisher_;

  // original subscription
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_subscription_;
};

}  // namespace geometry_transformer::ros2

#endif  // GEOMETRY_TRANSFORMER__ROS2__POSE_TRANSFORMER_NODE_HPP_
