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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visibility.hpp"

namespace geometry_transformer::ros2
{

class PoseWithCovarianceTransformerNode : public rclcpp::Node
{
  using PoseWithCovarianceMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PoseRawMsg = geometry_msgs::msg::Pose;

public:
  static constexpr auto kDefaultNodeName = "pose_with_covariance_transformer";

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  inline PoseWithCovarianceTransformerNode(
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
  explicit inline PoseWithCovarianceTransformerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PoseWithCovarianceTransformerNode(node_name, "", options)
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PoseWithCovarianceTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PoseWithCovarianceTransformerNode(kDefaultNodeName, "", options)
  {
  }

  ~PoseWithCovarianceTransformerNode() override {}

private:
  rclcpp::Publisher<PoseWithCovarianceMsg>::SharedPtr create_pose_transformed_publisher()
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<PoseWithCovarianceMsg>("pose/with_cov/transformed", rclcpp::QoS(10), publisher_options);
  }

  rclcpp::Subscription<PoseWithCovarianceMsg>::SharedPtr create_pose_subscription()
  {
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<PoseWithCovarianceMsg>(
      "pose/with_cov", rclcpp::QoS(10), [this](PoseWithCovarianceMsg::ConstSharedPtr msg) {this->transform(std::move(msg));},
      subscription_options);
  }

  void transform(PoseWithCovarianceMsg::ConstSharedPtr pose_msg)
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
    tf2::fromMsg(pose_msg->pose.pose, pose_relative_tf);
    const tf2::Matrix3x3 cov00_relative_mat = tf2::Matrix3x3(
      pose_msg->pose.covariance[0], pose_msg->pose.covariance[1], pose_msg->pose.covariance[2],
      pose_msg->pose.covariance[6], pose_msg->pose.covariance[7], pose_msg->pose.covariance[8],
      pose_msg->pose.covariance[12], pose_msg->pose.covariance[13], pose_msg->pose.covariance[14]);
    const tf2::Matrix3x3 cov01_relative_mat = tf2::Matrix3x3(
      pose_msg->pose.covariance[3], pose_msg->pose.covariance[4], pose_msg->pose.covariance[5],
      pose_msg->pose.covariance[9], pose_msg->pose.covariance[10], pose_msg->pose.covariance[11],
      pose_msg->pose.covariance[15], pose_msg->pose.covariance[16], pose_msg->pose.covariance[17]);
    const tf2::Matrix3x3 cov10_relative_mat = tf2::Matrix3x3(
      pose_msg->pose.covariance[18], pose_msg->pose.covariance[19], pose_msg->pose.covariance[20],
      pose_msg->pose.covariance[24], pose_msg->pose.covariance[25], pose_msg->pose.covariance[26],
      pose_msg->pose.covariance[30], pose_msg->pose.covariance[31], pose_msg->pose.covariance[32]);
    const tf2::Matrix3x3 cov11_relative_mat = tf2::Matrix3x3(
      pose_msg->pose.covariance[21], pose_msg->pose.covariance[22], pose_msg->pose.covariance[23],
      pose_msg->pose.covariance[27], pose_msg->pose.covariance[28], pose_msg->pose.covariance[29],
      pose_msg->pose.covariance[33], pose_msg->pose.covariance[34], pose_msg->pose.covariance[35]);

    // calculate the new pose in the target frame
    const auto pose_tf = pose_base_tf * pose_relative_tf;
    tf2::Matrix3x3 rot = pose_base_tf.getBasis();
    const auto cov00_mat = rot * cov00_relative_mat * rot.transpose();
    const auto cov01_mat = rot * cov01_relative_mat * rot.transpose();
    const auto cov10_mat = rot * cov10_relative_mat * rot.transpose();
    const auto cov11_mat = rot * cov11_relative_mat * rot.transpose();

    // create a new PoseWithCovarianceMsg with the pose in the target frame
    auto pose_transformed_msg = std::make_unique<PoseWithCovarianceMsg>();
    pose_transformed_msg->header.stamp = pose_msg->header.stamp;
    pose_transformed_msg->header.frame_id = frame_id_;
    tf2::toMsg(pose_tf, pose_transformed_msg->pose.pose);
    pose_transformed_msg->pose.covariance = {
      cov00_mat[0][0], cov00_mat[0][1], cov00_mat[0][2], cov01_mat[0][0], cov01_mat[0][1], cov01_mat[0][2],
      cov00_mat[1][0], cov00_mat[1][1], cov00_mat[1][2], cov01_mat[1][0], cov01_mat[1][1], cov01_mat[1][2],
      cov00_mat[2][0], cov00_mat[2][1], cov00_mat[2][2], cov01_mat[2][0], cov01_mat[2][1], cov01_mat[2][2],
      cov10_mat[0][0], cov10_mat[0][1], cov10_mat[0][2], cov11_mat[0][0], cov11_mat[0][1], cov11_mat[0][2],
      cov10_mat[1][0], cov10_mat[1][1], cov10_mat[1][2], cov11_mat[1][0], cov11_mat[1][1], cov11_mat[1][2],
      cov10_mat[2][0], cov10_mat[2][1], cov10_mat[2][2], cov11_mat[2][0], cov11_mat[2][1], cov11_mat[2][2],
    };

    // publish the pose in the target frame
    pose_transformed_publisher_->publish(std::move(pose_transformed_msg));
  }

  // parameter
  std::string frame_id_;

  // tf buffer & listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // transformed publisher
  rclcpp::Publisher<PoseWithCovarianceMsg>::SharedPtr pose_transformed_publisher_;

  // original subscription
  rclcpp::Subscription<PoseWithCovarianceMsg>::SharedPtr pose_subscription_;
};

}  // namespace geometry_transformer::ros2

#endif  // GEOMETRY_TRANSFORMER__ROS2__POSE_TRANSFORMER_NODE_HPP_
