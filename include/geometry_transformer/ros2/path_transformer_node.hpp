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

#ifndef GEOMETRY_TRANSFORMER__ROS2__PATH_TRANSFORMER_NODE_HPP_
#define GEOMETRY_TRANSFORMER__ROS2__PATH_TRANSFORMER_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "create_tf_listener.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visibility.hpp"

namespace geometry_transformer::ros2
{

class PathTransformerNode : public rclcpp::Node
{
  using PathMsg = nav_msgs::msg::Path;
  using PoseMsg = geometry_msgs::msg::PoseStamped;

public:
  static constexpr auto kDefaultNodeName = "path_transformer";

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  inline PathTransformerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", std::string("map"))),
    tf_buffer_(this->get_clock()),
    tf_listener_(create_tf_listener(this, tf_buffer_)),
    path_frame_changed_publisher_(this->create_path_frame_changed_publisher()),
    path_subscription_(this->create_path_subscription())
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PathTransformerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PathTransformerNode(node_name, "", options)
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PathTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PathTransformerNode(kDefaultNodeName, "", options)
  {
  }

  ~PathTransformerNode() override {}

private:
  rclcpp::Publisher<PathMsg>::SharedPtr create_path_frame_changed_publisher()
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<PathMsg>("path/transformed", rclcpp::QoS(10), publisher_options);
  }

  rclcpp::Subscription<PathMsg>::SharedPtr create_path_subscription()
  {
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<PathMsg>(
      "path", rclcpp::QoS(10), [this](PathMsg::ConstSharedPtr msg) {this->grant(std::move(msg));},
      subscription_options);
  }

  void grant(PathMsg::ConstSharedPtr path_msg)
  {
    // get header tf
    std::map<std::string, tf2::Transform> path_base_tf_map;
    if (!path_msg->header.frame_id.empty()) {
      geometry_msgs::msg::TransformStamped path_base_tf_msg;
      try {
        path_base_tf_msg =
          tf_buffer_.lookupTransform(frame_id_, path_msg->header.frame_id, path_msg->header.stamp);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to get transform: %s", ex.what());
        return;
      }

      tf2::Transform path_base_tf;
      tf2::fromMsg(path_base_tf_msg.transform, path_base_tf);
      path_base_tf_map[path_msg->header.frame_id] = path_base_tf_map[""] = path_base_tf;
    }

    // get pose tf
    for (const auto & pose_msg_data : path_msg->poses) {
      if (path_base_tf_map.count(pose_msg_data.header.frame_id) > 0) {
        continue;
      }

      geometry_msgs::msg::TransformStamped path_base_tf_msg;
      try {
        path_base_tf_msg =
          tf_buffer_.lookupTransform(frame_id_, path_msg->header.frame_id, path_msg->header.stamp);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to get transform: %s", ex.what());
        return;
      }

      tf2::Transform path_base_tf;
      tf2::fromMsg(path_base_tf_msg.transform, path_base_tf);
      path_base_tf_map[pose_msg_data.header.frame_id] = path_base_tf;
    }

    // create a new PathMsg with the path in the target frame
    auto path_transformed_msg = std::make_unique<PathMsg>();
    path_transformed_msg->header.stamp = path_msg->header.stamp;
    path_transformed_msg->header.frame_id = frame_id_;
    for (const auto & pose_msg_data : path_msg->poses) {
      const auto & pose_base_tf = path_base_tf_map[pose_msg_data.header.frame_id];
      tf2::Transform pose_relative_tf;
      tf2::fromMsg(pose_msg_data.pose, pose_relative_tf);
      const auto pose_tf = pose_base_tf * pose_relative_tf;
      PoseMsg pose_transformed_msg_data;
      pose_transformed_msg_data.header.stamp = pose_msg_data.header.stamp;
      tf2::toMsg(pose_tf, pose_transformed_msg_data.pose);
      path_transformed_msg->poses.emplace_back(pose_transformed_msg_data);
    }

    // publish the path in the target frame
    path_frame_changed_publisher_->publish(std::move(path_transformed_msg));
  }

  // parameter
  std::string frame_id_;

  // tf buffer & listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // donkey_fix publisher
  rclcpp::Publisher<PathMsg>::SharedPtr path_frame_changed_publisher_;

  // donkey_gps subscription
  rclcpp::Subscription<PathMsg>::SharedPtr path_subscription_;
};

}  // namespace geometry_transformer::ros2

#endif  // GEOMETRY_TRANSFORMER__ROS2__PATH_TRANSFORMER_NODE_HPP_
