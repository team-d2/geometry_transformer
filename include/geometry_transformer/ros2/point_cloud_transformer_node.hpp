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

#ifndef GEOMETRY_TRANSFORMER__ROS2__POINT_CLOUD_TRANSFORMER_NODE_HPP_
#define GEOMETRY_TRANSFORMER__ROS2__POINT_CLOUD_TRANSFORMER_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "create_tf_listener.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visibility.hpp"
#include "Eigen/Core"
#include "pcl/conversions.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"

namespace geometry_transformer::ros2
{

class PointCloudTransformerNode : public rclcpp::Node
{
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

public:
  static constexpr auto kDefaultNodeName = "point_cloud_transformer";

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  inline PointCloudTransformerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", std::string("map"))),
    tf_buffer_(this->get_clock()),
    tf_listener_(create_tf_listener(this, tf_buffer_)),
    point_cloud_frame_changed_publisher_(this->create_point_cloud_frame_changed_publisher()),
    point_cloud_subscription_(this->create_point_cloud_subscription())
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PointCloudTransformerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PointCloudTransformerNode(node_name, "", options)
  {
  }

  GEOMETRY_TRANSFORMER__ROS2_PUBLIC
  explicit inline PointCloudTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PointCloudTransformerNode(kDefaultNodeName, "", options)
  {
  }

  ~PointCloudTransformerNode() override {}

private:
  rclcpp::Publisher<PointCloudMsg>::SharedPtr create_point_cloud_frame_changed_publisher()
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<PointCloudMsg>("points/transformed", rclcpp::QoS(10), publisher_options);
  }

  rclcpp::Subscription<PointCloudMsg>::SharedPtr create_point_cloud_subscription()
  {
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<PointCloudMsg>(
      "points", rclcpp::QoS(10), [this](PointCloudMsg::UniquePtr msg) {this->transform(std::move(msg));},
      subscription_options);
  }

  void transform(PointCloudMsg::UniquePtr points_msg)
  {
    // get header tf
    geometry_msgs::msg::TransformStamped point_cloud_base_tf_msg;
    try {
      point_cloud_base_tf_msg =
        tf_buffer_.lookupTransform(frame_id_, points_msg->header.frame_id, points_msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }

    Eigen::Isometry3f point_cloud_base_tf;
    point_cloud_base_tf.translation() = Eigen::Vector3f(
      point_cloud_base_tf_msg.transform.translation.x,
      point_cloud_base_tf_msg.transform.translation.y,
      point_cloud_base_tf_msg.transform.translation.z);
    Eigen::Quaternionf q(
      point_cloud_base_tf_msg.transform.rotation.w,
      point_cloud_base_tf_msg.transform.rotation.x,
      point_cloud_base_tf_msg.transform.rotation.y,
      point_cloud_base_tf_msg.transform.rotation.z);
    point_cloud_base_tf.linear() = q.toRotationMatrix();

    // transform point cloud
    points_msg->header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Iterator<float> points_x_itr(*points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> points_y_itr(*points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> points_z_itr(*points_msg, "z");

    while (points_x_itr != points_x_itr.end()) {
      Eigen::Vector3f point_in_source_frame(*points_x_itr, *points_y_itr, *points_z_itr);
      Eigen::Vector3f point_in_target_frame = point_cloud_base_tf * point_in_source_frame;

      *points_x_itr = point_in_target_frame.x();
      *points_y_itr = point_in_target_frame.y();
      *points_z_itr = point_in_target_frame.z();

      ++points_x_itr;
      ++points_y_itr;
      ++points_z_itr;
    }

    // publish the point_cloud in the target frame
    point_cloud_frame_changed_publisher_->publish(std::move(points_msg));
  }

  // parameter
  std::string frame_id_;

  // tf buffer & listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr point_cloud_frame_changed_publisher_;

  rclcpp::Subscription<PointCloudMsg>::SharedPtr point_cloud_subscription_;
};

}  // namespace geometry_transformer::ros2

#endif  // GEOMETRY_TRANSFORMER__ROS2__POINT_CLOUD_TRANSFORMER_NODE_HPP_
