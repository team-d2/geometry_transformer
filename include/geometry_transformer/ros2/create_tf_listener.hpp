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

#ifndef GEOMETRY_TRANSFORMER__ROS2__CREATE_TF_LISTENER_HPP_
#define GEOMETRY_TRANSFORMER__ROS2__CREATE_TF_LISTENER_HPP_

#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace geometry_transformer::ros2
{

template<class NodeT, class AllocatorT = std::allocator<void>>
inline tf2_ros::TransformListener create_tf_listener(
  NodeT && node, tf2_ros::Buffer & tf_buffer)
{
  auto static_sub_options = tf2_ros::detail::get_default_transform_listener_static_sub_options<AllocatorT>();
  static_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return tf2_ros::TransformListener(
    tf_buffer, // tf buffer
    node, // node pointer
    true, // spin_thread
    tf2_ros::DynamicListenerQoS(), // qos(dynamic)
    tf2_ros::StaticListenerQoS(), // static qos
    tf2_ros::detail::get_default_transform_listener_sub_options<AllocatorT>(), // sub_options(dynamic)
    static_sub_options // sub_options(static)
    );
}

}  // namespace geometry_transformer::ros2

#endif  // GEOMETRY_TRANSFORMER__ROS2__CREATE_TF_LISTENER_HPP_
