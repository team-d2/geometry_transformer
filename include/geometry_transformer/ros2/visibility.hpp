// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef GEOMETRY_TRANSFORMER__ROS2__VISIBILITY_HPP_
#define GEOMETRY_TRANSFORMER__ROS2__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borroexampled (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define GEOMETRY_TRANSFORMER__ROS2_EXPORT __attribute__((dllexport))
#define GEOMETRY_TRANSFORMER__ROS2_IMPORT __attribute__((dllimport))
#else
#define GEOMETRY_TRANSFORMER__ROS2_EXPORT __declspec(dllexport)
#define GEOMETRY_TRANSFORMER__ROS2_IMPORT __declspec(dllimport)
#endif

#ifdef GEOMETRY_TRANSFORMER__ROS2_DLL
#define GEOMETRY_TRANSFORMER__ROS2_PUBLIC GEOMETRY_TRANSFORMER__ROS2_EXPORT
#else
#define GEOMETRY_TRANSFORMER__ROS2_PUBLIC GEOMETRY_TRANSFORMER__ROS2_IMPORT
#endif

#define GEOMETRY_TRANSFORMER__ROS2_PUBLIC_TYPE GEOMETRY_TRANSFORMER__ROS2_PUBLIC

#define GEOMETRY_TRANSFORMER__ROS2_LOCAL

#else

#define GEOMETRY_TRANSFORMER__ROS2_EXPORT __attribute__((visibility("default")))
#define GEOMETRY_TRANSFORMER__ROS2_IMPORT

#if __GNUC__ >= 4
#define GEOMETRY_TRANSFORMER__ROS2_PUBLIC __attribute__((visibility("default")))
#define GEOMETRY_TRANSFORMER__ROS2_LOCAL __attribute__((visibility("hidden")))
#else
#define GEOMETRY_TRANSFORMER__ROS2_PUBLIC
#define GEOMETRY_TRANSFORMER__ROS2_LOCAL
#endif

#define GEOMETRY_TRANSFORMER__ROS2_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // GEOMETRY_TRANSFORMER__ROS2__VISIBILITY_HPP_
