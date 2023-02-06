// Copyright 2023 HarvestX Inc.
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

#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace h6x_tf_handler
{
using namespace std::chrono_literals;  // NOLINT
class PoseTfHandler
{
public:
  using SharedPtr = std::shared_ptr<PoseTfHandler>;
  using UniquePtr = std::unique_ptr<PoseTfHandler>;

private:
  using NodeClockInterface = rclcpp::node_interfaces::NodeClockInterface;
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  const NodeClockInterface::SharedPtr clock_if_;
  const NodeLoggingInterface::SharedPtr logging_if_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::chrono::milliseconds timeout_;
  std::string src_frame_id_, dist_frame_id_;

public:
  PoseTfHandler() = delete;
  explicit PoseTfHandler(
    const NodeClockInterface::SharedPtr,
    const NodeLoggingInterface::SharedPtr);

  void setSrcFrameId(const std::string &) noexcept;
  void setDistFrameId(const std::string &) noexcept;

  bool tfSrc2Dist(PoseStamped &) noexcept;
  bool tfSrc2Dist(const PoseStamped &, PoseStamped &) noexcept;

  bool configure();
  bool activate(const std::chrono::milliseconds = 5s);
  bool cleanup();

private:
  bool doTransform(const PoseStamped &, PoseStamped &);
};
}  // namespace h6x_tf_handler
