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
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace h6x_tf_handler
{
using namespace std::chrono_literals;  // NOLINT
template<typename MessageT>
class TfHandlerBase
{
public:
  using SharedPtr = std::shared_ptr<TfHandlerBase>;
  using UniquePtr = std::unique_ptr<TfHandlerBase>;

private:
  using NodeClockInterface = rclcpp::node_interfaces::NodeClockInterface;
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;

  const NodeClockInterface::SharedPtr clock_if_;
  const NodeLoggingInterface::SharedPtr logging_if_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::chrono::nanoseconds timeout_ = 5s;
  std::string src_frame_id_, dist_frame_id_;

public:
  TfHandlerBase() = delete;
  explicit TfHandlerBase(
    const NodeClockInterface::SharedPtr,
    const NodeLoggingInterface::SharedPtr);

  void setSrcFrameId(const std::string &) noexcept;
  void setDistFrameId(const std::string &) noexcept;

  bool tfSrc2Dist(MessageT &) noexcept;
  [[deprecated("tfSrc2Dist(in, out) has been deprecated. Use tf2Dist(in, out) instead.")]]
  bool tfSrc2Dist(const MessageT &, MessageT &) noexcept;

  bool tfHeader2Dist(const MessageT &, MessageT &) noexcept;

  bool configure();
  bool activate(const std::chrono::nanoseconds = 5s);
  bool cleanup();

private:
  bool doTransform(const MessageT &, MessageT &);
};
}  // namespace h6x_tf_handler

#include "h6x_tf_handler/tf_handler_base_impl.hpp"
