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

#include "h6x_tf_handler/poes_tf_handler.hpp"

namespace h6x_tf_handler
{
PoseTfHandler::PoseTfHandler(
  const NodeClockInterface::SharedPtr clock_if,
  const NodeLoggingInterface::SharedPtr logging_if
)
: clock_if_(clock_if),
  logging_if_(logging_if)
{
  this->src_frame_id_.clear();
  this->dist_frame_id_.clear();
}

void PoseTfHandler::setSrcFrameId(const std::string & src_frame_id) noexcept
{
  this->src_frame_id_ = src_frame_id;
}


void PoseTfHandler::setDistFrameId(const std::string & dist_frame_id) noexcept
{
  this->dist_frame_id_ = dist_frame_id;
}

bool PoseTfHandler::tfSrc2Dist(PoseStamped & in) noexcept
{
  in.header.frame_id = this->src_frame_id_;
  in.header.stamp = this->clock_if_->get_clock()->now();
  in.pose.position.set__x(0.0).set__y(0.0).set__z(0.0);
  in.pose.orientation.set__w(1.0).set__x(0.0).set__y(0.0).set__z(0.0);
  return this->doTransform(in, in);
}

bool PoseTfHandler::tfSrc2Dist(const PoseStamped & in, PoseStamped & out) noexcept
{
  if (in.header.frame_id == this->dist_frame_id_) {
    // No need to transform
    out = in;
    return true;
  }
  return this->doTransform(in, out);
}

bool PoseTfHandler::configure()
{
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->clock_if_->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  return true;
}

bool PoseTfHandler::activate(const std::chrono::nanoseconds timeout)
{
  if (this->src_frame_id_.empty()) {
    RCLCPP_ERROR(this->logging_if_->get_logger(), "Src frame empty");
    return false;
  }
  if (this->dist_frame_id_.empty()) {
    RCLCPP_ERROR(this->logging_if_->get_logger(), "Dist frame empty");
    return false;
  }

  this->timeout_ = timeout;
  std::string tf_error;
  RCLCPP_INFO(
    this->logging_if_->get_logger(), "Checking transform %s -> %s",
    this->src_frame_id_.c_str(), this->dist_frame_id_.c_str());

  rclcpp::Rate r(10);
  const bool can_transform = this->tf_buffer_->canTransform(
    this->dist_frame_id_, this->src_frame_id_, this->clock_if_->get_clock()->now(),
    this->timeout_, &tf_error);
  if (!can_transform) {
    RCLCPP_ERROR(
      this->logging_if_->get_logger(), "%s -> %s : %s",
      this->src_frame_id_.c_str(), this->dist_frame_id_.c_str(), tf_error.c_str());
    return false;
  }
  return true;
}

bool PoseTfHandler::cleanup()
{
  this->tf_listener_.reset();
  this->tf_buffer_.reset();
  return true;
}

bool PoseTfHandler::doTransform(const PoseStamped & in, PoseStamped & out)
{
  try {
    out = this->tf_buffer_->transform(in, this->dist_frame_id_, this->timeout_);
    return true;
  } catch (const tf2::LookupException & e) {
    RCLCPP_ERROR(
      this->logging_if_->get_logger(),
      "No Transform available for looking up target frame: %s", e.what());
  } catch (const tf2::ConnectivityException & e) {
    RCLCPP_ERROR(
      this->logging_if_->get_logger(),
      "Connectivity Error looking up target frame: %s", e.what());
  } catch (const tf2::ExtrapolationException & e) {
    RCLCPP_ERROR(
      this->logging_if_->get_logger(),
      "Extrapolation Error looking up target frame: %s", e.what());
  } catch (const tf2::TimeoutException & e) {
    RCLCPP_ERROR(this->logging_if_->get_logger(), "Transform timeout");
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      this->logging_if_->get_logger(),
      "Failed to transform from %s -> %s : %s",
      this->src_frame_id_.c_str(), this->dist_frame_id_.c_str(), e.what());
  }

  return false;
}
}  // namespace h6x_tf_handler
