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

#include <string>
#include <memory>

#include "h6x_tf_handler/tf_handler_base.hpp"

namespace h6x_tf_handler
{
template<typename MessageT>
TfHandlerBase<MessageT>::TfHandlerBase(
  const NodeClockInterface::SharedPtr clock_if,
  const NodeLoggingInterface::SharedPtr logging_if)
: clock_if_(clock_if), logging_if_(logging_if)
{
  this->src_frame_id_.clear();
  this->dist_frame_id_.clear();
}

template<typename MessageT>
void TfHandlerBase<MessageT>::setSrcFrameId(const std::string & src_frame_id) noexcept
{
  this->src_frame_id_ = src_frame_id;
}

template<typename MessageT>
void TfHandlerBase<MessageT>::setDistFrameId(const std::string & dist_frame_id) noexcept
{
  this->dist_frame_id_ = dist_frame_id;
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::tfSrc2Dist(MessageT & in) noexcept
{
  in.header.frame_id = this->src_frame_id_;
  in.header.stamp = this->clock_if_->get_clock()->now();
  return this->doTransform(in, in);
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::tfSrc2Dist(const MessageT & in, MessageT & out) noexcept
{
  return this->tfHeader2Dist(in, out);
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::tfHeader2Dist(const MessageT & in, MessageT & out) noexcept
{
  if (in.header.frame_id == this->dist_frame_id_) {
    out = in;
    return true;
  }
  return this->doTransform(in, out);
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::configure()
{
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->clock_if_->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  return true;
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::activate(const std::chrono::nanoseconds timeout)
{
  // If src frame is empty, then transform-ability is not checked
  if (this->src_frame_id_.empty()) {
    return true;
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
    this->dist_frame_id_, this->src_frame_id_,
    this->clock_if_->get_clock()->now(), this->timeout_, &tf_error);
  if (!can_transform) {
    RCLCPP_ERROR(
      this->logging_if_->get_logger(), "%s -> %s : %s",
      this->src_frame_id_.c_str(), this->dist_frame_id_.c_str(), tf_error.c_str());
    return false;
  }
  return true;
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::cleanup()
{
  this->tf_listener_.reset();
  this->tf_buffer_.reset();
  return true;
}

template<typename MessageT>
bool TfHandlerBase<MessageT>::doTransform(const MessageT & in, MessageT & out)
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
