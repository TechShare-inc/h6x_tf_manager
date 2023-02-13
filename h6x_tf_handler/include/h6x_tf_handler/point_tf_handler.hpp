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

#include <geometry_msgs/msg/point_stamped.hpp>
#include "h6x_tf_handler/tf_handler_base.hpp"

namespace h6x_tf_handler
{
using PointTfHandler = TfHandlerBase<geometry_msgs::msg::PointStamped>;
}  // namespace h6x_tf_handler
