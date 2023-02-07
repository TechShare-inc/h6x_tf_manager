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

#include <gtest/gtest.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <h6x_tf_handler/pose_tf_handler.hpp>

using namespace h6x_tf_handler;  // NOLINT

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

geometry_msgs::msg::TransformStamped getTf(
  const std::string & frame_id, const std::string & child_frame_id,
  double x, double y, double z)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.set__frame_id(frame_id);
  tf.header.stamp = rclcpp::Clock().now();
  tf.set__child_frame_id(child_frame_id);
  tf.transform.translation.set__x(x).set__y(y).set__z(z);
  tf.transform.rotation.set__w(1.0).set__x(0.0).set__y(0.0).set__z(0.0);
  return tf;
}

class TestPoseTfNode : public rclcpp::Node
{
public:
  explicit TestPoseTfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_pose_tf_node", options)
  {}
};

class TestPoseTfHandler : public ::testing::Test
{
protected:
  TestPoseTfNode::SharedPtr host_node;
  PoseTfHandler::SharedPtr handler;

  const std::string src_frame_id = "src_link";
  const std::string dist_frame_id = "dist_link";

  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  virtual void SetUp()
  {
    this->host_node = std::make_shared<TestPoseTfNode>();
    this->executor.add_node(this->host_node);

    this->handler = std::make_shared<PoseTfHandler>(
      this->host_node->get_node_clock_interface(), this->host_node->get_node_logging_interface());

    this->tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
      this->host_node.get());

    const auto tf = getTf(this->dist_frame_id, this->src_frame_id, 1.0, 0.0, 0.0);
    this->tf_broadcaster_->sendTransform(tf);
    this->executor.spin_some();
  }

  void setFrames()
  {
    this->handler->setSrcFrameId(this->src_frame_id);
    this->handler->setDistFrameId(this->dist_frame_id);
  }
};

TEST_F(TestPoseTfHandler, woSrcFrame)
{
  this->handler->configure();
  // this->handler->setSrcFrameId(this->src_frame_id);
  this->handler->setDistFrameId(this->dist_frame_id);
  ASSERT_FALSE(this->handler->activate());
}

TEST_F(TestPoseTfHandler, woDistFrame)
{
  this->handler->configure();
  this->handler->setSrcFrameId(this->src_frame_id);
  // this->handler->setDistFrameId(this->dist_frame_id);
  ASSERT_FALSE(this->handler->activate());
}

TEST_F(TestPoseTfHandler, testActivationOK)
{
  this->handler->configure();
  this->setFrames();
  ASSERT_TRUE(this->handler->activate());
}

TEST_F(TestPoseTfHandler, testActivationNG)
{
  this->handler->configure();
  this->handler->setSrcFrameId("not_exist_frame");
  this->handler->setDistFrameId(this->dist_frame_id);
  ASSERT_FALSE(this->handler->activate());
}

TEST_F(TestPoseTfHandler, testTransform1)
{
  this->handler->configure();
  this->setFrames();
  using namespace std::chrono_literals;  // NOLINT
  this->handler->activate(100ms);

  geometry_msgs::msg::PoseStamped out;
  ASSERT_TRUE(this->handler->tfSrc2Dist(out));
  ASSERT_EQ(out.header.frame_id, this->dist_frame_id);
  ASSERT_DOUBLE_EQ(out.pose.position.x, 1.0);
}

TEST_F(TestPoseTfHandler, testTransform2) {
  this->handler->configure();
  this->setFrames();
  using namespace std::chrono_literals;  // NOLINT
  this->handler->activate(100ms);

  geometry_msgs::msg::PoseStamped in, out;
  in.header.set__frame_id(this->src_frame_id);
  in.pose.position.set__x(2.0);
  ASSERT_TRUE(this->handler->tfSrc2Dist(in, out));
  ASSERT_EQ(out.header.frame_id, this->dist_frame_id);
  ASSERT_DOUBLE_EQ(out.pose.position.x, 3.0);
}
