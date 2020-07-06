/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 * ROS2 update: Dominik Franek <dominik.franek@gmail.com>
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <swiftpro/msg/position.hpp>
#include <math.h>

/*
 * Node name:
 *	 swiftpro_moveit_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   position_write_topic
 *
 * Topic subscribe: (queue size = 1)
 *   move_group/fake_controller_joint_states
 */
class SwiftproMoveitNode : public rclcpp::Node
{
  static constexpr double MATH_PI = 3.141592653589793238463;
  static constexpr double MATH_TRANS = 57.2958;
  static constexpr double MATH_L1 = 106.6;
  static constexpr double MATH_L2 = 13.2;
  static constexpr double MATH_LOWER_ARM = 142.07;
  static constexpr double MATH_UPPER_ARM = 158.81;
public:
  SwiftproMoveitNode()
    : Node("swiftpro_moveit_node")
    , sub(this->create_subscription<sensor_msgs::msg::JointState>(
            "move_group/fake_controller_joint_states",
            1,
            std::bind(&SwiftproMoveitNode::jointCallback, this, std::placeholders::_1)))
    , pub(this->create_publisher<swiftpro::msg::Position>("position_write_topic", 1))
    , timer(this->create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&SwiftproMoveitNode::timerCallback,
                                    this)))
  {}

private:

  /*
   * Description: forward kinematics of swift pro
   * Inputs: 		angle[3]			3 motor angles(degree)
   * Outputs:		position[3]			3 cartesian coordinates: x, y, z(mm)
   */
  void swiftFk(double angle[3], double position[3])
  {
	  double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS)
				     + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

	  double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS)
				    - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;

	  position[0] = stretch * sin(angle[0] / MATH_TRANS);
	  position[1] = -stretch * cos(angle[0] / MATH_TRANS);
	  position[2] = height - 74.55;
  }

  /*
   * Description: callback when receive data from move_group/fake_controller_joint_states
   * Inputs: 		msg					3 necessary joints for kinematic chain(degree)
   * Outputs:		this->motorAngle[3]		3 motor angles(degree)
   */
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
	  this->motorAngle[0] = msg->position[0] * 57.2958 + 90;
	  this->motorAngle[1] = 90 - msg->position[1] * 57.2958;
	  this->motorAngle[2] = (msg->position[1] + msg->position[2]) * 57.2958;
  }

  void timerCallback()
  {
  	this->swiftFk(this->motorAngle, this->tempPosition);
		this->msgPosition.x = this->tempPosition[0];
		this->msgPosition.y = this->tempPosition[1];
		this->msgPosition.z = this->tempPosition[2];
		this->pub->publish(this->msgPosition);
	}

  double tempPosition[3];
	swiftpro::msg::Position msgPosition;
  double motorAngle[3] = {90.0, 90.0, 0.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
  rclcpp::Publisher<swiftpro::msg::Position>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwiftproMoveitNode>());
  rclcpp::shutdown();
  return 0;
}

