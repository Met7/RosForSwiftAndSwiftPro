/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 * ROS2 update: Dominik Franek <dominik.franek@gmail.com>
 */
#include <rclcpp/rclcpp.hpp>
#include <swiftpro/msg/swiftpro_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

/*
 * Node name:
 *	 swiftpro_rviz_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 jointStates
 *
 * Topic subscribe: (queue size = 1)
 *	 SwiftproState_topic
 */
class SwiftproRvizNode : public rclcpp::Node
{
  static constexpr double MATH_PI = 3.141592653589793238463;
  static constexpr double MATH_TRANS = 57.2958;
  static constexpr double MATH_L1 = 106.6;
  static constexpr double MATH_L2 = 13.2;
  static constexpr double MATH_LOWER_ARM = 142.07;
  static constexpr double MATH_UPPER_ARM = 158.81;
  static constexpr double MATH_UPPER_LOWER = MATH_UPPER_ARM / MATH_LOWER_ARM;

  static constexpr double LOWER_ARM_MAX_ANGLE = 135.6;
  static constexpr double LOWER_ARM_MIN_ANGLE = 0;
  static constexpr double UPPER_ARM_MAX_ANGLE = 100.7;
  static constexpr double UPPER_ARM_MIN_ANGLE = 0;
  static constexpr double LOWER_UPPER_MAX_ANGLE = 151;
  static constexpr double LOWER_UPPER_MIN_ANGLE = 10;
public:
  SwiftproRvizNode()
    : Node("swiftpro_rviz_node")
    , sub(this->create_subscription<swiftpro::msg::SwiftproState>(
            "SwiftproState_topic",
            1,
            std::bind(&SwiftproRvizNode::swiftproStateCallback, this, std::placeholders::_1)))
    , pub(this->create_publisher<sensor_msgs::msg::JointState>("jointStates", 1))
    , timer(this->create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&SwiftproRvizNode::timerCallback,
                                    this)))
  {
  	this->odomTrans.header.frame_id = "odom";
	  this->odomTrans.child_frame_id  = "Base";
  }

private:

  /*
   * Description: Get 9 joint angles from 3 motor angles
   * Inputs: 		angle[3]			3 motor angles(degree)
   * Outputs:		this->jointAngle[9]		9 joint angles(degree)
   */
  void allJointsState(double angle[3], double jointAngle[9])
  {
	  double alpha2;
	  double alpha3;

	  alpha2 = angle[1];
	  alpha3 = angle[2] - 3.8;

	  // 3 necessary joints for kinematic chain
	  jointAngle[0] = angle[0] - 90;
	  jointAngle[1] = 90 - alpha2;
	  jointAngle[5] = alpha3;

	  // 6 passive joints for display
	  jointAngle[2] = (alpha2 + alpha3) - 176.11 + 90;
	  jointAngle[3] = -90 + alpha2;
	  jointAngle[4] = jointAngle[1];
	  jointAngle[6] = 90 - (alpha2 + alpha3);
	  jointAngle[7] = 176.11 - 180 - alpha3;
	  jointAngle[8] = 48.39 + alpha3 - 44.55;
  }

  /*
   * Description: inverse kinematics of swift pro
   * Inputs: 		position[3]			3 cartesian coordinates: x, y, z(mm)
   * Outputs:		angle[3]			3 motor angles(degree)
   */
  bool swiftproIk(double position[3], double angle[3])
  {
	  double x = position[0];
	  double y = position[1];
	  double z = position[2];
	  double xIn, zIn, phi, rightAll, sqrtZX = 0.0;
	  double angleRot, angleLeft, angleRight = 0.0;

	  z += 74.55;
	  zIn = (z - MATH_L1) / MATH_LOWER_ARM;

	  if (x < 0.1)
		  x = 0.1;

	  // calculate value of theta1: the rotation angle
	  if (y == 0)
		  angleRot = 90;
	  else if (y < 0)
		  angleRot = -atan(x / y) * MATH_TRANS;
	  else if (y > 0)
		  angleRot = 180 - atan(x / y) * MATH_TRANS;

	  xIn 	= (x / sin(angleRot / MATH_TRANS) - MATH_L2 - 56.55) / MATH_LOWER_ARM;
	  phi 	= atan(zIn / xIn) * MATH_TRANS;
	  sqrtZX 	= sqrt(zIn * zIn + xIn * xIn);
	  rightAll   = (sqrtZX * sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER  - 1)
			     / (2 * MATH_UPPER_LOWER  * sqrtZX);
	  angleRight = acos(rightAll) * MATH_TRANS;

	  // calculate value of theta2 and theta3
	  rightAll   = (sqrtZX * sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER ) / (2 * sqrtZX);
	  angleLeft  = acos(rightAll) * MATH_TRANS;
	  angleLeft  = angleLeft + phi;
	  angleRight = angleRight - phi;

	  if (isnan(angleRot) || isnan(angleLeft) || isnan(angleRight))
		  return false;

	  angle[0] = angleRot;
	  angle[1] = angleLeft;
	  angle[2] = angleRight;
	  return true;
  }

  /*
   * Description: callback when receive data from position_read_topic
   * Inputs: 		msg(SwiftproState)	data about swiftpro
   * Outputs:		this->jointAngle[9]		9 joint angles(degree)
   */
  void swiftproStateCallback(const swiftpro::msg::SwiftproState::SharedPtr msg)
  {
	  double position[3];
	  double angle[3];

	  position[0] = msg->x;
	  position[1] = msg->y;
	  position[2] = msg->z;

	  if (swiftproIk(position, angle))
		  this->allJointsState(angle, this->jointAngle);
	  else
		  RCLCPP_ERROR(this->get_logger(), "Inverse kinematic is wrong");
  }

  void timerCallback()
  {
		this->jointState.header.stamp = this->now();
		this->jointState.name.resize(9);
		this->jointState.position.resize(9);
		this->jointState.name[0] = "Joint1";
		this->jointState.position[0] = this->jointAngle[0] / 57.2958;
    this->jointState.name[1] = "Joint2";
		this->jointState.position[1] = this->jointAngle[1] / 57.2958;
		this->jointState.name[2] = "Joint3";
		this->jointState.position[2] = this->jointAngle[2] / 57.2958;
		this->jointState.name[3] = "Joint4";
		this->jointState.position[3] = this->jointAngle[3] / 57.2958;
		this->jointState.name[4] = "Joint5";
		this->jointState.position[4] = this->jointAngle[4] / 57.2958;
		this->jointState.name[5] = "Joint6";
		this->jointState.position[5] = this->jointAngle[5] / 57.2958;
		this->jointState.name[6] = "Joint7";
		this->jointState.position[6] = this->jointAngle[6] / 57.2958;
		this->jointState.name[7] = "Joint8";
		this->jointState.position[7] = this->jointAngle[7] / 57.2958;
		this->jointState.name[8] = "Joint9";
		this->jointState.position[8] = this->jointAngle[8] / 57.2958;

		this->odomTrans.header.stamp = this->now();
		this->odomTrans.transform.translation.x = 0;
		this->odomTrans.transform.translation.y = 0;
		this->odomTrans.transform.translation.z = 0.0;
		tf2::Quaternion quaternion;
		quaternion.setEuler(10, 0, 0);
		quaternion.normalize();
		this->odomTrans.transform.rotation = tf2::toMsg(quaternion);

		this->pub->publish(this->jointState);
		this->broadcaster->sendTransform(this->odomTrans);
  }

	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
	sensor_msgs::msg::JointState jointState;
	geometry_msgs::msg::TransformStamped odomTrans;

  double jointAngle[9] = {0.0}; // 9 joint angles of swiftpro(degree)
  rclcpp::Subscription<swiftpro::msg::SwiftproState>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwiftproRvizNode>());
  rclcpp::shutdown();
  return 0;
}

