/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 * ROS2 update: Dominik Franek <dominik.franek@gmail.com>
 */
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <swiftpro/msg/swiftpro_state.hpp>
#include <swiftpro/msg/status.hpp>
#include <swiftpro/msg/position.hpp>
#include <swiftpro/msg/angle4th.hpp>

using std::placeholders::_1;

/*
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
class SwiftproWriteNode : public rclcpp::Node
{
public:
  SwiftproWriteNode()
    : Node("swiftpro_write_node")
    , publisher(this->create_publisher<swiftpro::msg::SwiftproState>("SwiftproState_topic", 1))
    , sub1(this->create_subscription<swiftpro::msg::Position>(
             "position_write_topic", 1, std::bind(&SwiftproWriteNode::positionWriteCallback, this, _1)))
    , sub2(this->create_subscription<swiftpro::msg::Status>(
             "swiftpro_status_topic", 1, std::bind(&SwiftproWriteNode::swiftproStatusCallback, this, _1)))
    , sub3(this->create_subscription<swiftpro::msg::Angle4th>(
             "angle4th_topic", 1, std::bind(&SwiftproWriteNode::angle4thCallback, this, _1)))
    , sub4(this->create_subscription<swiftpro::msg::Status>(
            "gripper_topic", 1, std::bind(&SwiftproWriteNode::gripperCallback, this, _1)))
    , sub5(this->create_subscription<swiftpro::msg::Status>(
            "pump_topic", 1, std::bind(&SwiftproWriteNode::pumpCallback, this, _1)))
    , timer(this->create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&SwiftproWriteNode::timerCallback,
                                    this)))
  {
	  swiftpro::msg::SwiftproState swiftpro_state;

	  this->serial.setPort("/dev/ttyACM0");
	  this->serial.setBaudrate(115200);
	  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	  this->serial.setTimeout(to);
	  this->serial.open();
	  RCLCPP_INFO_STREAM(this->get_logger(), "Port has been open successfully");

	  if (this->serial.isOpen())
	  {
		  rclcpp::sleep_for(std::chrono::milliseconds(3500)); // wait 3.5s
		  this->serial.write("M2120 V0\r\n"); // stop report position
		  rclcpp::sleep_for(std::chrono::milliseconds(100)); // wait 0.1s
		  this->serial.write("M17\r\n");	// attach
		  rclcpp::sleep_for(std::chrono::milliseconds(100)); // wait 0.1s
		  RCLCPP_INFO_STREAM(this->get_logger(), "Attach and wait for commands");
	  }
  }

private:

  /*
   * Description: callback when receive data from position_write_topic
   * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
   * Outputs:		Gcode				send gcode to control swift pro
   */
  void positionWriteCallback(const swiftpro::msg::Position::SharedPtr msg)
  {
	  std::string Gcode = "";
	  std_msgs::msg::String result;
	  char x[10];
	  char y[10];
	  char z[10];

	  position.x = msg->x;
	  position.y = msg->y;
	  position.z = msg->z;
	  sprintf(x, "%.2f", msg->x);
	  sprintf(y, "%.2f", msg->y);
	  sprintf(z, "%.2f", msg->z);
	  Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
	  RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());
	  this->serial.write(Gcode.c_str());
	  result.data = this->serial.read(this->serial.available());
  }

  /*
   * Description: callback when receive data from angle4th_topic
   * Inputs: 		msg(float)			angle of 4th motor(degree)
   * Outputs:		Gcode				send gcode to control swift pro
   */
  void angle4thCallback(const swiftpro::msg::Angle4th::SharedPtr msg)
  {
	  std::string Gcode = "";
	  std_msgs::msg::String result;
	  char m4[10];

	  position.motor_angle4 = msg->angle4th;
	  sprintf(m4, "%.2f", msg->angle4th);
	  Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
	  RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());
	  this->serial.write(Gcode.c_str());
	  result.data = this->serial.read(this->serial.available());
  }

  /*
   * Description: callback when receive data from swiftpro_status_topic
   * Inputs: 		msg(uint8)			status of gripper: attach if 1; detach if 0
   * Outputs:		Gcode				send gcode to control swift pro
   */
  void swiftproStatusCallback(const swiftpro::msg::Status::SharedPtr msg)
  {
	  std::string Gcode = "";
	  std_msgs::msg::String result;

	  if (msg->status == 1)
		  Gcode = (std::string)"M17\r\n";   // attach
	  else if (msg->status == 0)
		  Gcode = (std::string)"M2019\r\n";
	  else
	  {
		  RCLCPP_INFO(this->get_logger(), "Error:Wrong swiftpro status input");
		  return;
	  }

	  position.swiftpro_status = msg->status;
	  RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());
	  this->serial.write(Gcode.c_str());
	  result.data = this->serial.read(this->serial.available());
  }

  /*
   * Description: callback when receive data from gripper_topic
   * Inputs: 		msg(uint8)			status of gripper: work if 1; otherwise 0
   * Outputs:		Gcode				send gcode to control swift pro
   */
  void gripperCallback(const swiftpro::msg::Status::SharedPtr msg)
  {
	  std::string Gcode = "";
	  std_msgs::msg::String result;

	  if (msg->status == 1)
		  Gcode = (std::string)"M2232 V1" + "\r\n";
	  else if (msg->status == 0)
		  Gcode = (std::string)"M2232 V0" + "\r\n";
	  else
	  {
		  RCLCPP_INFO(this->get_logger(), "Error:Wrong gripper status input");
		  return;
	  }

	  position.gripper = msg->status;
	  RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());
	  this->serial.write(Gcode.c_str());
	  result.data = this->serial.read(this->serial.available());
  }

  /*
   * Description: callback when receive data from pump_topic
   * Inputs: 		msg(uint8)			status of pump: work if 1; otherwise 0
   * Outputs:		Gcode				send gcode to control swift pro
   */
  void pumpCallback(const swiftpro::msg::Status::SharedPtr msg)
  {
	  std::string Gcode = "";
	  std_msgs::msg::String result;

	  if (msg->status == 1)
		  Gcode = (std::string)"M2231 V1" + "\r\n";
	  else if (msg->status == 0)
		  Gcode = (std::string)"M2231 V0" + "\r\n";
	  else
	  {
		  RCLCPP_INFO(this->get_logger(), "Error:Wrong pump status input");
		  return;
	  }

	  position.pump = msg->status;
	  RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());
	  this->serial.write(Gcode.c_str());
	  result.data = this->serial.read(this->serial.available());
  }

  void timerCallback()
  {
		publisher->publish(position);
	}

  swiftpro::msg::SwiftproState position;

  rclcpp::Publisher<swiftpro::msg::SwiftproState>::SharedPtr publisher;
  rclcpp::Subscription<swiftpro::msg::Position>::SharedPtr sub1;
  rclcpp::Subscription<swiftpro::msg::Status>::SharedPtr sub2;
  rclcpp::Subscription<swiftpro::msg::Angle4th>::SharedPtr sub3;
  rclcpp::Subscription<swiftpro::msg::Status>::SharedPtr sub4;
  rclcpp::Subscription<swiftpro::msg::Status>::SharedPtr sub5;
  rclcpp::TimerBase::SharedPtr timer;
  serial::Serial serial; // serial object
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SwiftproWriteNode> node;
  try
  {
    node = std::make_shared<SwiftproWriteNode>();
  }
	catch (serial::IOException& e)
  {
	  RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Unable to open port");
	  return -1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


