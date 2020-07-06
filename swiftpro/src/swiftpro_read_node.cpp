/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *		   David Long <xiaokun.long@ufactory.cc>
 * ROS2 update: Dominik Franek <dominik.franek@gmail.com>
 */
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <swiftpro/msg/swiftpro_state.hpp>

/*
 * Node name:
 *	 swiftpro_read_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   position_read_topic
 */
class SwiftproReadNode : public rclcpp::Node
{
public:
  SwiftproReadNode()
    : Node("swiftpro_read_node")
    , publisher(this->create_publisher<swiftpro::msg::SwiftproState>("SwiftproState_topic", 1))
    , timer(this->create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&SwiftproReadNode::timerCallback,
                                    this)))
  {

	  this->serial.setPort("/dev/ttyACM0");
	  this->serial.setBaudrate(115200);
	  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	  this->serial.setTimeout(to);
	  this->serial.open();
	  RCLCPP_INFO_STREAM(this->get_logger(), "Port has been open successfully");

	  if (this->serial.isOpen())
	  {
		  rclcpp::sleep_for(std::chrono::seconds(3)); // wait 3s
		  this->serial.write("M2019\r\n"); // detach
		  rclcpp::sleep_for(std::chrono::milliseconds(500)); // wait 0.5s
		  this->serial.write("M2120 V0.05\r\n"); // report position per 0.05s
		  RCLCPP_INFO_STREAM(this->get_logger(), "Start to report data");
	  }
  }

private:
  void handleStr()
  {
	  char* pch = strtok(this->strData, " ");
	  double value[8];
	  int   index = 0;

	  while (pch != NULL && index < 5)
	  {
		  value[index] = atof(pch+1);
		  pch = strtok(NULL, " ");
		  index++;
	  }
	  this->position[0] = value[1];
	  this->position[1] = value[2];
	  this->position[2] = value[3];
	  this->position[3] = value[4];
  }

  void handleChar(char c)
  {
	  static int index = 0;

	  switch(c)
	  {
		  case '\r':
			  break;

		  case '\n':
			  this->strData[index] = '\0';
			  this->handleStr();
			  index = 0;
			  break;

		  default:
			  this->strData[index++] = c;
			  break;
	  }
  }

  void timerCallback()
  {
		if (this->serial.available())
		{
		  std_msgs::msg::String result;
		  swiftpro::msg::SwiftproState swiftproState;
			result.data = this->serial.read(this->serial.available());
			// RCLCPP_INFO_STREAM(this->get_logger(), "Read:" << result.data);
			for (size_t i = 0; i < result.data.length(); i++)
				this->handleChar(result.data.c_str()[i]);

			swiftproState.pump = 0;
			swiftproState.gripper = 0;
			swiftproState.swiftpro_status = 0;
			swiftproState.motor_angle1 = 0.0;
			swiftproState.motor_angle2 = 0.0;
			swiftproState.motor_angle3 = 0.0;
			swiftproState.motor_angle4 = this->position[3];
			swiftproState.x = this->position[0];
			swiftproState.y = this->position[1];
			swiftproState.z = this->position[2];
			publisher->publish(swiftproState);
			RCLCPP_INFO(this->get_logger(),
			  "position: %.2f %.2f %.2f %.2f",
			  this->position[0],
			  this->position[1],
			  this->position[2],
			  this->position[3]
			);
		}
  }

  double position[4] = {0.0}; // 3 cartesian coordinates: x, y, z(mm) and 1 angle(degree)
  char strData[2048]; // global variables for handling string

  rclcpp::Publisher<swiftpro::msg::SwiftproState>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  serial::Serial serial;	// serial object
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SwiftproReadNode> node;
  try
  {
    node = std::make_shared<SwiftproReadNode>();
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

