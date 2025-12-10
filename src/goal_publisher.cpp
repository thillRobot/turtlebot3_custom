// goal_publisher.cpp - creates a ros2 node and publishes a goal topic
// Tristan Hill, turtlebot3_custom
// November 20, 2025
 
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

int main(int argc, char * argv[])
{
  // initialize ros2 node		
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("goal_publisher", options);

  // initialize publisher 
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  //std_msgs::msg::String msg; // msg object to publish 
  std::stringstream str;    

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp=node->get_clock()->now();
  msg.header.frame_id="map";


  int cnt=0;
  rclcpp::WallRate loop_rate(50);
  while (rclcpp::ok()) {
	
	// format string with stringstream	  
  	str<<"main() loop cnt: "<<cnt<<std::endl;
	std::cout<<str.str(); // print string to terminal
	
	msg.pose.position.x = 0.0;
        msg.pose.position.y = 1.0;
        msg.pose.position.z = 0;
        msg.pose.orientation.w = 1.0;

	//msg.data=str.str();   // put string into msg data
	pub->publish(msg);    // publish message

	rclcpp::spin_some(node); // spin ros2
	cnt++; // increment loop counter 
 	str.str(""); // empty string for next iteration
  }

  return 0;
} 
