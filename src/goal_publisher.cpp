// goal_publisher.cpp - creates a ros2 node and publishes a goal topic
// Tristan Hill, turtlebot3_custom
// November 20, 2025
 
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals; // time durations with units

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

  // array of goal points
  float goallist[5][3]={{0.0,-1.0,0.0},   //{x,y,theta_z}
	    	        {0.0,-3.0,0.7},
			{0.0,-5.0,1.5}};

  int cnt=0;
  rclcpp::WallRate loop_rate(50);

  auto seconds = 30s; // variable for delay time

  while (rclcpp::ok()) {
	
	// format string with stringstream	  
  	str<<"main() loop cnt: "<<cnt<<std::endl;
	std::cout<<str.str(); // print string to terminal
	
	msg.pose.position.x = goallist[0][0];
        msg.pose.position.y = goallist[0][1];
        msg.pose.position.z = 0;
        msg.pose.orientation.w = goallist[0][2];

	std::cout<<"publishing goal 1"<<std::endl;
	pub->publish(msg);    // publish message
	rclcpp::spin_some(node); // spin ros2

        seconds = 30s;
	rclcpp::sleep_for(seconds);
	std::cout<<"waiting to reach goal 1"<<std::endl;

	msg.pose.position.x = goallist[1][0];
        msg.pose.position.y = goallist[1][1];
        msg.pose.position.z = 0;
        msg.pose.orientation.w = goallist[1][2];

	std::cout<<"publishing goal 2"<<std::endl;
	pub->publish(msg);    // publish message
	rclcpp::spin_some(node); // spin ros2
        
	seconds = 30s;
	rclcpp::sleep_for(seconds);
	std::cout<<"waiting to reach goal 2"<<std::endl;
	
	cnt++; // increment loop counter 
 	str.str(""); // empty string for next iteration
  }

  return 0;
} 
