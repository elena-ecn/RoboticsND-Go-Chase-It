/*
* ROS node that implements a service which, when requested, publishes
* robot velocities to drive the robot around.
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;  // Define publisher

// Callback function that executes whenever a /ball_chaser/command_robot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
                          ball_chaser::DriveToTarget::Response &res){

  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;

  // Set robot velocities
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;

  // Publish robot velocities to drive the robot
  motor_command_publisher.publish(motor_command);

  // Return response message
  res.msg_feedback = "DriveToTarget request received. Velocities set. Linear x: "
  + std::to_string(motor_command.linear.x ) + ", angular z: " + std::to_string(motor_command.angular.z);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv){

  // Initialize the drive_bot ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // Inform ROS master that we will be publishing a message of type
  // geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a /ball_chaser/command_robot service with a handle_drive_request callback function
  ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
  ROS_INFO("Ready to send joint commands");

  // Handle ROS communication events
  ros::spin();

  return 0;
}
