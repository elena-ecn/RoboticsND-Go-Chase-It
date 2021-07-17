/*
* Subscribes to robot's camera images, analyzes the images to determine the
* presence and position of a white ball and calls a service to drive the robot
* towards the ball.
*/
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Calls the /ball_chaser/command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z){

  // Request a service and pass the velocities to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the /ball_chaser/command_robot service
  if (!client.call(srv))
      ROS_ERROR("Failed to call service command_robot");
}

// Callback function that continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img){

  int white_pixel = 255;
  bool ball_inside_image = false;
  int column_index = 0;

  // Loop through each pixel in the image and check if there's a bright white one.
  // Then, identify if this pixel falls in the left, mid or right side of the image.
  // Depending on the white ball position, call the drive_bot function and pass velocities to it.
  // Request a stop when there's no white ball seen by the camera.
  for(int i = 0; i < img.height * img.step; i+=3){

    // Check all rgb channels for white pixel
    // red: img.data[i], green: img.data[i+1], blue: img.data[i+2]
    if ((img.data[i] == white_pixel) && (img.data[i+1] == white_pixel) && (img.data[i+2] == white_pixel)){
      ball_inside_image = true;
      column_index = int(i/3) % img.width;
      break;
    }
  }

  if (!ball_inside_image){
    // Ball not found in image, stop
    drive_robot(0.0, 0.0);
  }
  else if (column_index < img.width/3){
    // Ball is in the left side, turn left
    drive_robot(0, 0.5);
  }
  else if (column_index > img.width*2/3){
    // Ball is in the right side, turn right
    drive_robot(0, -0.5);
  }
  else{
    // Ball is in the middle, go forward
    drive_robot(1.0, 0.0);
  }

}


int main(int argc, char** argv){

  // Initialize the process_image node and create a handle to it.
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from /ball_chaser/command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside process_image_callback
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
