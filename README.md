# Go-Chase-It
This project involves designing a differential drive mobile robot which is able to detect and chase white-colored balls inside a custom Gazebo world.


A mobile robot is designed with the Unified Robot Description Format (URDF). It employs two sensors: 
* Hokuyo lidar 
* camera
and utilizes Gazebo plugins for the differential drive and the sensors. 
Two nodes are used to detect a white ball and to drive the robot around in order to chase the ball.


## Key Concepts
Concepts explored in this project:
* Gazebo model and building editor
* Gazebo plugins
* URDF
* Launch files
* ROS Publishers, Subscribers and Services
* C++

### Structure
There are two packages in this project.
* **my_robot**: This package holds the robot, the white ball and the Gazebo world.

* **ball_chaser**: The ball_chaser package has two nodes: the *drive_bot* and *process_image*.
  * *drive_bot*: This server node provides a service to drive the robot by controlling its linear x and angular z velocities. This service publishes a message containing the requested robot velocities. 
  * *process_image*: This client node subscribes to the robot's camera images and analyzes each image to determine the presence and position of a white ball. If a white ball exists in the image, the node will request a service via a client to drive the robot towards it.

The directory structure is depicted below:


## Technologies
Project is created with:

## Installation
To run this project, you must have ROS and Gazebo installed.

To run this project, install it locally using:
```
git clone https://github.com/elena-ecn/RoboticsND-Go-Chase-It.git
```

## License
The contents of this repository are covered under the [MIT License](LICENSE).
