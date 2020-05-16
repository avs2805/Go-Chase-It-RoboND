#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever
// a drive_bot service is requested

bool handle_drive_bot_request(ball_chaser::DriveToTarget::Request &req,
                              ball_chaser::DriveToTarget::Response &res)
{
  geometry_msgs::Twist motor_command;

  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;

  // Publishing velocities
  motor_command_publisher.publish(motor_command);

  // After publishing the requested velocities, a message feedback is returned
  // with the requested wheel velocities
  res.msg_feedback =
      "Motor commands set - Linear: " + std::to_string(req.linear_x) +
      " Angluar: " + std::to_string(req.angular_z);

  ROS_INFO_STREAM(res.msg_feedback);

  // Call the drive_bot service and pass the requested velocities
  return true;
}

// Defining the main function.

int main(int argc, char **argv)
{
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // Inform ROS master that we will be publishing a message of type
  // geometry_msgs::Twist on the robot actuation topic with a publishing queue
  // size of 10
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Create the Service Server named /ball_chaser/command_robot with a
  // handle_drive_request callback function

  ros::ServiceServer service =
      n.advertiseService("/ball_chaser/command_robot", handle_drive_bot_request);

  ROS_INFO("Ready to send motor commands.");

  ros::spin();
  return 0;
}
