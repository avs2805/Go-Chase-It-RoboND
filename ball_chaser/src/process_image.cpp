#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the
// specified direction
void drive_robot(float lin_x, float ang_z)
{
  // Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the command_robot service and pass the requested velocities
  if (!client.call(srv))
    ROS_ERROR("Failed to call service command_robot!");
}

// the image callback function
// This continuously executes and reads the image data

void process_image_callback(const sensor_msgs::Image img)
{

  int white_pixel = 255;
  int ball_position = {0};
  bool ball_found = false;
  int left_boundary = img.step / 3;
  int right_boundary = 2 * img.step / 3;
  int image_center = img.step / 2;

  // ROS_INFO("Image Dims: height: %d | width: %d | step: %d", img.height, img.width, img.step);

  for (int i = 0; i < img.height * img.step; i++)
  {
    // Compare R, G & B channels to 255
    // Red img.data[i] | G = img.data[i+1] | B = img.data[i+2]

    if (img.data[i] == 255 && img.data[i + 1] == 255 && img.data[i + 2] == 255)
    {
      ball_found = true;
      ball_position = i % img.step;
      break;
    }
  }



  if (ball_found)
  {
    if (ball_position < left_boundary)
    {
      // ROS_INFO_STREAM("Driving Left");
      drive_robot(0.1, 0.7);
    }
    else if (ball_position > right_boundary)
    {
      // ROS_INFO_STREAM("Driving right");
      drive_robot(0.1, -0.7);
    }
    else if (ball_position >= left_boundary &&
             ball_position <= right_boundary)
    {
      if (ball_position < image_center)
      {
        drive_robot(0.5, 0.1);
      }
      else if (ball_position > image_center)
      {
        drive_robot(0.5, -0.1);
      }
      else
      {
        // ROS_INFO_STREAM("Driving Straight");
        drive_robot(0.5, 0);
      }
    }
  }
  else
  {
    // Stop the bot
    // ROS_INFO_STREAM("Ball not found.");
    drive_robot(0, 0);
  }
}

// The main function.

int main(int argc, char **argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from
  // command_robot
  client =
      n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside
  // the process_image_callback function
  ros::Subscriber sub1 =
      n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
