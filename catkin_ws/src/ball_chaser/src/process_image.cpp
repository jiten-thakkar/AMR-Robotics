#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <cstdlib>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO("x: %f, z: %f\n", lin_x, ang_z);
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget drive;
    drive.request.linear_x = lin_x;
    drive.request.angular_z = ang_z;
    if (client.call(drive)) {
    	ROS_INFO("Message: %s", drive.response.msg_feedback.c_str());
    } else {
    	ROS_ERROR("Failed to call service DriveToTarget");
    }
}

void move_forward() {
  drive_robot(0.5, 0.0);
}

void move_left() {
  drive_robot(0.5, 0.5);
}

void move_right() {
  drive_robot(0.5, -0.5);
}

void white_ball_detected(int i, int j, int scene_width) {
  int scene_section_size = scene_width/3;
  if(j < scene_section_size) {
    move_left();
  } else if(j > scene_section_size && j < 2 * scene_section_size) {
    move_forward();
  } else {
    move_right();
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    ROS_INFO("Starting image processing");
    int white_pixel = 255;
    int ball_x, ball_y;
    /*for(int i=0; i<img.height; i++) {
    	for(int j=0; j<img.width; j++) {
		ROS_INFO("%d\n", img.data[i*img.width + j]);
		if(img.data[i*img.width + j] == white_pixel) {
		  white_ball_detected(i, j, img.width);
	  	  return;	  
		}
	}
    }*/
    for(int i=0; i<img.step * img.height; i+=3) {
    	if(img.data[i] == white_pixel 
	    && img.data[i+1] == white_pixel
	    && img.data[i+2] == white_pixel) {
		int k = i/3;
		ROS_INFO("k: %d, i: %d, j: %d\n", k, k/img.width, k%img.width);
		white_ball_detected(k/img.width, k%img.width, img.width);
		return;
	}
    }
    ROS_INFO("Finished image processing. Didn't find white ball.");
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
