#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
	//while(ros::ok()) {
		geometry_msgs::Twist motor_command;
		motor_command.linear.x = req.linear_x;
		motor_command.angular.z = req.angular_z;
		motor_command_publisher.publish(motor_command);
	//}
	std::ostringstream ss;
	ss << req.linear_x;
	ss << " ";
	ss << req.angular_z;
	res.msg_feedback = ss.str();
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ball_chaser");
	ros::NodeHandle n;
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ros::spin();
  	return 0;
}
