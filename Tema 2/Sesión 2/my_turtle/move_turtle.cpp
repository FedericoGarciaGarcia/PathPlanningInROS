#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
int main(int argc, char **argv)
{
	const double FORWARD_SPEED_MPS = 0.5;
	
	// Initialize the node
	ros::init(argc, argv, "move_turtle");
	ros::NodeHandle node;
	
	// A publisher for the movement data
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	
	// Publishing at 10 Hz
	ros::Rate rate(10);
	
	// Drive forward at a given speed. The robot points up the x-axis.
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;

	// Publishing movement commands until we shut down
	while (ros::ok()) {
		ROS_INFO("Moving forward");
		pub.publish(msg);
		rate.sleep();
	}
}
