#include "ros/ros.h"
#include "turtlesim/Pose.h"

// Topic messages callback
void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	ROS_INFO("x: %.2f, y: %.2f", msg->x, msg->y);
}

int main(int argc, char **argv)
{
	// Initiate a new ROS node named "listener"
	ros::init(argc, argv, "listen_turtle");
	ros::NodeHandle node;
	
	// Subscribe to a given topic
	ros::Subscriber sub = node.subscribe("turtle1/pose", 1000, poseCallback);
	
	// Enter a loop, pumping callbacks
	ros::spin(); // Allow processing of incoming messages
	
	return 0;
}
