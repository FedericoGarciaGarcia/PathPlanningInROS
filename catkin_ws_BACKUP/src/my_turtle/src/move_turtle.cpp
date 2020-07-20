#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define PI 3.14159

int main(int argc, char **argv)
{
	const double FORWARD_SPEED_MPS = 0.5;
	
	// Set a name
	std::string robot_name = std::string(argv[1]);

	// Initialize the node
	ros::init(argc, argv, "move_turtle");
	ros::NodeHandle node;
	
	// A publisher for the movement data
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 1000);
	
	// Drive forward at a given speed. The robot points up the x-axis.
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;

	ros::Rate rate(1);
	rate.sleep();

	// Mover la tortuga 1m (2 mensajes a 1/2 metro por segundo)
	for(int i=0; i<2; i++) {
		ROS_INFO("Moving forward");
		
		pub.publish(msg);
		rate.sleep();
	}

	// Girar la tortuga 45 grados
	msg.linear.x = 0;
	msg.angular.z = 45.0 * (PI)/180.0;
	pub.publish(msg);
	rate.sleep();
}
