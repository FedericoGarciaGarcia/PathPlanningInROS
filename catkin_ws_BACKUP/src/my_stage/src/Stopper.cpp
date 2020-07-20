/*
 * Stopper.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: roiyeho
 */

#include "Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper()
{
	steps = STEPS_TO_ROTATE;
	keepMoving  = true;
	advance = true;

	// Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("base_scan", 1, &Stopper::scanCallback, this);
}

// Send a velocity command
void Stopper::moveForward() {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
	steps++;
};

// Send a velocity command
void Stopper::randomRotate() {
	if(steps >= STEPS_TO_ROTATE) {
		steps = 0;

		ROS_INFO("Rotate randomly!");

		geometry_msgs::Twist msg;
		msg.angular.z = ROTATE_MPS * (3.1519/180.0);
		ros::Rate rate(10);

		for(int i=0; i<10; i++) {
			commandPub.publish(msg);
			rate.sleep();
		}
	}
};


// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex =  ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);


	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M) {
		ROS_INFO("Rotate!");
		geometry_msgs::Twist msg;
		msg.angular.z = -ROTATE_MPS * (M_PI/180.0f);
		commandPub.publish(msg);
		advance = false; // Dejar de avanzar
	}
	else {
		advance = true; // Seguir avanzando
	}
}

void Stopper::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok() && keepMoving) {
		if(advance)
		moveForward();

		randomRotate();
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}
}




