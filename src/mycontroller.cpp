#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "second_assignment/ChangeVel.h"

// initialize a publisher
ros::Publisher pub;
// the current velocity is initialized at 1.0
float currentVelocity = 1.0;
// threshold value
float fr_threshold = 1.5;

// function to receive the minimum value in a given array 'arr[]',
float min(float arr[], int min, int max) //'min' and 'max' are two integers that specify the indexes of the
{
	float distance = 80;
	for (int i = min; i < max; i++)
	{

		if (arr[i] < distance)
		{
			distance = arr[i]; // array in which the minimum value has to be calculated
		}
	}
	return distance;
}

// implementation of the service used for controlling the velocity of the robot
bool changeVelocity(second_assignment::ChangeVel::Request &req, second_assignment::ChangeVel::Response &res)
{

	ROS_INFO("Input Received:", req.input);

	if (req.input == 'w')
	{
		res.multiplier = 2.0 * currentVelocity;
	}
	if (req.input == 's')
	{
		res.multiplier = 0.5 * currentVelocity;
	}
	if (req.input == 'r')
	{
		res.multiplier = 1;
	}
	currentVelocity = res.multiplier;
	ROS_INFO("Speed:", currentVelocity);

	return true;
}

// This function retrieves information from `/base_scan` topic and publishes to `/cmd_vel` topic to make the robot move.
void driveCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

	geometry_msgs::Twist my_vel;
	//laser array (range from 0 to 720)
	float array[msg->ranges.size()];
	for (int i = 0; i < msg->ranges.size(); i++)
		array[i] = msg->ranges[i];
	// receive the minimum distances on the left, on the right and in front of the robot
	float left = min(array, 600, msg->ranges.size() - 1);
	float right = min(array, 0, 120);
	float front = min(array, 330, 390);


	// this will implements the logic to make the robot move (see README for more info)
	if (front < fr_threshold)
	{ // check if the frontal distance is lower than fr_threshold

		if (left <= right) // checks if the distance on the left is lower than the right one
		{
			if (2 * left < right) // in this case the left distance (left) is at least 2.0 times smaller than the right distance (right), so i only need to turn to the right
				my_vel.angular.z = -1.5;

			else
			{ // the two lateral distances are too similar, better to go forward while turning
				my_vel.linear.x = 0.5;
				my_vel.angular.z = -2.0;
			}
		}																		 // if the cycle arrives here, it means that right<left
		else if (2 * right < left) // if the right distance (right) is at least 2.0 times smaller than the left distance (left), so i only need to turn to the left

			my_vel.angular.z = 1.5;
		else // the two lateral distances are too similar, better to go forward while turning
		{
			my_vel.linear.x = 0.5;
			my_vel.angular.z = 2.0;
		}
	}
	else
	{ // if the frontal distance is greater than fr_threshold, then go forward
		my_vel.linear.x = currentVelocity;
	}
	// publish the velocity to 'cmd_vel'
	pub.publish(my_vel);
}

int main(int argc, char **argv)
{

	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	// subscription to /base_scan topic, messages are passed to a callback function, here called driveCallback
	ros::Subscriber sub = nh.subscribe("/base_scan", 1000, driveCallback);
	// setting up the publisher for the topic /cmd_vel (pub says it will publish on that topic)
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	// advertising of the service /changeVel
	ros::ServiceServer service = nh.advertiseService("/changeVel", changeVelocity);
	ros::spin();
	return 0;
}
