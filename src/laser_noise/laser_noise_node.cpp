/*
 * Source code for the node laser_noise
 * Get the real scan message and publish a noised one
*/

// Includes
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


// Namespaces
using namespace std;

// Global variables
ros::Publisher pub_noisedScan, pub_diffScan;
default_random_engine randomGenerator;  // Random generator for noises
double sigma;  // Noise parameter


// Useful functions
double randomGaussianDouble(double a, double b) {
	normal_distribution<double> distribution(a, b);
	return distribution(randomGenerator);
}


// Callbacks
void scan_callback(sensor_msgs::LaserScan scan) {
	sensor_msgs::LaserScan noisedScan = scan;

	// Add noise to each measurement
	for (int i = 0 ; i < scan.ranges.size() ; i++) {
		double range = scan.ranges[i];

		if (range == range) {	// Ensure that range is not NaN
			if (range < 1.0)
				noisedScan.ranges[i] += randomGaussianDouble(0, sigma);
			else
				noisedScan.ranges[i] += randomGaussianDouble(0, sigma*abs(range));
		}
	}

	// Publish the noised scan
	noisedScan.header.stamp = ros::Time::now();
	pub_noisedScan.publish(noisedScan);

	// Publish the difference between the noised scan and the real one
	sensor_msgs::LaserScan diffScan = scan;

	for (int i = 0 ; i < scan.ranges.size() ; i++)
		diffScan.ranges[i] = scan.ranges[i] - noisedScan.ranges[i];

	diffScan.header.stamp = ros::Time::now();
	pub_diffScan.publish(diffScan);

}


int main(int argc, char** argv)
{
	// ROS Initialisation
	ros::init(argc, argv, "laser_noise_node");
	ROS_INFO("Node laser_noise_node connected to roscore");
	ros::NodeHandle nh_("~");//ROS Handler - local namespace.

	// Subscribing
	ros::Subscriber sub_scan = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, scan_callback);

	// Publishing
	pub_noisedScan = nh_.advertise<sensor_msgs::LaserScan>("/noisedScan", 1);
	pub_diffScan = nh_.advertise<sensor_msgs::LaserScan>("/diffScan", 1);

	// Parameters
	nh_.param<double>("sigma", sigma, 0.0);


	ros::Rate rate(100);
	ros::spin();

}
