/*
 * Source code for the node odom_noise_node
 * Get the ground truth pose of the robot provided by Gazebo and create a noised odometry
*/

// Includes
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>	// for quaternions


// Namespaces
using namespace std;

// Global variables
ros::Publisher pub_noisedOdom, pub_diffOdom, pub_groundTruthOdom;
nav_msgs::Odometry odom, lastOdom, lastNoisedOdom;
bool isOdomInialised = false;
default_random_engine randomGenerator;  // Random generator for noises
double alpha1, alpha2, alpha3, alpha4;  // Odometry noise parameters


// Useful functions
double randomGaussianDouble(double a, double b) {
	normal_distribution<double> distribution(a, b);
	return distribution(randomGenerator);
}


// Callbacks
void states_callback(gazebo_msgs::ModelStates modelStates) {
	/* Get the odometry from the ground truth provided by gazebo
       Also publish the ground truth odometry
	*/

	// Find the mobile_base in the modelStates message and fill odom with its ground truth values
	for (int i = 0 ; i < modelStates.name.size() ; i++) {
		if (modelStates.name[i] == "mobile_base") {
			odom.pose.pose = modelStates.pose[i];
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation.x = 0.0;
			odom.pose.pose.orientation.y = 0.0;
			odom.twist.twist = modelStates.twist[i];
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			break;
		}
	}

	if (!isOdomInialised) {
		// Initialise each of the global variables
		lastOdom = odom;
		lastNoisedOdom = odom;

		isOdomInialised = true;

		ROS_INFO("Noised odom initialised.");
	}
}

void computeNoisedOdom() {
	/* Compute a noised odom from the ground truth provided by Gazebo
	   Fill also the groundTruthOdom variable
	*/

	if (isOdomInialised) {
		// Compute the difference between the current odom and the last one
		double d_x = odom.pose.pose.position.x - lastOdom.pose.pose.position.x;
		double d_y = odom.pose.pose.position.y - lastOdom.pose.pose.position.y;

		double yawOdom, yawLastOdom, foo;
		tf::Quaternion quatOdom, quatLastOdom;
		tf::quaternionMsgToTF(odom.pose.pose.orientation, quatOdom);
		tf::quaternionMsgToTF(lastOdom.pose.pose.orientation, quatLastOdom);
		tf::Matrix3x3(quatOdom).getRPY(foo, foo, yawOdom);
		tf::Matrix3x3(quatLastOdom).getRPY(foo, foo, yawLastOdom);

		double d_yaw = yawOdom - yawLastOdom;

		// Compute the deltas according to the odometry model
		double d_trans = sqrt(d_x*d_x + d_y*d_y);
		double d_rot1 = atan2(d_y, d_x) - yawLastOdom;
		double d_rot2 = yawOdom - yawLastOdom - d_rot1;

		// Noise the deltas if the robot has moved a given threshold (simulating encoders ticks)(11.7 ticks/mm)
		double noised_d_trans, noised_d_rot1, noised_d_rot2;

		if (abs(d_trans) > 0.00007 || abs(d_yaw) > 0.006) {
			noised_d_trans = d_trans + randomGaussianDouble(0, alpha3*d_trans + alpha4*(abs(d_rot1)+abs(d_rot2)));
			noised_d_rot1 = d_rot1 + randomGaussianDouble(0, alpha1*abs(d_rot1) + alpha2*d_trans);
			noised_d_rot2 = d_rot2 + randomGaussianDouble(0, alpha1*abs(d_rot2) + alpha2*d_trans);
		} else {
			noised_d_trans = d_trans;
			noised_d_rot1 = d_rot1;
			noised_d_rot2 = d_rot2;
		}

		// Compute the new noised position from the real position difference
		nav_msgs::Odometry noisedOdom;
		noisedOdom.pose.pose.position.x = lastNoisedOdom.pose.pose.position.x + noised_d_trans*cos(yawLastOdom+noised_d_rot1);
		noisedOdom.pose.pose.position.y = lastNoisedOdom.pose.pose.position.y + noised_d_trans*sin(yawLastOdom+noised_d_rot1);
		noisedOdom.pose.pose.position.z = lastNoisedOdom.pose.pose.position.z;

		// Compute the new noised orientation from the real orientation difference
		double yawNoisedOdom, yawLastNoisedOdom;
		tf::Quaternion quatNoisedOdom, quatLastNoisedOdom;

		tf::quaternionMsgToTF(lastNoisedOdom.pose.pose.orientation, quatLastNoisedOdom);
		tf::Matrix3x3(quatLastNoisedOdom).getRPY(foo, foo, yawLastNoisedOdom);

		yawNoisedOdom = yawLastNoisedOdom + noised_d_rot1 + noised_d_rot2;
		quatNoisedOdom.setRPY(0.0, 0.0, yawNoisedOdom);
		tf::quaternionTFToMsg(quatNoisedOdom, noisedOdom.pose.pose.orientation); // fill the message orientation field

		// Copy the speed of the real odometry
		noisedOdom.twist = odom.twist;

		// Set headers
		noisedOdom.header.stamp = ros::Time::now();
		noisedOdom.child_frame_id = "base_footprint";
		odom.header.stamp = ros::Time::now();
		odom.child_frame_id = "base_footprint";

        // Publish the noised and ground truth odometry
        pub_noisedOdom.publish(noisedOdom);
        pub_groundTruthOdom.publish(odom);

		// Publish the difference between noised and true odom
		nav_msgs::Odometry diffOdom;
        diffOdom.header.stamp = ros::Time::now();
		diffOdom.child_frame_id = "base_footprint";
		diffOdom.pose.pose.position.x = odom.pose.pose.position.x - noisedOdom.pose.pose.position.x;
		diffOdom.pose.pose.position.y = odom.pose.pose.position.y - noisedOdom.pose.pose.position.y;
		diffOdom.pose.pose.position.z = odom.pose.pose.position.z - noisedOdom.pose.pose.position.z;

		double yaw1, yaw2;
	    tf::Quaternion quat1, quat2, quatDiffOdom;
	    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat1);
	    tf::quaternionMsgToTF(noisedOdom.pose.pose.orientation, quat2);
	    tf::Matrix3x3(quat1).getRPY(foo, foo, yaw1);
	    tf::Matrix3x3(quat2).getRPY(foo, foo, yaw2);
		quatDiffOdom.setRPY(0.0, 0.0, yaw1-yaw2);
		tf::quaternionTFToMsg(quatDiffOdom, diffOdom.pose.pose.orientation);

		pub_diffOdom.publish(diffOdom);

		// Save the global variables
		lastOdom = odom;
		lastNoisedOdom = noisedOdom;
	}
}


int main(int argc, char** argv)
{
	// ROS Initialisation
	ros::init(argc, argv, "odom_noise_node");
	ROS_INFO("Node odom_noise_node connected to roscore");
	ros::NodeHandle nh_("~");//ROS Handler - local namespace.

	// Subscribing
	ros::Subscriber sub_states = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, states_callback);

	// Publishing
	pub_noisedOdom = nh_.advertise<nav_msgs::Odometry>("/noisedOdom", 1);
	pub_diffOdom = nh_.advertise<nav_msgs::Odometry>("/diffOdom", 1);
	pub_groundTruthOdom = nh_.advertise<nav_msgs::Odometry>("/groundTruthOdom", 1);

	// Parameters
	nh_.param<double>("alpha1", alpha1, 0.0);
	nh_.param<double>("alpha2", alpha2, 0.0);
	nh_.param<double>("alpha3", alpha3, 0.0);
	nh_.param<double>("alpha4", alpha4, 0.0);


	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();

		computeNoisedOdom();	// We want it at a fixed rate

		rate.sleep();
	}

}
