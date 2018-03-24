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
#include <tf/transform_listener.h>


// Namespaces
using namespace std;

// Global variables
ros::Publisher pub_noisedOdom, pub_diffOdom, pub_groundTruthOdom;
nav_msgs::Odometry GTOdom, lastGTOdom;	// Ground truth odoms : the first one is modified by the gazebo states_callback, the second by computeNoisedOdom
nav_msgs::Odometry lastOdom, lastNoisedOdom;
nav_msgs::Odometry initialOdomOffset;	// Initial pose offset between the GTOdom and odom
bool isGTOdomInitialised(false), isOdomOffsetInitialised(false);	// Whether
default_random_engine randomGenerator;  // Random generator for noises
double alpha1, alpha2, alpha3, alpha4;  // Odometry noise parameters


// Useful functions

double randomGaussianDouble(double mu, double sigma) {
	/* Generate a random number following a normal law (mu, sigma)
	*/

	normal_distribution<double> distribution(mu, sigma);
	return distribution(randomGenerator);
}


nav_msgs::Odometry computeOdometryDifference(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2) {
	/* Compute the pose difference between two odometry messages
	   Don't manage the header
	*/

	nav_msgs::Odometry diffOdom;

	diffOdom.pose.pose.position.x = odom1.pose.pose.position.x - odom2.pose.pose.position.x;
	diffOdom.pose.pose.position.y = odom1.pose.pose.position.y - odom2.pose.pose.position.y;
	diffOdom.pose.pose.position.z = odom1.pose.pose.position.z - odom2.pose.pose.position.z;

	double yaw1, yaw2, foo;
	tf::Quaternion quat1, quat2, quatDiffOdom;
	tf::quaternionMsgToTF(odom1.pose.pose.orientation, quat1);
	tf::quaternionMsgToTF(odom2.pose.pose.orientation, quat2);
	tf::Matrix3x3(quat1).getRPY(foo, foo, yaw1);
	tf::Matrix3x3(quat2).getRPY(foo, foo, yaw2);
	quatDiffOdom.setRPY(0.0, 0.0, yaw1-yaw2);
	tf::quaternionTFToMsg(quatDiffOdom, diffOdom.pose.pose.orientation);

	return diffOdom;
}


// Node behaviour functions

void states_callback(gazebo_msgs::ModelStates modelStates) {
	/* Get the odometry from the ground truth provided by gazebo
       Also publish the ground truth odometry
	*/

	// Find the mobile_base in the modelStates message and fill odom with its ground truth values
	for (int i = 0 ; i < modelStates.name.size() ; i++) {
		if (modelStates.name[i] == "mobile_base") {
			GTOdom.pose.pose = modelStates.pose[i];
			GTOdom.pose.pose.position.z = 0.0;
			GTOdom.pose.pose.orientation.x = 0.0;
			GTOdom.pose.pose.orientation.y = 0.0;
			GTOdom.twist.twist = modelStates.twist[i];
			GTOdom.twist.twist.linear.z = 0.0;
			GTOdom.twist.twist.angular.x = 0.0;
			GTOdom.twist.twist.angular.y = 0.0;
			break;
		}
	}

	if (!isGTOdomInitialised) {
		// Initialise each of the global variables
		lastGTOdom = GTOdom;
		lastNoisedOdom = GTOdom;

		isGTOdomInitialised = true;

		ROS_INFO("Noised odom initialised.");
	}
}


void odom_callback(nav_msgs::Odometry odom) {
	/* Callback for the /odom message
	   Initialise the offset between GTOdom and odom
    */

	lastOdom = odom;

	if (isGTOdomInitialised && !isOdomOffsetInitialised) {
		initialOdomOffset = computeOdometryDifference(GTOdom, odom);
		isOdomOffsetInitialised = true;
	}
}


void computeNoisedOdom() {
	/* Compute a noised odom from the ground truth provided by Gazebo
	   Fill also the groundTruthOdom variable
	*/

	if (isGTOdomInitialised) {
		// Compute the difference between the current odom and the last one
		double d_x = GTOdom.pose.pose.position.x - lastGTOdom.pose.pose.position.x;
		double d_y = GTOdom.pose.pose.position.y - lastGTOdom.pose.pose.position.y;

		double yawOdom, yawLastOdom, foo;
		tf::Quaternion quatOdom, quatLastOdom;
		tf::quaternionMsgToTF(GTOdom.pose.pose.orientation, quatOdom);
		tf::quaternionMsgToTF(lastGTOdom.pose.pose.orientation, quatLastOdom);
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
		noisedOdom.twist = GTOdom.twist;

		// Compute the difference between ground truth and noised odom
		nav_msgs::Odometry diffOdom = computeOdometryDifference(GTOdom, noisedOdom);

		// Set headers
		noisedOdom.header.stamp = ros::Time::now();
		noisedOdom.header.frame_id = "noisedOdom";
		noisedOdom.child_frame_id = "base_footprint";
		GTOdom.header.stamp = ros::Time::now();
		GTOdom.header.frame_id = "groundTruthOdom";
		GTOdom.child_frame_id = "base_footprint";
		diffOdom.header.stamp = ros::Time::now();
		diffOdom.header.frame_id = "diffOdom";
		diffOdom.child_frame_id = "base_footprint";

        // Publish
        pub_noisedOdom.publish(noisedOdom);
        pub_groundTruthOdom.publish(GTOdom);
		pub_diffOdom.publish(diffOdom);

		// Save the global variables
		lastGTOdom = GTOdom;
		lastNoisedOdom = noisedOdom;
	}
}

void publishTF(tf::TransformListener &tfListener, tf::TransformBroadcaster &tfBroadcaster) {
	/* Publish the different tf tranforms
	*/

	try {
		if (false) {
			// TO BE FIXED : enable to publish the noised odometry

			// Publish transform between GTOdom and odom
			nav_msgs::Odometry diffOdom = computeOdometryDifference(computeOdometryDifference(lastGTOdom, lastOdom), initialOdomOffset);
			geometry_msgs::Pose pose = diffOdom.pose.pose;
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
			transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

			string child_frame_id = "odom";		// frame defined by the transform
			string frame_id = "GTOdom";			// frame in wich the transform is defined
			tfBroadcaster.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), frame_id, child_frame_id));

			// Publish transform between noisedOdom and GTOdom
			diffOdom = computeOdometryDifference(lastGTOdom, lastNoisedOdom);
			pose = diffOdom.pose.pose;
			transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
			transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

			child_frame_id = "GTOdom";		// frame defined by the transform
			frame_id = "noisedOdom";		// frame in wich the transform is defined
			tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
		}
		if (true) {
			// Publish transform between odom and GTOdom
			nav_msgs::Odometry diffOdom = computeOdometryDifference(computeOdometryDifference(lastGTOdom, lastOdom), initialOdomOffset);
			geometry_msgs::Pose pose = diffOdom.pose.pose;
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
			transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

			string child_frame_id = "GTOdom";		// frame defined by the transform
			string frame_id = "odom";			// frame in wich the transform is defined
			tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
		}
	}
	// The tfListener and tfBroadcaster need a bit of time to be available, so the try catch is needed
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
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
	ros::Subscriber sub_odom = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, odom_callback);

	// Publishing
	pub_noisedOdom = nh_.advertise<nav_msgs::Odometry>("/noisedOdom", 1);
	pub_diffOdom = nh_.advertise<nav_msgs::Odometry>("/diffOdom", 1);
	pub_groundTruthOdom = nh_.advertise<nav_msgs::Odometry>("/groundTruthOdom", 1);

	// Parameters
	nh_.param<double>("alpha1", alpha1, 0.0);
	nh_.param<double>("alpha2", alpha2, 0.0);
	nh_.param<double>("alpha3", alpha3, 0.0);
	nh_.param<double>("alpha4", alpha4, 0.0);

	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;


	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();

		computeNoisedOdom();	// We want it at a fixed rate
		publishTF(tfListener, tfBroadcaster);

		rate.sleep();
	}

}
