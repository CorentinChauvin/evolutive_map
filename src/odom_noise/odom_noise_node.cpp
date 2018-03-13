/*
 *
*/

// Includes
#include <iostream>
#include <random>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


// Namespaces
using namespace std;

// Global variables
ros::Publisher pub_noisedOdom, pub_diffOdom;
nav_msgs::Odometry lastOdom, lastNoisedOdom;
bool isOdomInialised = false;
default_random_engine randomGenerator;  // Random generator for noises
double alpha1, alpha2, alpha3, alpha4;


// Useful functions
double randomGaussianDouble(double a, double b) {
    normal_distribution<double> distribution(a, b);
    return distribution(randomGenerator);
}


// Callbacks
void odom_callback(nav_msgs::Odometry odom)
{
    nav_msgs::Odometry noisedOdom, deltaOdom;

    if (!isOdomInialised) {
        // Initialise each of the global variables
        lastOdom = odom;
        lastNoisedOdom = odom;

        noisedOdom = odom;
        isOdomInialised = true;

        ROS_INFO("Noised odom initialised.");
    } else {
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

        // Publish the noised odom
        noisedOdom.header.stamp = ros::Time::now();
        noisedOdom.child_frame_id = "base_footprint";

        pub_noisedOdom.publish(noisedOdom);

        // Publish the difference between noised and true odom
        nav_msgs::Odometry diffOdom;
        diffOdom.header.stamp = ros::Time::now();
        diffOdom.child_frame_id = "base_footprint";
        diffOdom.pose.pose.position.x = odom.pose.pose.position.x - noisedOdom.pose.pose.position.x;
        diffOdom.pose.pose.position.y = odom.pose.pose.position.y - noisedOdom.pose.pose.position.y;
        diffOdom.pose.pose.position.z = odom.pose.pose.position.z - noisedOdom.pose.pose.position.z;
        diffOdom.pose.pose.orientation.x = odom.pose.pose.orientation.x - noisedOdom.pose.pose.orientation.x;
        diffOdom.pose.pose.orientation.y = odom.pose.pose.orientation.y - noisedOdom.pose.pose.orientation.y;
        diffOdom.pose.pose.orientation.z = odom.pose.pose.orientation.z - noisedOdom.pose.pose.orientation.z;
        diffOdom.pose.pose.orientation.w = odom.pose.pose.orientation.w - noisedOdom.pose.pose.orientation.w;

        pub_diffOdom.publish(diffOdom);

        // Save the global variables
        lastOdom = odom;
        lastNoisedOdom = noisedOdom;
    }
}


int main(int argc, char** argv)
{
    // ROS Initialisation
    ros::init(argc, argv, "package_template_node");
    ROS_INFO("Node package_template_node connected to roscore");
    ros::NodeHandle nh("~");//ROS Handler - local namespace.

    // Subscribing
    ROS_INFO("Subscribing to topics\n");
    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry> ("/odom", 1, odom_callback);

    // Publishing
    pub_noisedOdom = nh.advertise<nav_msgs::Odometry>("/noisedOdom", 1);
    pub_diffOdom = nh.advertise<nav_msgs::Odometry>("/diffOdom", 1);

	// Parameters
	nh.param<double>("/alpha1", alpha1, 0.0);
	nh.param<double>("/alpha2", alpha2, 0.0);
	nh.param<double>("/alpha3", alpha3, 0.0);
	nh.param<double>("/alpha4", alpha4, 0.0);


    ros::Rate rate(100);
    ros::spin();
}
