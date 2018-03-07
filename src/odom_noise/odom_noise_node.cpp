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
double alpha1=0.0001, alpha2=0.0001, alpha3=0.0001, alpha4=0.0001;


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
        //noisedOdom.child_frame_id = "base_footprint"; <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        pub_noisedOdom.publish(noisedOdom);

        // Publish the difference between noised and true odom
        nav_msgs::Odometry diffOdom;
        diffOdom.header.stamp = ros::Time::now();
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

        // TODO : essayer de generer l'odometrie a partir de /gazebo/model_states => theoriquement, petit bruit d'odometrie introduit par gazebo





    }
}

/*
name: [bookshelf, jersey_barrier, ground_plane_0, unit_cylinder_1, Dumpster, mobile_base]
pose:
  -
    position:
      x: 0.0
      y: 1.53026
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  -
    position:
      x: -4.0
      y: -1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.342897807455
      w: 0.939372712847
  -
    position:
      x: 0.497681
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  -
    position:
      x: -1.99999999999
      y: -3.48880000005
      z: 0.499120000039
    orientation:
      x: -2.22663668149e-11
      y: -2.27691340047e-11
      z: -7.46721789377e-10
      w: 1.0
  -
    position:
      x: 0.999999522584
      y: -3.44458004744
      z: 0.000783506610574
    orientation:
      x: 1.43255358577e-06
      y: -0.000101977660203
      z: 6.19666956736e-08
      w: 0.999999994799
  -
    position:
      x: -0.299766714827
      y: -0.689234294939
      z: -0.00113073179688
    orientation:
      x: 0.00366328708223
      y: 0.00160241086105
      z: 0.918528094487
      w: -0.395335493278
twist:
  -
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  -
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  -
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  -
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  -
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  -
    linear:
      x: 3.31031613028e-05
      y: -8.78382811802e-05
      z: 3.67016577692e-05
    angular:
      x: 0.000128963675904
      y: 0.000356629034858
      z: -0.00026078371632
---
*/

/*header:
  seq: 35365
  stamp:
    secs: 353
    nsecs: 950000000
  frame_id: "odom"
child_frame_id: "base_footprint"
pose:
  pose:
    position:
      x: 0.768396561567
      y: -0.0770501053568
      z: 0.0
    orientation:
      x: 0.0
      y: -0.0
      z: 0.99979230311
      w: -0.0203801531297
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
twist:
  twist:
    linear:
      x: -1.21043981693e-05
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.00154933941484
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
*/



int main(int argc, char** argv)
{
    // ROS Initialisation
    ros::init(argc, argv, "package_template_node");
    ROS_INFO("Node package_template_node connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.

    // Subscribing
    ROS_INFO("Subscribing to topics\n");
    ros::Subscriber sub_odom = nh_.subscribe<nav_msgs::Odometry> ("/odom", 1, odom_callback);

    // Publishing
    pub_noisedOdom = nh_.advertise<nav_msgs::Odometry>("/noisedOdom", 1);
    pub_diffOdom = nh_.advertise<nav_msgs::Odometry>("/diffOdom", 1);


    ros::Rate rate(100);
    ros::spin();
}
