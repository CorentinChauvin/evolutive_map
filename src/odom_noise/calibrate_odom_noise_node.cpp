/*
 *
*/

// Includes
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>	// for quaternions

// Namespaces
using namespace std;

// Global variables
nav_msgs::Odometry initialOdom, initialDiffOdom, lastOdom, lastDiffOdom;
ros::Publisher pub_command;
bool isOdomInitialised(false), isDiffOdomInitialised(false);
bool isForwardInitialised(false), isTurnInitialised(false), isForwardFinished(false), isTurnFinished(false);


// Callbacks
void odom_callback(nav_msgs::Odometry odom) {
    lastOdom = odom;
    isOdomInitialised = true;
}

void diffOdom_callback(nav_msgs::Odometry diffOdom) {
    lastDiffOdom = diffOdom;
    isDiffOdomInitialised = true;
}

// Other functions
double cartesianDistance(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2) {
    /* Compute the cartesian distance between two odometry poses (in the (x,y) plan)
    */
    double diff_x = odom1.pose.pose.position.x - odom2.pose.pose.position.x;
    double diff_y = odom1.pose.pose.position.y - odom2.pose.pose.position.y;

    return sqrt(pow(diff_x, 2) + pow(diff_y, 2));

}

double yawDifference(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2) {
    /* Compute the radian yaw difference between two odometry poses
    */
    double foo, yaw1, yaw2;
    tf::Quaternion quat1, quat2, quatDiff;
    tf::quaternionMsgToTF(odom1.pose.pose.orientation, quat1);
    tf::quaternionMsgToTF(odom2.pose.pose.orientation, quat2);
    tf::Matrix3x3(quat1).getRPY(foo, foo, yaw1);
    tf::Matrix3x3(quat2).getRPY(foo, foo, yaw2);

    return yaw1 - yaw2;
}


void goForward() {
    /* Move forward for 1 meter and see the difference between the real and noised odom
    */
    if (!isForwardInitialised && isOdomInitialised && isDiffOdomInitialised) {
        initialOdom = lastOdom;
        initialDiffOdom = lastDiffOdom;
        isForwardInitialised = true;
        cout << "Begining to move forward" << endl;
    } else if (cartesianDistance(lastOdom, initialOdom) < 1.0) {
        // The robot has not moved enough : send the move forward command
        geometry_msgs::Twist command;
        command.linear.y = command.angular.z = 0;
        command.linear.x = 0.25;    // in m/s
        pub_command.publish(command);
    } else {
        // The robot has moved enough : display the final odometry difference
        cout << "Difference after moving forward for 1 meter" << endl;

        cout << "d_x=" << lastDiffOdom.pose.pose.position.x - initialDiffOdom.pose.pose.position.x;
        cout << " ; d_y=" << lastDiffOdom.pose.pose.position.y - initialDiffOdom.pose.pose.position.y;
        cout << " ; d_yaw=" << yawDifference(lastDiffOdom, initialDiffOdom) * 360 / M_PI  << " (in degrees)" << endl;

        isForwardFinished = true;

        // Then stop the robot
        geometry_msgs::Twist command;
        command.linear.x = command.linear.y = command.angular.z = 0;
        pub_command.publish(command);
        ros::Duration(1.0).sleep();
    }
}

void turn() {
    /* Turn for 360 degrees and see the difference between the real and noised odom
    */
    if (!isTurnInitialised) {
        initialOdom = lastOdom;
        initialDiffOdom = lastDiffOdom;
        isTurnInitialised = true;
        cout << "Begining to turn" << endl;
    } else if (abs(yawDifference(lastOdom, initialOdom) * 180 / M_PI) < 180) {
        // The robot has not turned enough : send the turn command
        geometry_msgs::Twist command;
        command.linear.x = command.linear.y = 0;
        command.angular.z = 0.75;
        pub_command.publish(command);
    } else {
        // The robot has turned enough : display the final odometry difference
        cout << "Difference after turning 180 degrees" << endl;

        cout << "d_x=" << lastDiffOdom.pose.pose.position.x - initialDiffOdom.pose.pose.position.x;
        cout << " ; d_y=" << lastDiffOdom.pose.pose.position.y - initialDiffOdom.pose.pose.position.y;
        cout << " ; d_yaw=" << yawDifference(lastDiffOdom, initialDiffOdom) * 360 / M_PI  << " (in degrees)" << endl;

        isTurnFinished = true;
    }
}



int main(int argc, char** argv)
{
	// ROS Initialisation
	ros::init(argc, argv, "calibrate_odom_noise_node");
	ROS_INFO("Node calibrate_odom_noise_node connected to roscore");
	ros::NodeHandle nh_("~"); //ROS Handler - local namespace.

	// Subscribing
	ros::Subscriber sub_odom = nh_.subscribe<nav_msgs::Odometry>("/groundTruthOdom", 1, odom_callback);
	ros::Subscriber sub_diffOdom = nh_.subscribe<nav_msgs::Odometry>("/diffOdom", 1, diffOdom_callback);

    // Publishing
    pub_command = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);


	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();

		if (!isForwardFinished)
            goForward();
        else if (!isTurnFinished)
            turn();
        else
            return 0;

		rate.sleep();
	}

}
