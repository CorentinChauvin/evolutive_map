/**
 * Provides a circuit to the robot
**/

// Constants
#define PI 3.14159265359

// C++
#include <iostream>
#include <cmath>

// ROS
#include "ros/ros.h"

// ROS msgs
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


//Namespaces
using namespace std;


// Global variables
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Point
struct Point{
	double x;
	double y;
};


int main (int argc, char** argv)
{
    // ROS Initialization
    ros::init(argc, argv, "evolutive_map");
    ROS_INFO("Node evolutive_map connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.
    
    
    Point goal1 = {0.0, 0.0};
    Point goal2 = {5.0, 5.0};
    Point goal3 = {3.0, -7.0};
    Point goalList[3] = { goal1, goal2, goal3 };
    int index_goal = 0;
    int nbrLoops = 0;
    
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

		//we'll send a goal to the robot
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.seq = 0;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = goalList[index_goal].x;
		goal.target_pose.pose.position.y = goalList[index_goal].y;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Goal achieved, moving on.");
			index_goal++;
		}
		else
		{
			ROS_INFO("Goal not reached, moving on to the next anyways...");
			index_goal++; //ou pas?
		}

			
		if (index_goal == 3){
			index_goal = 0;
			nbrLoops++;
		}

		rate.sleep();
    
    }

    ROS_INFO("ROS-Node Terminated\n");
    
    cout << "Turtlebot stopped after successfully realising " << nbrLoops << " loops." << endl;
    
    return 0;
}
