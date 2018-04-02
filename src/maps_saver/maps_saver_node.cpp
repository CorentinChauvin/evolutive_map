/*
 * Source code for the node map_saver
 * Save regularly the map in images
*/

// Includes
#include <cstdlib>
#include <string>
#include <ros/ros.h>

// Namespaces
using namespace std;

// Global variables
double delay;
string path;


int main (int argc, char** argv)
{
	// ROS Initialization
	ros::init(argc, argv, "maps_saver_node");
	ROS_INFO("Node maps_saver_node connected to roscore");
	ros::NodeHandle nh_("~"); //ROS Handler - local namespace.

	// Parameters
	nh_.param<double>("delay", delay, 60.0);
	nh_.param<string>("path", path, "/tmp/");

	int k = 0;

	ros::Duration(10.0).sleep();	// Wait for the map to be built a first time

	cout << "delay : " << 1/delay << endl;

	ros::Duration duration(delay);
	while (ros::ok())
	{
		ros::spinOnce();

		// Save the map
		string command = "mkdir -p " + path + " && rosrun map_server map_saver -f " + path + to_string(k);
		system(command.c_str());

		k++;
		duration.sleep();
	}
}
