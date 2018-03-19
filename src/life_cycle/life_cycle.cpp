/**
 * Fait apparaitre et disparaitre des contenaires sur la map du robot. Il faut verifier qu'on les fait bien spawn:
 * Pas sur le robot, pas sur un autre element de decor ni hors-map.
**/

// Constants
#define PI 3.14159265359

// C++
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <time.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

// Gazebo
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
//#include <gazebo/urdf2gazebo.h>

// My files
#include "Container.h"
#include "detect_containers.h"


//Namespaces
using namespace std;

// Global variables
//ros::Publisher a_publisher; //si on veut s'en servir dans un callback par exemple
string mapImagePath;


// Callbacks
//void callback_aCallback(geometry_msgs::Pose2D pose) {
    /// Do something
//}


int main (int argc, char** argv)
{
	// ROS Initialization
    ros::init(argc, argv, "life_cycle");
    ROS_INFO("Node life_cycle connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.

    // Subscribing
    //ROS_INFO("Subscribing to topics\n");
    //ros::Subscriber a_subscriber = nh_.subscribe<a_message_type> ("/published_topic_name", [nbr message stockable en buffer, souvent 1], callback_aCallback);
    // Parameter
    //nh_.param<[the param type]>("my_param_name_in_ROS", my_param_variable, [default_value]);//my_param_variable=variable in my functions

	// Parameters
	nh_.param<string>("mapImagePath", mapImagePath, "");

	// Services
	ros::ServiceClient gazebo_spawn_clt= nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");


	int maplimits = 40; //a passer en parametre plus tard?
	maplimits*=2; //on redivisera par 2 après le tirage aleatoire de la position
	//pour avoir un pas de 0.5m


	// Container urdf loading
	string urdfPath = ros::package::getPath("evolutive_map") + "/urdf/cube.urdf";
	ifstream file(urdfPath.c_str());

	string line;
	gazebo_msgs::SpawnModel sm;

	while(!file.eof()) // Parse the contents of the given urdf in a string
	{
	  std::getline(file,line);
	  sm.request.model_xml+=line;
	}
	file.close();

	// Loading the map from the image
	srand(time(NULL));
	int nbContainers = 0; //attention c'est le nbr total de conteneur spawn depuis le debut, ceux delete y compris! Le nbr courant de conteneurs est listeContainers.size()
	int signs[2] = {-1, 1};
	vector<Container> listContainers;

	detect_containers(listContainers, nbContainers, mapImagePath);

	// Map generation
    ros::Duration(10).sleep(); // sleep for 10 seconds

	for (int i=0; i<nbContainers; i++) {
		Container new_cont = listContainers[i];

		sm.request.model_name = new_cont.getName();
		sm.request.reference_frame = "world";
		sm.request.initial_pose.position.x = new_cont.getX();
		sm.request.initial_pose.position.y = new_cont.getY();
		sm.request.initial_pose.position.z = 0.0;
		sm.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(new_cont.getYaw());

	  	if (gazebo_spawn_clt.call(sm)) //Call the spawn service
	  		cout << i << ": Initial container spawned !  " << new_cont << endl;
	}

	cout << "Map initialised with success!" << endl;

















	/*
	ros::Duration(180).sleep();//pause 3min

	//Boucle
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        double sleeptime = (rand()%100) / 4;
        ros::Duration(sleeptime).sleep(); // sleep for 0 to 25 seconds

        int actionMode = rand()%2;

		//Spawn a container

		if (actionMode == 0)
		{
			gazebo_msgs::SpawnModel sm;


			std::ifstream file("/home/ecn/ros/src/evolutive_map/urdf/cube.urdf");
		  	std::string line;

		   	while(!file.eof()) // Parse the contents of the given urdf in a string
			{
			  std::getline(file,line);
			  sm.request.model_xml+=line;
			}
		  	file.close();

			stringstream ss;
			ss << nbContainers;
			string nbstr = ss.str();

			string _nom = "box"+nbstr;
			nbContainers++;
			sm.request.model_name = _nom;
			sm.request.reference_frame = "world";

			double _x = signs[rand()%2]*(rand()%maplimits /2);
			double _y = signs[rand()%2]*(rand()%maplimits /2);
			double _z = 0.0; //hauteur de chute lors du spawn
			double _yaw = 0.0;

			sm.request.initial_pose.position.x = _x;
			sm.request.initial_pose.position.y = _y;
			sm.request.initial_pose.position.z = _z;
			sm.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(_yaw);

			Container cont(_x, _y, _z, _yaw, _nom);

		  	gazebo_spawn_clt.call(sm); //Call the spawn service
		  	listContainers.push_back(cont);//Ajouter ce nouvel objet a une liste des containers...
		  	cout << "Container spawned !  " << cont << endl;
	  	}


	  	//Delete a container

	  	ros::ServiceClient gazebo_delete_clt= nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

	  	if (actionMode == 1)
		{
		  	gazebo_msgs::DeleteModel dm;

		  	int s = listContainers.size();
		  	if (s != 0)
		  	{
			  	int whoToDelete = rand()%s;
			  	Container dcont = listContainers[whoToDelete];
			  	dm.request.model_name = dcont.getName();

			  	gazebo_delete_clt.call(dm); //Call the delete service
			  	listContainers.erase(listContainers.begin() + whoToDelete); //Supprimer cet objet de la liste des containers...
			  	cout << "Container deleted... " << dcont << endl;
		  	}
		}

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
    */
    return 0;
}
