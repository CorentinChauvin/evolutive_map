/**
 * Fait apparaitre et disparaitre des contenaires sur la map du robot. Il faut verifier qu'on les fait bien spawn:
 * Pas sur le robot, pas sur un autre element de decor ni hors-map.
**/

// C++
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <time.h>
#include <random>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

// Gazebo
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
//#include <gazebo/urdf2gazebo.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/physics/physics.hh>

// My files
#include "Container.h"
#include "detect_containers.h"


//Namespaces
using namespace std;

// Global variables

string mapImagePath;
int maplimits; // How far can a random container be spawned to ? Usualy 40 (m)
double minsleeptime; // minimal wait-time in seconds 2 actions of the life cycle (either spawn or delete)
double randsleeptime; // additional random wait-time in seconds
double leftoversRatio; // Between 0.0 et 1.0, filling ratio of the harbor, below which it's considered empty (or full, considering (1 - leftoversRatio))
// Beware if securitySpawningDistance is big and leftoversRatio is small, one may enconters problems with a non-moving robot
double securitySpawningDistance; // Security distance between the container and the robot during spawning in meters

double containerLength = 1.0; // defined in the urdf model of the containers
double containerWidth = 0.5; // defined in the urdf model of the containers

double robotPose_x;
double robotPose_y;


// Callbacks
void callback_odomReceived(nav_msgs::Odometry robot_odom) {
    robotPose_x = robot_odom.pose.pose.position.x;
    robotPose_y = robot_odom.pose.pose.position.y;
}


int main (int argc, char** argv)
{
	// ROS Initialization
    ros::init(argc, argv, "life_cycle");
    ROS_INFO("Node life_cycle connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.

    // Subscribing
    //ROS_INFO("Subscribing to topics\n");
    ros::Subscriber odom_subscriber = nh_.subscribe<nav_msgs::Odometry> ("/groundTruthOdom", 1, callback_odomReceived);

	// Parameters
	nh_.param<string>("mapImagePath", mapImagePath, "");
	nh_.param<int>("maplimits", maplimits, 40);
	nh_.param<double>("minsleeptime", minsleeptime, 0.0);
	nh_.param<double>("randsleeptime", randsleeptime, 30.0);
	nh_.param<double>("leftoversRatio", leftoversRatio, 0.1);
	nh_.param<double>("securitySpawningDistance", securitySpawningDistance, 2.0);

	cout << "Containers spawn/delete is set every " << minsleeptime + randsleeptime/2 << " seconds or so." << endl;
	cout << "Leftovers Ratio : " << leftoversRatio << endl;
	cout << "Security swpawning Distance : " << securitySpawningDistance << endl;

	// Clients
	ros::ServiceClient gazebo_spawn_clt= nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

	maplimits*=2; //will be re-divided by 2 after the random selection of the position
	// in order to have a footstep of 0.5m



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
	int nbContainers = 0; // Beware: it's the total nbr of containers spawned since the beginning, including those deleted! The current number of containers is listeContainers.size()
	int signs[2] = {-1, 1};
	vector<Container> listContainers;
	vector<Container> deletedContainers;

	detect_containers(listContainers, nbContainers, mapImagePath);

	// Map generation
    ros::Duration(5).sleep(); // sleep for 10 seconds

	for (int i=0; i<nbContainers; i++) {
		Container new_cont = listContainers[i];

		sm.request.model_name = new_cont.getName();
		sm.request.reference_frame = "world";
		sm.request.initial_pose.position.x = new_cont.getX();
		sm.request.initial_pose.position.y = new_cont.getY();
		sm.request.initial_pose.position.z = 0.0;
		sm.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(new_cont.getYaw());

	  	if (gazebo_spawn_clt.call(sm)) // Call the spawn service
	  		cout << i << ": Initial container spawned !  " << new_cont << endl;
	}

	cout << "Map initialised with success!" << endl;


	//ros::Duration(60).sleep();// Pause 2min
	cout << "Begining life cycling..." << endl;


	// Main Loop
	int actionMode = 1;
	double clearingFactor = 10.0;
    ros::Rate rate(100);

    while (ros::ok())
    {
        double sleeptime = 5.0; // randsleeptime*rand()/RAND_MAX + minsleeptime;
        ros::Duration(sleeptime).sleep(); // sleep for minsleeptime to minsleeptime + randsleeptime seconds
		ros::spinOnce();

        // Random choice of action mode. Can be biased to favorize delete or spawn phases.
        //actionMode = rand()%2;

        // For example, these lines favorize an alternance of "emptying the harbor" and "fill the harbor" phases :

        if ( ( actionMode == 1 ) && ( (double)listContainers.size()/nbContainers < leftoversRatio ) ) // Condition to switch from 1 to 0
		{
        	int randSwitch = rand()%nbContainers;
        	if ( randSwitch > listContainers.size()*clearingFactor )
        		actionMode = 0;
		}

        if ( ( actionMode == 0 ) && ( (double)listContainers.size()/nbContainers > (1-leftoversRatio) ) ) // Condition to switch from 0 to 1
        {
        	int randSwitch = rand()%nbContainers;
        	if ( randSwitch > (nbContainers-listContainers.size()) ) // Utiliser un clearing factor mais sans risquer d'être bloqué par la présence du robot
        		actionMode = 1;
        }


		// Spawn a container
		if (actionMode == 0)
		{
			gazebo_msgs::SpawnModel sm;


			std::ifstream file(urdfPath);
		  	std::string line;

		   	while(!file.eof()) // Parse the contents of the given urdf in a string
			{
			  std::getline(file,line);
			  sm.request.model_xml+=line;
			}
		  	file.close();

			// For a new random container:

			/*
			stringstream ss;
			ss << nbContainers;
			string nbstr = ss.str();

			string _nom = "box"+nbstr;
			nbContainers++;
			sm.request.model_name = _nom;
			sm.request.reference_frame = "world";

			double _x = signs[rand()%2]*(rand()%maplimits /2);
			double _y = signs[rand()%2]*(rand()%maplimits /2);
			double _z = 0.0; // Hauteur de chute lors du spawn
			double _yaw = 0.0;

			sm.request.initial_pose.position.x = _x;
			sm.request.initial_pose.position.y = _y;
			sm.request.initial_pose.position.z = _z;
			sm.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(_yaw);

			Container cont(_x, _y, _z, _yaw, _nom);

			gazebo_spawn_clt.call(sm); // Call the spawn service
		  	listContainers.push_back(cont); // Ajouter ce nouvel objet a une liste des containers...
		  	cout << "Container spawned !  " << cont << endl;
			*/

			// To revive a previously deleted container:


			if (deletedContainers.size() != 0)
			{
				random_device rd;     // only used once to initialise (seed) engine
				mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
				int min = 0;
				int max = deletedContainers.size()-1;
				uniform_int_distribution<int> uni(min,max); // guaranteed unbiased

				int random_index = uni(rng);
				Container revivedCont = deletedContainers[random_index];

				if ( sqrt( pow( revivedCont.getX() + containerLength - robotPose_x, 2 ) + pow( revivedCont.getY() - containerWidth - robotPose_y, 2 ) ) > securitySpawningDistance )
				{
					sm.request.model_name = revivedCont.getName();
					sm.request.reference_frame = "world";

					sm.request.initial_pose.position.x = revivedCont.getX();
					sm.request.initial_pose.position.y = revivedCont.getY();
					sm.request.initial_pose.position.z = 0.0;
					sm.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(revivedCont.getYaw());

					gazebo_spawn_clt.call(sm); // Call the spawn service
				  	listContainers.push_back(revivedCont); // Ajouter ce nouvel objet a une liste des containers...
				  	deletedContainers.erase(deletedContainers.begin() + random_index); // Et le supprimer de la liste des deleted containers
				  	cout << "Container (re)spawned !  " << revivedCont << endl;
				}
				else
				{
					cout << "Container NOT (re)spawned because it could land on the robot...  " << revivedCont << endl;
				}
			}
	  	}


	  	//Delete a container

	  	if (actionMode == 1)
		{
			cout << "Entering actionMode 1" << endl;

		  	gazebo_msgs::DeleteModel dm;

		  	int s = listContainers.size();
		  	if (s != 0)
		  	{
			  	int whoToDelete = rand()%s;
				cout << "Who to delete : " << whoToDelete << endl;
			  	Container dcont = listContainers[whoToDelete];
			  	dm.request.model_name = dcont.getName();

				cout << "Model name : " << dm.request.model_name << endl;

				ros::service::waitForService("/gazebo/delete_model");
				ros::ServiceClient gazebo_delete_clt= nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
				/*string a = "box22";

				gazebo::physics::World world("default");
				world.RemoveModel(dcont.getName());
				cout << "Hdey " <<endl;*/


				if (!gazebo_delete_clt.call(dm))
				  	cout << "Error at delete" << endl;

				cout << "Service called" << endl;

			  	listContainers.erase(listContainers.begin() + whoToDelete); // Supprimer cet objet de la liste des containers...
			  	deletedContainers.push_back(dcont); // Et l'ajouter a la liste des conteneurs effacés
			  	cout << "Container deleted... " << dcont << endl;
		  	}
		}

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");

    return 0;
}
