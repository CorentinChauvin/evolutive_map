/*
 * Source code for the detection of containers on an image
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "Container.h"

// Namespaces
using namespace std;
using namespace cv;


int detect_containers(vector<Container>& listCont, int& nbContainers, string imagePath)
{
	/* 	Function reading an image and creating a container list from points on the image

		If imagePath begins by "/", its an absolute path, else it is the relative path in the package "evolutive_map"
	*/

	//Open the image
	if (imagePath[0] != '/')
		imagePath = ros::package::getPath("evolutive_map") + "/" + imagePath;

	Mat image;
	image = imread(imagePath, CV_LOAD_IMAGE_COLOR); //open in colours


	if(! image.data )// Check for invalid input
	{
        	cout <<  "ERROR: Could not open or find the image" << std::endl;
        	return -1;
	}

	double img_scale = 10.0/35.0; //pixels to meters- no real influence in the end
	double biased_scale = 2.0/(20*img_scale);//real meters to biased meters
	// 2m/len_container, with: len_container = nb_pix_container_in_the_picture*img_scale

	for(int y=0;y<image.rows;y++)
	{
		for(int x=0;x<image.cols;x++)
		{
		    Vec3b color = image.at<Vec3b>(Point(x,y));
		    if(color[0] < 150 || color[1] < 150 || color[2] < 150)
		    {
		        bool new_cont_detected = false;
		        double yaw = 0.0;

		        //check for neighbour:
		        try{
				Vec3b neighb_1 = image.at<Vec3b>(Point(x+1,y));
				if(neighb_1[0] < 150 || neighb_1[1] < 150 || neighb_1[2] < 150)
				{
					new_cont_detected = true;
					yaw = 0.0;
				}
		    	} catch(exception const& e){}

		        try{
				Vec3b neighb_2 = image.at<Vec3b>(Point(x+1,y+1));
				if(neighb_2[0] < 150 || neighb_2[1] < 150 || neighb_2[2] < 150)
				{
					new_cont_detected = true;
					yaw = -M_PI/4;
				}
		    	} catch(exception const& e){}

		        try{
				Vec3b neighb_3 = image.at<Vec3b>(Point(x,y+1));
				if(neighb_3[0] < 150 || neighb_3[1] < 150 || neighb_3[2] < 150)
				{
					new_cont_detected = true;
					yaw = -M_PI/2;
				}
		    	} catch(exception const& e){}

		    	try{
				Vec3b neighb_4 = image.at<Vec3b>(Point(x-1,y+1));
				if(neighb_4[0] < 150 || neighb_4[1] < 150 || neighb_4[2] < 150)
				{
					new_cont_detected = true;
					yaw = -3*M_PI/4;
				}
		    	} catch(exception const& e){}

		    	if (new_cont_detected)
		    	{
				double x_m = (x - image.cols/2) * img_scale * biased_scale;//because of the real map being larger
				double y_m = (image.rows/2 - y) * img_scale * biased_scale;//same
				double yaw_m = yaw;
				cout << "Pixel found :" << x_m << " ; " << y_m << " ; " << yaw_m << endl;

				stringstream ss;
				ss << nbContainers;
				string nbstr = ss.str();

				string _nom = "box"+nbstr;
				Container new_cont(x_m, y_m, 0.0, yaw_m, _nom);

				listCont.push_back(new_cont);
				nbContainers++;
		        }

		    }
		 }
	}
	return 0;
}
