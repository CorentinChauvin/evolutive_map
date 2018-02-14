#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Container.h"

// Constants
#define PI 3.14159265359

using namespace std;
using namespace cv;

int detect_containers(vector<Container>& listCont, int& nbContainers)
{
	//Open an image
	string path = "evolutive_map/ressources/aerial_views/St_Nazaire_1.jpg";//argument?
	Mat image;
	image = imread(path, CV_LOAD_IMAGE_COLOR); //open in colours
	
	
	if(! image.data )// Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl;
        return -1;
    }
    
	double img_scale = 10.0/35.0; //pixels to meters
	double biased_scale = 2.0/(20*img_scale);//real meters to biased meters
	// 2m/long_container avec long_container = pix_cont*img_scale
	
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
						yaw = -PI/4;
					}
		    	} catch(exception const& e){}
		    	
		        try{
				    Vec3b neighb_3 = image.at<Vec3b>(Point(x,y+1));
				    if(neighb_3[0] < 150 || neighb_3[1] < 150 || neighb_3[2] < 150)
					{
						new_cont_detected = true;
						yaw = -PI/2;
					}
		    	} catch(exception const& e){}
		    	
		    	try{
				    Vec3b neighb_4 = image.at<Vec3b>(Point(x-1,y+1));
				    if(neighb_4[0] < 150 || neighb_4[1] < 150 || neighb_4[2] < 150)
					{
						new_cont_detected = true;
						yaw = -3*PI/4;
					}
		    	} catch(exception const& e){}
		    	
		    	if (new_cont_detected)
		    	{
				    //cout << "Pixel found :" << x << "," << y << endl;
				    double x_m = (x - image.cols/2)*img_scale*biased_scale;//map reelle trop grande
				    double y_m = (image.rows/2 - y)*img_scale*biased_scale;//idem
				    double yaw_m = yaw;
				    cout << "Pixel found :" << x_m << " ; " << y_m << " ; " << yaw_m << endl;
				    
				    stringstream ss;
					ss << nbContainers;
					string nbstr = ss.str();
	
					string _nom = "box"+nbstr;
					nbContainers ++;
				
				    Container new_cont(x_m, y_m, 0.0, yaw_m, _nom);
				    
				    listCont.push_back(new_cont);
		        }
		        
		    }
		 }
    }
	
	
	/*imshow("St_Nazaire_1", image);
	waitKey(0);*/
	
	return 0;
}
