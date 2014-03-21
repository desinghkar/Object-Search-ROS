#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"

using namespace std;
using namespace cv;

void print(geometry_msgs::PoseStamped goal)
{
	cout<<"The location is "<<endl;
	cout<<"x = "<<goal.pose.position.x<<" y = "<<goal.pose.position.y<<" z = "<<goal.pose.position.z<<endl;
	cout<<"The orientation is "<<endl;
	cout<<"x = "<<goal.pose.orientation.x<<" y = "<<goal.pose.orientation.y<<" z = "<<goal.pose.orientation.z<<" w = "<<goal.pose.orientation.w<<endl;
}
void setQ(Mat& Q, float phi, float theta, float si)
{
	Q.at<float>(0,0) = cos(phi/2)*cos(theta/2)*cos(si/2) + sin(phi/2)*sin(theta/2)*sin(si/2);
	Q.at<float>(1,0) = sin(phi/2)*cos(theta/2)*cos(si/2) - cos(phi/2)*sin(theta/2)*sin(si/2);
	Q.at<float>(2,0) = cos(phi/2)*sin(theta/2)*cos(si/2) + sin(phi/2)*cos(theta/2)*sin(si/2);
	Q.at<float>(3,0) = cos(phi/2)*cos(theta/2)*sin(si/2) - sin(phi/2)*sin(theta/2)*cos(si/2);

}
int main(int argc, char * argv[])
{
	ros::init(argc, argv, "goal_test");
	ros::NodeHandle nh;
	ros::Publisher goal_pub;
	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

	geometry_msgs::PoseStamped goal;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "/map";



	int k=0;
	int R=255, B=255, G=255;

	if(argc<2)
	{
		cout<<"Please enter the image of the map as an argument"<<endl;
		return 0;
	}
	Mat image = imread(argv[1], 1);
	Mat Q(4, 1, CV_32FC1);
	float bx=M_PI/2, by=M_PI/2, bz = 0;
	float phi = 0;
	float theta = 0;
	float si=0;
	cout<<"Enter the value of k"<<endl;
	cin>>k;

//	float az=0, aw=0;
	while(k<4)
	{
		switch(k)
		{
			case 0: R=255; G=0; B=0; si=0; setQ(Q, phi, theta, si);// az = -0.0101; aw = 0.999;
				break;
			case 1: R=0; G=0; B=255; si=-(M_PI/6); setQ(Q, phi, theta, si);//az = -0.386; aw = 0.9222;
				break;
			case 2: R=0; G=255; B=255; si= 210*M_PI/180; setQ(Q, phi, theta, si);//az = 0.964; aw = -0.2625;
				break; 
			case 3: R=255; G=0; B=255; si= 150*M_PI/180; setQ(Q, phi, theta, si);//az = 0.8646; aw = 0.5024;
				break;
		}
		
		for(int i=0; i<image.cols; i++)
			for(int j=0; j<image.rows; j++)
			{
				if(image.at<cv::Vec3b>(i, j)[0] == B&& image.at<cv::Vec3b>(i, j)[1]==G&& image.at<cv::Vec3b>(i, j)[2]==R)
				{
					float x = j;
					float y = i;
					float z = 0;
					cout<<"Level 1 "<<x<<" "<<y<<endl;
					y = image.cols - y;
					cout<<"Level 2 "<<x<<" "<<y<<endl;
					x = x-image.rows/2;
					y = y-image.cols/2;
					cout<<"Level 3 "<<x<<" "<<y<<endl;
					x = 0.05*x;
					y = 0.05*y;
					cout<<"Level 4 "<<x<<" "<<y<<endl;
					goal.pose.position.x = x;
					goal.pose.position.y = y;
					goal.pose.position.z = z;
					goal.pose.orientation.w = Q.at<float>(0, 0);
					goal.pose.orientation.x = Q.at<float>(1, 0);
					goal.pose.orientation.y = Q.at<float>(2, 0);
					goal.pose.orientation.z = Q.at<float>(3, 0);
					print(goal);

					goal_pub.publish(goal);
				}	

				
			}
	
		
		cout<<"Enter the value of k"<<endl;
		cin>>k;
	}
	ros::spin();
	return 0;

}
