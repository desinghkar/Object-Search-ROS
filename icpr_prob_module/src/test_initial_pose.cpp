#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
void setQ(Mat& Q, float phi, float theta, float si)
{
	cout<<"Inside setQ"<<endl;
	Q.at<float>(0,0) = cos(phi/2)*cos(theta/2)*cos(si/2) + sin(phi/2)*sin(theta/2)*sin(si/2);
	Q.at<float>(1,0) = sin(phi/2)*cos(theta/2)*cos(si/2) - cos(phi/2)*sin(theta/2)*sin(si/2);
	Q.at<float>(2,0) = cos(phi/2)*sin(theta/2)*cos(si/2) + sin(phi/2)*cos(theta/2)*sin(si/2);
	Q.at<float>(3,0) = cos(phi/2)*cos(theta/2)*sin(si/2) - sin(phi/2)*sin(theta/2)*cos(si/2);

}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "map_initial_pose");
	ros::NodeHandle nh;
	ros::Publisher initial_pose = nh.advertise<geometry_msgs::PoseStamped>("talk_pose", 1);
	geometry_msgs::PoseStamped p;

	p.header.stamp = ros::Time::now();
	p.header.frame_id = "/map";



	p.pose.position.x = 0;
	p.pose.position.y = 0;
	p.pose.position.z = 0;

	float phi=0, theta=0, si=0;
	Mat Q = Mat::zeros(4, 1, CV_32FC1);
	setQ(Q, phi, theta, si);
	p.pose.orientation.w = Q.at<float>(0, 0);
	p.pose.orientation.x = Q.at<float>(1, 0);
	p.pose.orientation.y = Q.at<float>(2, 0);
	p.pose.orientation.z = Q.at<float>(3, 0);

	while(ros::ok())
	{
		int constant;
		cout<<"Enter for start"<<endl;
		cin>>constant;
		cout<<"Q is "<<endl<<Q<<endl;

		initial_pose.publish(p);
	}
	ros::spin();
	return 0;

}
