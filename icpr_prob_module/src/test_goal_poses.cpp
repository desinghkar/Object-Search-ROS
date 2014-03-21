#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <math.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace cv;
ofstream file("heuristicGoals.txt");
Mat A = imread("buff_Env3.png");
geometry_msgs::Pose g;
float phi, theta, si;
clock_t in, fin;
void convertQtoE(Mat& M)
{
	float q0 = M.at<float>(0, 0);
	float q1 = M.at<float>(1, 0);
	float q2 = M.at<float>(2, 0);
	float q3 = M.at<float>(3, 0);

	//		*phi = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
	//		*theta = asin(2*(q0*q2 - q3*q1));
	si = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
}

void getGlobalToMapSystem()
{
	float indexi, indexj;
	indexi = g.position.y;
	indexj = g.position.x;
	cout<<"Level 4 x="<<indexj<<" y="<<indexi<<endl;
	//Level 4-3
	indexi = indexi/0.05;
	indexj = indexj/0.05;
	cout<<"Level 3 x="<<indexj<<" y="<<indexi<<endl;
	//Level 3-2
	indexi = A.rows/2 + indexi;
	indexj = A.cols/2 + indexj;
	cout<<"Level 2 x="<<indexj<<" y="<<indexi<<endl;
	//Level 2-1
	indexi = A.cols - indexi;
	cout<<"Level 1 x="<<indexj<<" y="<<indexi<<endl;


	Mat Q(4, 1, CV_32FC1);
	Q.at<float>(0, 0) = g.orientation.w;
	Q.at<float>(1, 0) = g.orientation.x;
	Q.at<float>(2, 0) = g.orientation.y;
	Q.at<float>(3, 0) = g.orientation.z;

	convertQtoE(Q);
	cout<<"si is "<<si*180/M_PI<<endl;
	cout<<"location is "<<indexi<<" "<<indexj<<endl;
//	table_area_seen =0.05*0.05*PatchTraceAt((int)indexj, (int)indexi, (si*180/M_PI));
	fin = clock()-in;
	double time_now = (double)fin/(double)CLOCKS_PER_SEC;
	file<<indexj<<" "<<indexi<<" "<<si*180/M_PI<<" "<<time_now<<endl;
	in = fin;
	//namedWindow("updatedPmap", 1);
	//imshow("updatePmap", updatedPotentialMap);
	//waitKey(0);
/*	imwrite("updatedProbabilityMap.png", updatedPotentialMap);
	Mat buff(updatedPotentialMap.size(), updatedPotentialMap.type());
	updatedPotentialMap.copyTo(buff);
	stringstream temp;
	string num;
	temp<<num_iterations;
	temp>>num;
	temp.clear();
	circle(buff, cv::Point(indexj, indexi), 3, cv::Scalar(0, 0, 0), -1);
	cv::Point P1(indexj, indexi);
	cv::Point P2(indexj+5*cos(si*M_PI/180), indexi-10*sin(si));

	cv::Point Q1(indexj+1, indexi+1);
	cv::Point Q2(indexj+1+5*cos(si*M_PI/180), indexi+1-10*sin(si));
	line(buff, P1, P2, cv::Scalar(255, 255, 255));
	putText(buff, num, P1, FONT_HERSHEY_SIMPLEX, 70, cv::Scalar(0, 0, 255));
	imwrite("PathTraced.png", buff);
	num_iterations++;
*/
}

void callback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	g.position.x = goal->pose.position.x;
	g.position.y = goal->pose.position.y;
	g.position.z = goal->pose.position.z;
	
	g.orientation.x = goal->pose.orientation.x;
	g.orientation.y = goal->pose.orientation.y;
	g.orientation.z = goal->pose.orientation.z;
	g.orientation.w = goal->pose.orientation.w;
	getGlobalToMapSystem();
}
int main(int argc, char* argv[])
{
	in = clock();
	ros::init(argc, argv, "get_the_goals");
	ros::NodeHandle nh;
	ros::Subscriber getgoal_sub;
//	ros::Time t();
	getgoal_sub = nh.subscribe("/move_base_simple/goal", 1000, callback);
	ros::spin();
	return 0;

}
