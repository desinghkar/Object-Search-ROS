#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#define RANGE 45
#define SCALE 10
using namespace std;
using namespace cv;
ofstream loc("locations_of_objects.txt");
int num_iterations=0;
int kcount=0;
int seen[10]={0,0,0,0,0,0,0,0,0,0};
string image_path1;
string image_path2;
string path;
class Potential_update
{
	ros::NodeHandle nh;
	ros::Subscriber goal_sub;
	ros::Publisher goal_pub;
//	ros::Publisher marker_pub;
	geometry_msgs::Pose g;
	geometry_msgs::PoseStamped next;
	double indexi, indexj, si;
	double table_area_seen;
	int item_no;

//	Mat updatedPotentialMap;
	Mat originalPotentialMap;

	public:
	Potential_update()
	{
//		updatedPotentialMap =imread(image_path1);

		cout<<"In potentialupdate..."<<endl;

		goal_sub = nh.subscribe("/move_base_simple/goal", 1, &Potential_update::updateMap, this);
		goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ourgoal", 1);
//		marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/potentialloc", 1);

//		updatePotentialMap(updatedPotentialMap);
	}
	~Potential_update(){}
//	Potential_update(){}
	void updateMap(const geometry_msgs::PoseStamped::ConstPtr& goal)
	{
		cout<<"Inside the potential update with image "<<endl;	
		if(kcount<1)
			path = image_path1;
		else
			path = image_path2;
		cout<<"Image being used is "<<path<<endl;
		originalPotentialMap = imread(path);
		cout<<"Enter to update the map and set the goal"<<endl;
		int k;
		cin>>k;
		next.header.stamp = ros::Time::now();
		next.header.frame_id = "/map";
		next.pose.position.x = g.position.x = goal->pose.position.x;
		next.pose.position.y = g.position.y = goal->pose.position.y;
		next.pose.position.z = g.position.z = goal->pose.position.z;

		next.pose.orientation.x = g.orientation.x = goal->pose.orientation.x;
		next.pose.orientation.y = g.orientation.y = goal->pose.orientation.y;
		next.pose.orientation.z = g.orientation.z = goal->pose.orientation.z;
		next.pose.orientation.w = g.orientation.w = goal->pose.orientation.w;


		getGlobalToMapSystem();
		goal_pub.publish(next);
		kcount++;
	}
	void getGlobalToMapSystem()
	{
		indexi = g.position.y;
		indexj = g.position.x;
//		cout<<"Level 4 x="<<indexj<<" y="<<indexi<<endl;
		//Level 4-3
		indexi = indexi/0.05;
		indexj = indexj/0.05;
//		cout<<"Level 3 x="<<indexj<<" y="<<indexi<<endl;
		//Level 3-2
		indexi = originalPotentialMap.rows/2 + indexi;
		indexj = originalPotentialMap.cols/2 + indexj;
//		cout<<"Level 2 x="<<indexj<<" y="<<indexi<<endl;
		//Level 2-1
		indexi = originalPotentialMap.cols - indexi;
//		cout<<"Level 1 x="<<indexj<<" y="<<indexi<<endl;

		double phi, theta;
		Mat Q(4, 1, CV_32FC1);
		Q.at<double>(0, 0) = g.orientation.w;
		Q.at<double>(1, 0) = g.orientation.x;
		Q.at<double>(2, 0) = g.orientation.y;
		Q.at<double>(3, 0) = g.orientation.z;
		cout<<"Q is "<<endl<<Q<<endl;
		si = convertQtoE(Q);
		cout<<"si is "<<si<<endl;
		cout<<"location is "<<indexi<<" "<<indexj<<endl;
		table_area_seen =0.05*0.05*PatchTraceAt((int)indexj, (int)indexi, (si*180/M_PI));

		//namedWindow("updatedPmap", 1);
		//imshow("updatedPmap", updatedPotentialMap);
		//waitKey(0);
		imwrite("updatedProbabilityMap.png", originalPotentialMap);

		//updatePotentialCloud(originalPotentialMap);
	/*	
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
		stringstream temp2;
		string p1 = "PathTracedAt";
		temp2<<num_iterations;
		string p2;
		temp2>>p2;
		string path  = p1+p2+".png";
		imwrite("PathTraced.png", buff);
	*/	
		//namedWindow("pathtraced", 1);
		//imshow("pathtraced", buff);
		//waitKey(0);
		num_iterations++;

	}
	void updatePotentialCloud(Mat& M)
	{
			
		visualization_msgs::MarkerArray MA;
		int id=0;
		for(int i=0; i<M.rows; i++)
			for(int j=0; j<M.cols; j++)
			{
				if(isGreen(M, i, j)||isPink(M, i, j))
				{
					visualization_msgs::Marker m;
					uint32_t shape = visualization_msgs::Marker::CUBE;
					m.header.frame_id = "/map";
					m.header.stamp = ros::Time::now();
//					m.ns = "basic_shapes";
					m.id = id;
					m.type = shape;
					m.action = visualization_msgs::Marker::ADD;
					//convert the map.x, map.y to geometry_msgs.x and geometry_msgs.y
					geometry_msgs::Pose g = setLocalToGlobal(Point(j-5, i+5), 0, M.rows, M.cols);
					m.pose.position.x = g.position.x;
					m.pose.position.y = g.position.y;
					m.pose.position.z = g.position.z;
					m.pose.orientation.x = g.orientation.x;
					m.pose.orientation.y = g.orientation.y;
					m.pose.orientation.z = g.orientation.z;
					m.pose.orientation.w = g.orientation.w;

					m.scale.x = 0.05;
					m.scale.y = 0.05;
					m.scale.z = 0.001;
					
					m.color.r = 1.0*(isGreen(M, i, j));
					m.color.g = 1.0*(isPink(M, i, j));
					m.color.b = 0.0f;
					m.color.a = 1.0f;
					

					m.lifetime = ros::Duration();
					MA.markers.push_back(m);
					++id;
				}

			}
		//marker_pub.publish(MA);
	}

	geometry_msgs::Pose setLocalToGlobal(Point P, double th, int m_rows, int m_cols)
	{
		geometry_msgs::Pose gl;
		gl.position.x = P.x; gl.position.y = P.y; gl.position.z = 0.0;
//		cout<<"Level 1 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
		gl.position.y = m_cols - gl.position.y;
//		cout<<"Level 2 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
		gl.position.x = gl.position.x - m_rows/2;
		gl.position.y = gl.position.y - m_cols/2;
//		cout<<"Level 3 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
		gl.position.x*=0.05;
		gl.position.y*=0.05;
//		cout<<"Level 4 at "<<gl.position.x<<" , "<<gl.position.y<<endl;

		Mat Q(4, 1, CV_64F);
		double pi=0, theta=0;
		double si = th*M_PI/180;
/*
		convertEtoQ(Q, pi, theta, si);

		geometry_msgs::Pose global_location;
		global_location.position.x = gl.position.x;
		global_location.position.y = gl.position.y;
		global_location.position.z = gl.position.z;

		global_location.orientation.w = Q.at<double>(0, 0);
		global_location.orientation.x = Q.at<double>(1, 0);
		global_location.orientation.y = Q.at<double>(2, 0);
		global_location.orientation.z = Q.at<double>(3, 0);
*/
		return gl;
	}
	bool isGreen(Mat M, int i, int j)
	{
		if(M.at<Vec3b>(i, j)[0]==0 && M.at<Vec3b>(i, j)[1]==255 && M.at<Vec3b>(i, j)[2]==0)
			return true;
		return false;
	}
	bool isPink(Mat M, int i, int j)
	{
		if(M.at<Vec3b>(i, j)[0]==255 && M.at<Vec3b>(i, j)[1]==0 && M.at<Vec3b>(i, j)[2]==255)
			return true;
		return false;
	}
/*	void convertEtoQ(Mat& Q, double pi, double theta, double si)
	{
		Q.at<double>(0,0) = cos(pi/2)*cos(theta/2)*cos(si/2) + sin(pi/2)*sin(theta/2)*sin(si/2);
		Q.at<double>(1,0) = sin(pi/2)*cos(theta/2)*cos(si/2) - cos(pi/2)*sin(theta/2)*sin(si/2);
		Q.at<double>(2,0) = cos(pi/2)*sin(theta/2)*cos(si/2) + sin(pi/2)*cos(theta/2)*sin(si/2);
		Q.at<double>(3,0) = cos(pi/2)*cos(theta/2)*sin(si/2) - sin(pi/2)*sin(theta/2)*cos(si/2);

	}
*/	double convertQtoE(Mat& M)
	{
		double q0 = M.at<double>(0, 0);
		double q1 = M.at<double>(1, 0);
		double q2 = M.at<double>(2, 0);
		double q3 = M.at<double>(3, 0);

		double 	local_orientation = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
	//	local_orientation = local_orientation*180/M_PI;
		cout<<"Local orientation computed is "<<local_orientation*180/M_PI<<endl;
		return local_orientation;
	}

	int PatchTraceAt(int x, int y, double theta)
	{
		cout<<"In PatchTrace"<<endl;
		double min_theta =(theta - 45);
		double max_theta =(theta + 45);
		int count = 0;
		double s = max_theta;
		for(double s=min_theta; s<max_theta; s++)
		{

			int x2 = x + RANGE*cos(s*M_PI/180);
			int y2 = y - RANGE*sin(s*M_PI/180);
			int x1 = x;
			int y1 = y;

			count+=rayTrace2D(x1, y1, x2, y2);

		}
		cout<<"Count at this location is "<<count<<endl;
		return count;
	}
	int rayTrace2D(int x1, int y1, int x2, int y2)
	{
		//		cout<<"In rayTrace2D"<<endl;
		double distance = dist(x1, y1, x2, y2);
		//		if(distance>RANGE)
		//			return 0;
		int steps = (int)floor(distance)+1;
		double x_step = 1.0*(x2-x1)/steps;
		double y_step = 1.0*(y2-y1)/steps;
		int B, G, R;	
		int count=0;
		double x = x1, y = y1;
		int xi1, xi2, yi1, yi2;
		int vector = 0;
		double dx, dy; //bool temp = false;
		while(++vector <=steps)
		{
			xi1 = (int)x; xi2 = (xi1<originalPotentialMap.cols - 1)? xi1+1: xi1;
			yi1 = (int)y; yi2 = (yi1<originalPotentialMap.rows - 1)? yi1+1: yi1;

			dx = x - xi1; dy = y - yi1;
			int kx, ky;
			if(dx < 0.5)
				if(dy < 0.5) {
					B = originalPotentialMap.at<Vec3b>(yi1, xi1)[0]; 
					G = originalPotentialMap.at<Vec3b>(yi1, xi1)[1];
					R = originalPotentialMap.at<Vec3b>(yi1, xi1)[2];
					kx = xi1;
					ky = yi1;
				}
				else         {
					B = originalPotentialMap.at<Vec3b>(yi2, xi1)[0]; 
					G = originalPotentialMap.at<Vec3b>(yi2, xi1)[1];
					R = originalPotentialMap.at<Vec3b>(yi2, xi1)[2];
					kx = xi1;
					ky = yi2;
				}
			else
				if(dy < 0.5) {
					B = originalPotentialMap.at<Vec3b>(yi1, xi2)[0]; 
					G = originalPotentialMap.at<Vec3b>(yi1, xi2)[1];
					R = originalPotentialMap.at<Vec3b>(yi1, xi2)[2];
					kx = xi2;
					ky = yi1;
				}
				else {
					B = originalPotentialMap.at<Vec3b>(yi2, xi2)[0]; 
					G = originalPotentialMap.at<Vec3b>(yi2, xi2)[1];
					R = originalPotentialMap.at<Vec3b>(yi2, xi2)[2];
					kx = xi2;
					ky = yi2;
				}
#if 0
			updatedPotentialMap.at<Vec3b>(ky, kx)[0] = 255;
			updatedPotentialMap.at<Vec3b>(ky, kx)[1] = 0;=
			updatedPotentialMap.at<Vec3b>(ky, kx)[2] = 255;
#endif
			if(B==0&&G==255&&R==0)
			{
				count++;
//				cout<<"Making pink"<<endl;
				originalPotentialMap.at<Vec3b>(ky, kx)[0] = 255;
				originalPotentialMap.at<Vec3b>(ky, kx)[1] = 0;
				originalPotentialMap.at<Vec3b>(ky, kx)[2] = 255;
				
				
				//				cout<<"Green "<<count<<endl;
			}
			
			if(object(ky, kx))
			{
				loc<<x1<<" "<<y1<<" "<<item_no<<" "<<kcount<<endl;
			}
			else if(B==0&&G==0&&R==0)
				return count;
			x+= x_step;
			y+= y_step;


		}
		return count;

	}
	bool object(int i, int j)
	{
		int B = originalPotentialMap.at<Vec3b>(i, j)[0]; 
		int G = originalPotentialMap.at<Vec3b>(i, j)[1];
		int R = originalPotentialMap.at<Vec3b>(i, j)[2];

		for(int i=25; i<255; i+=25)
		{
			if(B==0&&G==0&&R==i)
			{
				item_no = i/25;
				if(seen[item_no-1]==0)
				{
					seen[item_no-1]=1;
					return true;
				}

			}
		}
		return false;
		
	}
	double dist(int a1, int b1, int a2, int b2)
	{
		double d = 1.0*(a1-a2)*(a1-a2) + 1.0*(b1-b2)*(b1-b2);
		return sqrt(d);
	}

};

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "Potential_map_update");
	if(argc<3)
	{
		cout<<"Pls enter the path of the original potential map and the update map"<<endl;
		return 0;
	}
	image_path1 = argv[1];
	image_path2 = argv[2];
	Potential_update P_up();
	cout<<"Spinning..."<<endl;
	ros::spin();
}
