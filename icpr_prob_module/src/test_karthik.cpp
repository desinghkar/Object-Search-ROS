#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>

#define RANGE 30
#define SCALE 10
using namespace std;
using namespace cv;
int kcount =0;
int row = 0;
string image_path;
class Karthik_map
{
	ros::NodeHandle nh;
	ros::Publisher goal_pub;
	ros::Subscriber currentloc_sub;
	geometry_msgs::PoseStamped goal;
	geometry_msgs::Pose current;
	
	public:
	Karthik_map()
	{
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "/map";

		goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
		cout<<"In potential_map"<<endl;
		currentloc_sub = nh.subscribe("talk_pose", 1, &Karthik_map::updateProb, this);


	}
	void updateProb(const geometry_msgs::PoseStamped::ConstPtr& loc)
	{
		cout<<"Inside the updateProb"<<endl;
		current.position.x = loc->pose.position.x;
		current.position.y = loc->pose.position.y;
		current.position.z = loc->pose.position.z;

		current.orientation.x = loc->pose.orientation.x;
		current.orientation.y = loc->pose.orientation.y;
		current.orientation.z = loc->pose.orientation.z;
		current.orientation.w = loc->pose.orientation.w;

		goal.pose.position.x = current.position.x;
		goal.pose.position.y = current.position.y;
		goal.pose.position.z = current.position.z;

		goal.pose.orientation.x = current.orientation.x;
		goal.pose.orientation.y = current.orientation.y;
		goal.pose.orientation.z = current.orientation.z;
		goal.pose.orientation.w = current.orientation.w;
		
		goal_pub.publish(goal);
		
	}
};	
int main(int argc, char* argv[])
{

	Mat potential_map;
	if(argc<2)
	{
		cout<<"Enter the potential map path with the binary"<<endl;
		exit(0);
	}
		
	ros::init(argc, argv, "probability_update");
	image_path = argv[1];
	Karthik_map P();
	cout<<"Spinning"<<endl;

	ros::spin();
}
