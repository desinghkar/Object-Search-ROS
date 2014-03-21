#include "just_vp_prob/just_vp_prob.h"
#include "just_time/just_time.h"
#include "fuseProbabilities/fuseProbabilities.h"
#include "localglobalmap/localglobalmap.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

string image_path1;
string image_path2;
string path;
string files_outpath1 = "/workspace/karthik_2/ros_workspace/probability_module/vp_prob/";
string files_outpath2 = "/workspace/karthik_2/ros_workspace/probability_module/time_prob/";
string files_outpath3 = "/workspace/karthik_2/ros_workspace/probability_module/result_prob/";
ros::Publisher pub;
int iter=0;
void callback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& loc)
{
	if(iter>0)
		path = image_path2;
	else
		path = image_path1;
	iter++;
	cout<<"Received the initialpose and path is "<<path<<endl;
	LocalGlobalMap LG;
	if(!LG.readMap(path))
		return;
	//Get the geometry_msgs::PoseWithCovarianceStamped to geometry_msgs::Pose
	geometry_msgs::Pose c;
	c.position.x = loc->pose.pose.position.x;
	c.position.y = loc->pose.pose.position.y;
	c.position.z = loc->pose.pose.position.z;
	c.orientation.x = loc->pose.pose.orientation.x;
	c.orientation.y = loc->pose.pose.orientation.y;
	c.orientation.z = loc->pose.pose.orientation.z;
	c.orientation.w = loc->pose.pose.orientation.w;

	//Now get the Global to Local conversion
	LG.setGlobalToLocal(c);
	cout<<"Now at "<<c.position.x<<" , "<<c.position.y<<" , "<<c.position.z<<endl;
	cout<<"with orientation "<<c.orientation.x<<" , "<<c.orientation.y<<" , "<<c.orientation.z<<" ,  "<<c.orientation.w<<endl;
	cout<<"Given initialpose is at "<<LG.getLocalPosition().x<<" , "<<LG.getLocalPosition().y<<" and angle "<<LG.getLocalOrientation()<<endl;


	//Get the vp prob updated
	MapViewProb VP(path, files_outpath1);
	//Get the time prob updated
	MapTimeProb TP(path, files_outpath2, LG.getLocalPosition(), LG.getLocalOrientation());
	cout<<"Given initialpose is at "<<LG.getLocalPosition().x<<" , "<<LG.getLocalPosition().y<<" and angle "<<LG.getLocalOrientation()<<endl;
	//Fuse them
	Mat im = imread(path);
	FuseProb FP(VP.getResultProb(), TP.getResultTime(), im, 8, files_outpath3);
	FP.computeGoal();
	cout<<"Computed goal to be at "<<FP.getGoalPosition().x<<" , "<<FP.getGoalPosition().y<<" and angle "<<FP.getGoalOrientation()<<endl;
	LG.setLocalToGlobal(FP.getGoalPosition(), FP.getGoalOrientation());

	geometry_msgs::Pose n;
	n = LG.getGlobal();
	geometry_msgs::PoseStamped next;
	next.header.stamp = ros::Time::now();
	next.header.frame_id = "/map";
	next.pose.position.x = n.position.x;
	next.pose.position.y = n.position.y;
	next.pose.position.z = n.position.z;
	next.pose.orientation.x = n.orientation.x;
	next.pose.orientation.y = n.orientation.y;
	next.pose.orientation.z = n.orientation.z;
	next.pose.orientation.w = n.orientation.w;
	cout<<"Publishing the goal..."<<endl;

	pub.publish(next);

}
void callback2(const geometry_msgs::PoseStamped::ConstPtr& loc)
{
	path = image_path2;
	iter++;
	cout<<"Received the goal and path is "<<path<<endl;
	LocalGlobalMap LG;
	if(!LG.readMap(path))
		return;
	//Get the geometry_msgs::PoseWithCovarianceStamped to geometry_msgs::Pose
	geometry_msgs::Pose c;
	c.position.x = loc->pose.position.x;
	c.position.y = loc->pose.position.y;
	c.position.z = loc->pose.position.z;
	c.orientation.x = loc->pose.orientation.x;
	c.orientation.y = loc->pose.orientation.y;
	c.orientation.z = loc->pose.orientation.z;
	c.orientation.w = loc->pose.orientation.w;

	//Now get the Global to Local conversion
	LG.setGlobalToLocal(c);
	cout<<"Now at "<<c.position.x<<" , "<<c.position.y<<" , "<<c.position.z<<endl;
	cout<<"with orientation "<<c.orientation.x<<" , "<<c.orientation.y<<" , "<<c.orientation.z<<" ,  "<<c.orientation.w<<endl;
	cout<<"Given goal is at "<<LG.getLocalPosition().x<<" , "<<LG.getLocalPosition().y<<" and angle "<<LG.getLocalOrientation()<<endl;


	//Get the vp prob updated
	MapViewProb VP(path, files_outpath1);
	//Get the time prob updated
	MapTimeProb TP(path, files_outpath2, LG.getLocalPosition(), LG.getLocalOrientation());
	cout<<"Given goal is at "<<LG.getLocalPosition().x<<" , "<<LG.getLocalPosition().y<<" and angle "<<LG.getLocalOrientation()<<endl;
	//Fuse them
	Mat im = imread(path);
	FuseProb FP(VP.getResultProb(), TP.getResultTime(), im, 8, files_outpath3);
	FP.computeGoal();
	cout<<"Computed goal to be at "<<FP.getGoalPosition().x<<" , "<<FP.getGoalPosition().y<<" and angle "<<FP.getGoalOrientation()<<endl;
	LG.setLocalToGlobal(FP.getGoalPosition(), FP.getGoalOrientation());

	geometry_msgs::Pose n;
	n = LG.getGlobal();
	geometry_msgs::PoseStamped next;
	next.header.stamp = ros::Time::now();
	next.header.frame_id = "/map";
	next.pose.position.x = n.position.x;
	next.pose.position.y = n.position.y;
	next.pose.position.z = n.position.z;
	next.pose.orientation.x = n.orientation.x;
	next.pose.orientation.y = n.orientation.y;
	next.pose.orientation.z = n.orientation.z;
	next.pose.orientation.w = n.orientation.w;
	cout<<"Publishing the goal..."<<endl;

	pub.publish(next);
}
int main(int ac, char** av)
{
	//Subscribe for /initialpose to start the callback on both time vp probs
	//Publisher for /move_base/goal to move towards the goal
	//In callback function get the 
	//MapViewProb - This will give the vp based prob with the current map
	//MapTimeProb - This will give the time based prob with the current location and current map
	//FuseProb    - This will fuse the vp and time based prob and decide the goal
	//LocalGlobal - This will take the goal from fuseprob and get the global goal
	//              This will also take the initial pose and convert into local pose 

	if(ac<3)
	{
		cout<<"Please enter the rosrun <package_name> <path_to_original_image> <path_to_partial_explored_image>"<<endl;
		return -1;
	}
	image_path1 = av[1];
	image_path2 = av[2];

	ros::init(ac, av, "probability_update");
	ros::NodeHandle nh;
	ros::Subscriber sub1 = nh.subscribe("/initialpose", 1000, callback1);
	ros::Subscriber sub2 = nh.subscribe("/ourgoal", 1000, callback2);
	pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	cout<<"Spinning..."<<endl;
	ros::spin();
	
	
	
	return 0;
}

