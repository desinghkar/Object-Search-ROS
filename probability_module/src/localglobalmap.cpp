#include "localglobalmap/localglobalmap.h"
#define MAP_SCALE 0.05

bool LocalGlobalMap::readMap(string inpath)
{
	cout<<"Inside the LocalGlobalMap ..."<<endl;
	map = imread(inpath);
	if(map.data == NULL)
	{
		cout<<"File not found at "<<inpath<<endl;
		return false;
	}
	m_rows = map.rows;
	m_cols = map.rows;
	m_size = map.size();
	m_type = map.type();
	return true;
}
void LocalGlobalMap::setGlobalToLocal(geometry_msgs::Pose P)
{ 
	double indexi, indexj;
	indexi = P.position.y;
	indexj = P.position.x;

	//Level 4-3
	indexi = indexi/MAP_SCALE;
	indexj = indexj/MAP_SCALE;

	//Level 3-2
	indexi = m_rows/2 + indexi;
	indexj = m_cols/2 + indexj;

	//Level 2-1
	indexi = m_cols - indexi;
	
	local_location.y = (int) indexi;
	local_location.x = (int) indexj;
	cout<<"indexi = "<<indexi<<" and indexj = "<<indexj<<endl;

	Mat Q(4, 1, CV_64F);
	Q.at<double>(0, 0) = P.orientation.w;
	Q.at<double>(1, 0) = P.orientation.x;
	Q.at<double>(2, 0) = P.orientation.y;
	Q.at<double>(3, 0) = P.orientation.z;

	convertQtoE(Q);
	
	cout<<"Local orientation is "<<local_orientation*180/M_PI<<endl;

} 


void LocalGlobalMap::setLocalToGlobal(Point P, double th)
{
	geometry_msgs::Pose gl;
	gl.position.x = P.x; gl.position.y = P.y; gl.position.z = 0.0;
//	cout<<"Level 1 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
	//Level 1-2
	gl.position.y = m_cols - gl.position.y;
//	cout<<"Level 2 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
	//Level 2-3
	gl.position.x = gl.position.x - m_rows/2;
	gl.position.y = gl.position.y - m_cols/2;
//	cout<<"Level 3 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
	//Level 3-4
	gl.position.x*=MAP_SCALE;
	gl.position.y*=MAP_SCALE;
//	cout<<"Level 4 at "<<gl.position.x<<" , "<<gl.position.y<<endl;

	Mat Q(4, 1, CV_64F);
	double pi=0, theta=0;
	double si = th*M_PI/180;

	convertEtoQ(Q, pi, theta, si);

	global_location.position.x = gl.position.x;
	global_location.position.y = gl.position.y;
	global_location.position.z = gl.position.z;
	
	global_location.orientation.w = Q.at<double>(0, 0);
	global_location.orientation.x = Q.at<double>(1, 0);
	global_location.orientation.y = Q.at<double>(2, 0);
	global_location.orientation.z = Q.at<double>(3, 0);
}

void LocalGlobalMap::convertQtoE(Mat& M)
{
	double q0 = M.at<double>(0, 0);
	double q1 = M.at<double>(1, 0);
	double q2 = M.at<double>(2, 0);
	double q3 = M.at<double>(3, 0);

	local_orientation = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
	local_orientation = local_orientation*180/M_PI;
}

void LocalGlobalMap::convertEtoQ(Mat& Q, double pi, double theta, double si)
{
	Q.at<double>(0,0) = cos(pi/2)*cos(theta/2)*cos(si/2) + sin(pi/2)*sin(theta/2)*sin(si/2);
	Q.at<double>(1,0) = sin(pi/2)*cos(theta/2)*cos(si/2) - cos(pi/2)*sin(theta/2)*sin(si/2);
	Q.at<double>(2,0) = cos(pi/2)*sin(theta/2)*cos(si/2) + sin(pi/2)*cos(theta/2)*sin(si/2);
	Q.at<double>(3,0) = cos(pi/2)*cos(theta/2)*sin(si/2) - sin(pi/2)*sin(theta/2)*cos(si/2);

}

geometry_msgs::Pose LocalGlobalMap::getGlobal()
{
	return global_location;
}

cv::Point LocalGlobalMap::getLocalPosition()
{
	return local_location;
}

double LocalGlobalMap::getLocalOrientation()
{
	return local_orientation;
}


//ros::Publisher goal_pub;
string path_;


/*	
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& loc)
{
	LocalGlobalMap LG;
	if(!LG.readMap(path_))
		return;
	geometry_msgs::Pose po;
	po.position.x = loc->pose.pose.position.x;
	po.position.y = loc->pose.pose.position.y;
	po.position.z = loc->pose.pose.position.z;
	po.orientation.x = loc->pose.pose.orientation.x;
	po.orientation.y = loc->pose.pose.orientation.y;
	po.orientation.z = loc->pose.pose.orientation.z;
	po.orientation.w = loc->pose.pose.orientation.w;
//	LG.setLocalToGlobal(Point(170, 100), 90);
	LG.setGlobalToLocal(po);
}
int main(int ac, char** av)
{
	path_ = av[1];
	ros::init(ac, av, "prob_update");
	ros::NodeHandle nh;

	ros::Subscriber cur_sub = nh.subscribe("/initialpose", 1, &callback);
	cout<<"spinning"<<endl;

	ros::spin();
	
//	return 0;
}
*/
