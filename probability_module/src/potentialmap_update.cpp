#include "potentialmap_update/potentialmap_update.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#define RANGE 30
#define ANGLE 27
string path_1, path_2;
ros::Publisher marker_pub;
ros::Publisher goal_pub;
PotentialMap::PotentialMap(string path)
{
	if(readImage(originalmap, path))
	{
		m_rows = originalmap.rows;
		m_cols = originalmap.cols;
		m_type = originalmap.type();
		m_size = originalmap.size();

	}
}

bool PotentialMap::readImage(Mat& M, string path)
{
	M = imread(path);
	if(M.data ==NULL)
		return false;
	return true;
}

void PotentialMap::updateMap(geometry_msgs::Pose g)
{
	//Convert the global to local first
	LocalGlobalMap LG;
	if(!LG.readMap(path_1))
		return ;
	
	LG.setGlobalToLocal(g);
	cout<<"Given pose is "<<LG.getLocalPosition().x<<" , "<<LG.getLocalPosition().y<<" and angle "<<LG.getLocalOrientation()<<endl;

	local_position.x = LG.getLocalPosition().x;
	local_position.y = LG.getLocalPosition().y;
	local_orientation = LG.getLocalOrientation();

	int count = patchTrace(local_position.x, local_position.y, local_orientation);

	cout<<"Count of the area "<<count<<endl;
}

int PotentialMap::patchTrace(int x, int y, double theta)
{
	double min_theta = (theta - 45);
	double max_theta = (theta + 45);
	int count = 0;
	Mat buff_map(m_size, m_type);
	updatedmap.copyTo(buff_map);
	for(double si = min_theta; si<max_theta; si++)
	{
		int x2 = x + RANGE*cos(si*M_PI/180);
		int y2 = y - RANGE*sin(si*M_PI/180);
		int x1 = x;
		int y1 = y;
		count+=rayTrace2D(buff_map, x1, y1, x2, y2);
	}
	buff_map.copyTo(updatedmap);
	return count;
}

int PotentialMap::rayTrace2D(Mat & buff_map, int x1, int y1, int x2, int y2)
{
	double distance = dist(x1, y1, x2, y2);
	int steps = (int)floor(distance)+1;
	double x_step = 1.0*(x2-x1)/steps;
	double y_step = 1.0*(y2-y1)/steps;
	int count=0;
	double x = x1, y = y1;
	int xi1, xi2, yi1, yi2;
	int vector = 0;
	double dx, dy; 
	while(++vector <=steps)
	{
		xi1 = (int)x; xi2 = (xi1<m_cols - 1)? xi1+1: xi1;
		yi1 = (int)y; yi2 = (yi1<m_rows - 1)? yi1+1: yi1;
		dx = x - xi1; dy = y - yi1;
		int kx, ky;
		if(dx < 0.5)
		{
			if(dy < 0.5) {
				kx = xi1; ky = yi1;
			}
			else {
				kx = xi1; ky = yi2;
			}
		}
		else
		{
			if(dy < 0.5) {
				kx = xi2; ky = yi1;
			}
			else {
				kx = xi2; ky = yi2;
			}
		}
		if(isGreen(buff_map, ky, kx))
		{
			count++;
			buff_map.at<Vec3b>(ky, kx)[0] = 255; 
			buff_map.at<Vec3b>(ky, kx)[1] = 0;
			buff_map.at<Vec3b>(ky, kx)[2] = 255;
		}
		if(isBlack(buff_map, ky, kx))
			return count;
		x+= x_step;
		y+= y_step;
	}
	return count;
}

void PotentialMap::writeImage(Mat& M, string outpath)
{
	imwrite(outpath, M);
	
}


void callback(const geometry_msgs::PoseStamped::ConstPtr& l)
{
	cout<<"In the callback function"<<endl;

	int k;
	cout<<"Enter to update the markers"<<endl;
	cin>>k;
	
	PotentialMap PM(path_1);
	PM.global_position.position.x = l->pose.position.x;
	PM.global_position.position.y = l->pose.position.y;
	PM.global_position.position.z = l->pose.position.z;
	PM.global_position.orientation.x = l->pose.orientation.x;
	PM.global_position.orientation.y = l->pose.orientation.y;
	PM.global_position.orientation.z = l->pose.orientation.z;
	PM.global_position.orientation.w = l->pose.orientation.w;
	PM.readImage(PM.updatedmap, path_2);
	PM.updateMap(PM.global_position);
	PM.writeImage(PM.updatedmap, path_2);

	PM.updatePotentialMap(PM.updatedmap);

}


void PotentialMap::updatePotentialMap(Mat& M)
{
	LocalGlobalMap G;
	if(!G.readMap(path_1))
		return ;

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
				G.setLocalToGlobal(Point(j-4, i+5), 0);

				geometry_msgs::Pose g = G.getGlobal();
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
	marker_pub.publish(MA);
	geometry_msgs::PoseStamped gb;
	gb.header.stamp = ros::Time::now();
	gb.header.frame_id = "/map";
	gb.pose.position.x = global_position.position.x;
	gb.pose.position.y = global_position.position.y;
	gb.pose.position.z = global_position.position.z;
	gb.pose.orientation.x = global_position.orientation.x;
	gb.pose.orientation.y = global_position.orientation.y;
	gb.pose.orientation.z = global_position.orientation.z;
	gb.pose.orientation.w = global_position.orientation.w;
	goal_pub.publish(gb);
}
int main(int ac, char** av)
{
	if(ac<3)
	{
		cout<<"Enter the path of original and updated images"<<endl;
		return -1;
	}
	path_1 = av[1];
	path_2 = av[2];
	ros::init(ac, av, "potentialmap_update");
	ros::NodeHandle nh;
	ros::Subscriber cur_sub = nh.subscribe("/move_base_simple/goal", 1000, &callback);
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/potentialloc", 1);	
	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ourgoal", 1);	
	cout<<"Spinning..."<<endl;

	ros::spin();
	return 0;
}
