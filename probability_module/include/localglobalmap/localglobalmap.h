#ifndef LOCALGLOBALMAP_H
#define LOCALGLOBALMAP_H
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>


using namespace std;
using namespace cv;
class LocalGlobalMap
{
	Mat map;
	geometry_msgs::Pose global_location;
	Point local_location;
	double local_orientation;
	int m_rows, m_cols, m_type;
	Size m_size;
	public:
	LocalGlobalMap(){};
	~LocalGlobalMap(){};

	//To read a map
	bool readMap(string);

	//To get the Global to Local map system
	void setGlobalToLocal(geometry_msgs::Pose);

	//To get the Local to Global map system
	void setLocalToGlobal(Point, double);

	//To convert the Quaternion to Euler measure 
	void convertQtoE(Mat&);

	//To convert the Euler measuer to Quaternion
	void convertEtoQ(Mat&, double, double, double);

	//To getGlobal
	geometry_msgs::Pose getGlobal();

	//To getLocal
	cv::Point getLocalPosition();

	//To getLocal
	double getLocalOrientation();
	
};



#endif

