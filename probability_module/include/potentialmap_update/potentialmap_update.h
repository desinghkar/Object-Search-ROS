#include "just_vp_prob/just_vp_prob.h"
#include "just_time/just_time.h"
#include "fuseProbabilities/fuseProbabilities.h"
#include "localglobalmap/localglobalmap.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PotentialMap
{
	public:
	Mat originalmap;
	Mat updatedmap;
	int m_rows, m_cols, m_type, orient;
	Size m_size;
	geometry_msgs::Pose global_position;
	cv::Point local_position;
	double local_orientation;


	PotentialMap(){};
	~PotentialMap(){};
	//To read the original map and set the params
	PotentialMap(string);

	//to read the image
	bool readImage(Mat&, string);

	//to update the patch based on the location from the global
	void updateMap(geometry_msgs::Pose);

	//to write the image to the disk
	void writeImage(Mat&, string);

	//To update patchtrace
	int patchTrace(int , int , double );

	//To update the raytrace2D
	int rayTrace2D(Mat&, int, int, int, int);

	//To update the markers in rviz
	void updatePotentialMap(Mat&);


	

};
