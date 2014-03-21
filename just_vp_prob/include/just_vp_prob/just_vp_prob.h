#ifndef JUST_VP_PROB_H
#define JUST_VP_PROB_H
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
//#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define RANGE 45
#define SCALE 10
using namespace std;
using namespace cv;

class MapViewProb
{
	Mat explored_map;
	vector<Mat> Prob;
	vector<Mat> Top;
	Mat result_prob;
	double Total_tableA;
	int m_rows, m_cols, m_type;
	Size m_size;


	public:
	MapViewProb(){};
	~MapViewProb(){};
	MapViewProb(string, string);
	
	//To read the .png file which is potential map 
	bool readExploredMap(string);

	//To set the 8 orientations prob map based on the potential map given as input
	void initializeProb();

	//To find the area of the table in the potential map
	bool findTableArea();

	//To update the probability of individual Mat in vector<Mat>
	void updateVPProb(double);

	//To find the area of view seen in from a x,y at an angle theta
	int patchTrace(int, int, double);

	//rayTrace to count the cells that are potential (table)
	int rayTrace2D(Mat&, int, int, int, int);
	
	//To test the probmaps
	int rayTrace(Mat&, int, int, int, int);

	//To compute the fused probabalities
	void fuseProbabilities();

	//To get the best view at location w r to 8 orientation
	void resultProbability();

	//To find the top 10 locations in each orientation of Prob
	void top10Loc();

	//To normalize for printing purpose
	void normalize(Mat&);
	
	/*************************************************************************************/

	//To access the area of table
	double getTotalArea();

	//To show the resultant prob map
	void saveResultImages(string); //Not writing this yet

	//To save prob maps
	void saveProbImage(Mat&, string, int);
	//void saveProbImage(Mat&, string);

	//To write the individual maps to a folder
	void writeProbMap(string);

};
//To check the pixel level classes
bool isGreen(Mat, int, int);
bool isBlack(Mat, int, int);
bool isPink(Mat, int, int);
bool isGrey(Mat, int, int);
bool isFree(Mat, int, int);
bool isNavigable(Mat, int, int);

double dist(int, int, int, int);
int roundup(double);
#endif
