#ifndef JUST_TIME_H
#define JUST_TIME_H
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "just_vp_prob/just_vp_prob.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class MapTimeProb{
	
	Mat explored_map;
	vector<Mat> Time;
	vector<Mat> Top;
	Mat result_time;
	Point current_location;
	double current_orientation;
	int m_rows, m_cols, m_type;
	Size m_size;

	public:
	MapTimeProb(){};
	~MapTimeProb(){};
	MapTimeProb(string, string, Point, double);

	//To read the .png potential map 
	bool readExploredMap(string);

	//To set the current location of the robot
	void setCurrentLocation(Point, double);

	//To initialize the time map for each orientation
	void initialize();

	//To update the time map for each orientation
	void updateTimeProb(double);

	//To calculate the time probability at a location and orientation
	double timeProbability(int, int, int);

	//To normalize the time probability map
	void normalize(Mat&);

	//To save the images of the time probs
	void saveResultImages(string);

	//To save image
	void saveTimeImage(Mat&, string, int);

	//To return the result 
	vector<Mat> getResultTime();

	//To write the result time probs in txt file
	void writeTimeMap(string);
};

double diff(double, double);
#endif
