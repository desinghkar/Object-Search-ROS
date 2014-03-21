#include "just_vp_prob/just_vp_prob.h"
#include "just_time/just_time.h"

class FuseProb
{
	vector<Mat> Prob;
	vector<Mat> Time;
	vector<Mat> ResultProb;
	int m_rows, m_cols, m_type, orient;
	Size m_size;
	Point goal;
	double goal_orientation;//In degrees
	
	public:
	FuseProb(){};
	~FuseProb(){};

	//To read the Prob and Time from the input
	FuseProb(vector<Mat>, vector<Mat>, Mat&, int, string);

	//To read the Prob from input
	void readProbTime(vector<Mat>, vector<Mat>);

	//To normalize the vector<mat>
	void normalizeProbs();
	
	//To normalize the mat
	void normalize(Mat&);

	//To fuse time and prob
	void multProbTime();

	//To get access to the ResultProb
	vector<Mat> getResultFusedProb();

	//To saveImages of the resulting probabilities vector<Mat>
	void saveImages(string);

	//To saveimages from mat
	void saveFusedImage(Mat&, string, int);

	//To find the goal where the robot has to search for 
	void computeGoal();	

	//To get the computed goal and orientation
	Point getGoalPosition();
	double getGoalOrientation(); //In degrees
};
