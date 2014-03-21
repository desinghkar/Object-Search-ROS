#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define RANGE 40
#define SCALE 10
using namespace std;
using namespace cv;

class Potential_map
{
	ros::NodeHandle nh;
	Mat P_map;
	Mat Prob1, Prob2, Prob3, Prob4;
	float TotalA;
	Mat disp_Prob;

	public:
	Potential_map()
	{}
	Potential_map(string path)
	{
		P_map = imread(path);
		disp_Prob.create(P_map.rows*10, P_map.cols*10, CV_8UC3);
		getTableArea();
		create_prob_maps();
		show_map("Potential_map");
	}
	~Potential_map()
	{
	}
	void read_map(string path)
	{
		P_map = imread(path);
	}
	void getTableArea()
	{
		for(int i=0; i<P_map.cols; i++)
			for(int j=0; j<P_map.rows; j++)
				if(P_map.at<Vec3b>(j, i)[0]==0&&P_map.at<Vec3b>(j, i)[1]==255&&P_map.at<Vec3b>(j, i)[2]==0)
					TotalA++;
		TotalA = TotalA * 0.05 * 0.05;
	}
	void show_map(string window_name)
	{
		namedWindow(window_name, 0);
		imshow(window_name, P_map);
		waitKey(0);
	}
	void create_prob_maps()
	{
		float theta = 0;
		initialize(Prob1);
		updateVPProb(Prob1, theta, "Prob1.png");
		theta = 90;
		initialize(Prob2);
		updateVPProb(Prob2, theta, "Prob2.png");
		theta = 180;
		initialize(Prob3);
		updateVPProb(Prob3, theta, "Prob3.png");
		theta = 270;
		initialize(Prob4);
		updateVPProb(Prob4, theta, "Prob4.png");

	}
	void initialize(Mat & Prob)
	{
		Prob.create(P_map.size(), CV_32F);
		for(int i=0; i<P_map.cols; i++)
			for(int j=0; j<P_map.rows; j++)
			{
				if(P_map.at<Vec3b>(i, j)[0]==205 && P_map.at<Vec3b>(i, j)[1]==205 && P_map.at<Vec3b>(i, j)[2]==205)
					Prob.at<float>(i, j) = 999; //Grey
				else if(P_map.at<Vec3b>(i, j)[0]==0 && P_map.at<Vec3b>(i, j)[1]==255 && P_map.at<Vec3b>(i, j)[2]==0)
					Prob.at<float>(i, j) = 555; //Green
				else if(P_map.at<Vec3b>(i, j)[0]==0 && P_map.at<Vec3b>(i, j)[1]==0 && P_map.at<Vec3b>(i, j)[2]==0)
					Prob.at<float>(i, j) = 888; //Black
				else
					Prob.at<float>(i, j) = 0; //Otherwise


			}
	}
	void updateVPProb(Mat & Prob, float theta, string path)
	{
		int kar=0;
		for(int i=0; i<Prob.cols; i++)
			for(int j=0; j<Prob.rows; j++)
			{
//				cout<<kar++<<"   "<<endl;
				if(Prob.at<float>(j, i)!=999 && Prob.at<float>(j, i)!=555 && Prob.at<float>(j, i)!=888)
				{
					float ViewA = 1.0*PatchTraceAt(i, j, theta)*0.05*0.05;
					Prob.at<float>(j, i) = ViewA/TotalA;

//					cout<<" [ "<<PatchTraceAt(i, j, theta)<<" ]"<<endl;
			        }
			}
//		cout<<Prob<<endl;
		disp_Probability(Prob, theta, path);
	}
	int PatchTraceAt(int x, int y, float theta)
	{
//		cout<<"In PatchTrace"<<endl;
		float min_theta =(theta - 45);
		float max_theta =(theta + 45);
		int count = 0;
		float si = max_theta;
		for(float si=min_theta; si<max_theta; si++)
		{

			int x2 = x + RANGE*cos(si*M_PI/180);
			int y2 = y - RANGE*sin(si*M_PI/180);
			int x1 = x;
			int y1 = y;

			count+=rayTrace2D(x1, y1, x2, y2);

		}			
		return count;
	}
	int rayTrace2D(int x1, int y1, int x2, int y2)
	{
//		cout<<"In rayTrace2D"<<endl;
		double distance = dist(x1, y1, x2, y2);
		if(distance>RANGE)
			return 0;
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
			xi1 = (int)x; xi2 = (xi1<P_map.cols - 1)? xi1+1: xi1;
			yi1 = (int)y; yi2 = (yi1<P_map.rows - 1)? yi1+1: yi1;

			dx = x - xi1; dy = y - yi1;
			
			if(dx < 0.5)
				if(dy < 0.5) {
					      B = P_map.at<Vec3b>(yi1, xi1)[0]; 
					      G = P_map.at<Vec3b>(yi1, xi1)[1];
					      R = P_map.at<Vec3b>(yi1, xi1)[2];
				}
				else         {
					      B = P_map.at<Vec3b>(yi2, xi1)[0]; 
					      G = P_map.at<Vec3b>(yi2, xi1)[1];
					      R = P_map.at<Vec3b>(yi2, xi1)[2];
				}
			else
				if(dy < 0.5) {
					      B = P_map.at<Vec3b>(yi1, xi2)[0]; 
					      G = P_map.at<Vec3b>(yi1, xi2)[1];
					      R = P_map.at<Vec3b>(yi1, xi2)[2];
				}
				else {
					      B = P_map.at<Vec3b>(yi2, xi2)[0]; 
					      G = P_map.at<Vec3b>(yi2, xi2)[1];
					      R = P_map.at<Vec3b>(yi2, xi2)[2];
				}

			if(B==0&&G==255&&R==0)
			{
				count++;
//				cout<<"Green "<<count<<endl;
			}
			else if(B==0&&G==0&&R==0)
				return count;
			x+= x_step;
			y+= y_step;
			

		}
		return count;

	}
	double dist(int a1, int b1, int a2, int b2)
	{
		double d = 1.0*(a1-a2)*(a1-a2) + 1.0*(b1-b2)*(b1-b2);
		return sqrt(d);
	}
	void show_vector_map(string window_name)
	{
	}
	void disp_Probability(Mat& Prob, float theta, string path)
	{
		int x, y;
		for(int i=0; i<P_map.rows; i++)
			for(int j=0; j<P_map.cols; j++)
			{
				cv::Point P1, P2;
				P1.x = i*SCALE; P1.y = j*SCALE;
				P2.x = (i+1)*SCALE-1; P2.y = (j+1)*SCALE-1;
				int v;
				if((Prob.at<float>(j, i)!=999)&&(Prob.at<float>(j, i)!=888)&&Prob.at<float>(j, i)!=555)
				{
					v = roundup(255*Prob1.at<float>(j, i));
			//		x = j; y = i;
				}
				else
					v = 255;
				rectangle(disp_Prob, P1, P2, cv::Scalar(110, 255-v, 0), -1);
			}
		/*****************************************************************/
		x = P_map.rows/2; y = P_map.cols/2;
		float min_theta =(theta - 45);
		float max_theta =(theta + 45);
		cout<<"min theta is "<<min_theta<<" max theta is "<<max_theta<<endl;
		int q = 0;
		for(float si=min_theta; si<=max_theta; si++)
		{
			q++;
			int x2 = x + RANGE*cos(si*M_PI/180);
			int y2 = y - RANGE*sin(si*M_PI/180);
			int x1 = x;
			int y1 = y;

			//count+=rayTrace2D(x1, y1, x2, y2);
			double distance = dist(x1, y1, x2, y2);
			//if(distance>RANGE)
			//	continue;
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
				xi1 = (int)x; xi2 = (xi1<P_map.cols - 1)? xi1+1: xi1;
				yi1 = (int)y; yi2 = (yi1<P_map.rows - 1)? yi1+1: yi1;

				dx = x - xi1; dy = y - yi1;

				cv::Point P1, P2;
				if(dx < 0.5)
					if(dy < 0.5) {
						P1.y = yi1*SCALE; P1.x = xi1*SCALE;
						P2.y = (yi1+1)*SCALE-1; P2.x = (xi1+1)*SCALE-1;
					}
					else         {
						P1.y = yi2*SCALE; P1.x = xi1*SCALE;
						P2.y = (yi2+1)*SCALE-1; P2.x = (xi1+1)*SCALE-1;
					}
				else
					if(dy < 0.5) {
						P1.y = yi1*SCALE; P1.x = xi2*SCALE;
						P2.y = (yi1+1)*SCALE-1; P2.x = (xi2+1)*SCALE-1;
					}
					else {
						P1.y = yi2*SCALE; P1.x = xi2*SCALE;
						P2.y = (yi2+1)*SCALE-1; P2.x = (xi2+1)*SCALE-1;
					}

				rectangle(disp_Prob, P1, P2, cv::Scalar(0, 0, 255), -1);
				x+= x_step;
				y+= y_step;


			}
		}
		cout<<"Q is "<<q<<endl;

		namedWindow("Probability", 0);
		imshow("Probability", disp_Prob);

		waitKey(0);
		imwrite(path, disp_Prob);
	}
	int roundup(double v)
	{
		int buff = v;
		if(v-buff>=0/5)
			return buff+1;
		else
			return buff;
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
	Potential_map P(argv[1]);


	ros::spin();

	

}
