#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define RANGE 45
#define SCALE 10
using namespace std;
using namespace cv;
int kcount =0;
int row = 0;
bool iter=true;
ofstream out("Viewpoint_path_traced.txt");
//ofstream locations("locations_of_objects.txt");
string image1_path, image2_path;
class Potential_map
{
	ros::NodeHandle nh;
	ros::Publisher goal_pub;
	ros::Subscriber currentloc_sub;
	Mat P_map;
//	Mat Prob[8];
	Mat Time1, Time2, Time3, Time4, Time5, Time6, Time7, Time8;
	Mat Prob1, Prob2, Prob3, Prob4, Prob5, Prob6, Prob7, Prob8;
	Mat ViewPointMat_v;
	Mat ViewPointMat_x;
	Mat ViewPointMat_y;
	Mat TimeMat_v;
	Mat TimeMat_x;
	Mat TimeMat_y;
	Mat buff2_map;
	Mat Prob_orientation;
	Mat best_orientation;
	Mat time_prob;
	cv::Point maxLoc;
	cv::Point minLoc;
	float maxOrientation, currentOrientation;
	float maxVal, minVal;
	float indexi, indexj;
	geometry_msgs::PoseStamped goal;
	geometry_msgs::Pose current;
	
	float TotalA;
	Mat disp_Prob;

	public:
	Potential_map()
	{
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "/map";


		goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
		//		cout<<"In potential_map"<<endl;
		currentloc_sub = nh.subscribe("/initialpose", 1, &Potential_map::updateProb, this);
//		updateProb();
	}
	void updateProb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& loc)
	{
		//		cout<<"Inside the updateProb"<<endl;
		while(1)
		{
			if(iter)
			{
				P_map = imread(image1_path);
				disp_Prob.create(P_map.rows*10, P_map.cols*10, CV_8UC3); //1000x1000
				time_prob.create(P_map.size(), CV_32FC1);
				ViewPointMat_v.create(8, 5, CV_32FC1);
				ViewPointMat_x.create(8, 5, CV_32FC1);
				ViewPointMat_y.create(8, 5, CV_32FC1);
				TimeMat_v.create(8, 5, CV_32FC1);
				TimeMat_x.create(8, 5, CV_32FC1);
				TimeMat_y.create(8, 5, CV_32FC1);
				cout<<"*******************************************************************"<<endl;
				cout<<"Now get the location from initial position"<<endl;
				current.position.x = loc->pose.pose.position.x;
				current.position.y = loc->pose.pose.position.y;
				current.position.z = loc->pose.pose.position.z;
				current.orientation.x = loc->pose.pose.orientation.x;
				current.orientation.y = loc->pose.pose.orientation.y;
				current.orientation.z = loc->pose.pose.orientation.z;
				current.orientation.w = loc->pose.pose.orientation.w;
/*				current.position.x = 0;
							current.position.y = 0;
							current.position.z = 0;

							current.orientation.x = 0;
							current.orientation.y = 0;
							current.orientation.z = 0;
							current.orientation.w = 0;
	*/			printPose(current);
				getGlobalToMapSystem(current);
				iter = false;
				kcount++;
			}
			else
			{
				P_map = imread(image2_path);
				disp_Prob.create(P_map.rows*10, P_map.cols*10, CV_8UC3); //1000x1000
				time_prob.create(P_map.size(), CV_32FC1);
				ViewPointMat_v.create(8, 5, CV_32FC1);
				ViewPointMat_x.create(8, 5, CV_32FC1);
				ViewPointMat_y.create(8, 5, CV_32FC1);
				TimeMat_v.create(8, 5, CV_32FC1);
				TimeMat_x.create(8, 5, CV_32FC1);
				TimeMat_y.create(8, 5, CV_32FC1);

				cout<<"*******************************************************************"<<endl;
				cout<<"Now get the location from goal"<<endl;
				current.position.x = goal.pose.position.x;
				current.position.y = goal.pose.position.y;
				current.position.z = goal.pose.position.z;

				current.orientation.x = goal.pose.orientation.x;
				current.orientation.y = goal.pose.orientation.y;
				current.orientation.z = goal.pose.orientation.z;
				current.orientation.w = goal.pose.orientation.w;
				printPose(current);
				getGlobalToMapSystem(current);
				kcount++;
			}

			cout<<"Indexi, Indexj "<<indexi<<" , "<<indexj<<endl;
			getTableArea();
			create_prob_maps();
//			create_time_maps();

//			multiplyProbTime();
			setGoal();
		}
		show_map("Potential_map");
	}

	void getGlobalToMapSystem(geometry_msgs::Pose c)
	{
		cout<<"Insite getGlobalToMapSystem"<<endl;
		indexi = c.position.y;
		indexj = c.position.x;
		cout<<"Level 4 indexi="<<indexi<<" indexj="<<indexj<<endl;
		//Level 4-3
		indexi = indexi/0.05;
		indexj = indexj/0.05;
		cout<<"Level 3 indexi="<<indexi<<" indexj="<<indexj<<endl;
		//Level 3-2
		indexi = P_map.rows/2 + indexi;
		indexj = P_map.cols/2 + indexj;
		cout<<"Level 2 indexi="<<indexi<<" indexj="<<indexj<<endl;
		//Level 2-1
		indexi = P_map.cols - indexi;
		cout<<"Level 1 indexi="<<indexi<<" indexj="<<indexj<<endl;

		float phi, theta;
		Mat Q(4, 1, CV_32FC1);
		Q.at<float>(0, 0) = c.orientation.w;
		Q.at<float>(1, 0) = c.orientation.x;
		Q.at<float>(2, 0) = c.orientation.y;
		Q.at<float>(3, 0) = c.orientation.z;

		convertQtoE(Q);
		cout<<"currentOrientation is "<<currentOrientation*180/M_PI<<endl;
		cout<<"location is "<<indexi<<" "<<indexj<<endl;
//		table_area_seen =i0.05*0.05*PatchTraceAt((int)indexj, (int)indexi, (si*180/M_PI));

		//namedWindow("updatedPmap", 1);
		//imshow("updatePmap", updatedPotentialMap);
		//waitKey(0);
//		imwrite("updatedProbabilityMap.png", updatedPotentialMap);
	}
	void convertQtoE(Mat& M)
	{
		float q0 = M.at<float>(0, 0);
		float q1 = M.at<float>(1, 0);
		float q2 = M.at<float>(2, 0);
		float q3 = M.at<float>(3, 0);

//		*phi = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
//		*theta = asin(2*(q0*q2 - q3*q1));
		currentOrientation = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
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
		TotalA = 0;
		for(int i=0; i<P_map.rows; i++)
			for(int j=0; j<P_map.cols; j++)
			{
				if(P_map.at<Vec3b>(i, j)[0]==0&&P_map.at<Vec3b>(i, j)[1]==255&&P_map.at<Vec3b>(i, j)[2]==0)
				{
					TotalA++;

				}
			}
		cout<<"TotalA in the input map is "<<TotalA<<endl;
		if(TotalA<5)
		{
			cout<<"All table regions are explored!!!!! Object in search is not found!!! :( "<<endl;
			exit(0);
		}
	}
	void show_map(string window_name)
	{
		//namedWindow(window_name, 0);
		//imshow(window_name, P_map);
		//waitKey(0);
	}
	void create_time_maps()
	{
		initialize(Time1);
		initialize(Time2);
		initialize(Time3);
		initialize(Time4);
		initialize(Time5);
		initialize(Time6);
		initialize(Time7);
		initialize(Time8);
		
		updateVPTime(Time1, 0, "Time.png");
		updateVPTime(Time2, 45, "Time.png");
		updateVPTime(Time3, 90, "Time.png");
		updateVPTime(Time4, 135, "Time.png");
		updateVPTime(Time5, 180, "Time.png");
		updateVPTime(Time6, 225, "Time.png");
		updateVPTime(Time7, 270, "Time.png");
		updateVPTime(Time8, 315, "Time.png");
		float max=-1;
		int ki;
		for(int i=0; i<8; i++)
		{
			float v=TimeMat_v.at<float>(i, 0);
			if(max<v)
			{
				max = v;
				ki=i;

			}
		}
		writeMatTime(ki);

		scaleUp(max, Time1);
		scaleUp(max, Time2);
		scaleUp(max, Time3);
		scaleUp(max, Time4);
		scaleUp(max, Time5);
		scaleUp(max, Time6);
		scaleUp(max, Time7);
		scaleUp(max, Time8);
		cout<<"max value is "<<max<<endl;
		normaliseProb(Time1, (2.0*max), 0, "Time1.png", 1);
		normaliseProb(Time2, (2.0*max), 1, "Time2.png", 1);
		normaliseProb(Time3, (2.0*max), 2, "Time3.png", 1);
		normaliseProb(Time4, (2.0*max), 3, "Time4.png", 1);
		normaliseProb(Time5, (2.0*max), 4, "Time5.png", 1);
		normaliseProb(Time6, (2.0*max), 5, "Time6.png", 1);
		normaliseProb(Time7, (2.0*max), 6, "Time7.png", 1);
		normaliseProb(Time8, (2.0*max), 7, "Time8.png", 1);
	}
	void writeMatTime(int k)
	{
		Mat S(Time1.size(), Time1.type());
		switch(k)
		{
			case 0: Time1.copyTo(S); break;
			case 1: Time2.copyTo(S); break;
			case 2: Time3.copyTo(S); break;
			case 3: Time4.copyTo(S); break;
			case 4: Time5.copyTo(S); break;
			case 5: Time6.copyTo(S); break;
			case 6: Time7.copyTo(S); break;
			case 7: Time8.copyTo(S); break;
		}
		ofstream file("TimeMat.txt");
		for(int i=0; i<S.rows; i++)
		{
			for(int j=0; j<S.cols; j++)
			{
				float v=S.at<float>(i,j);
				if(v!=999 && v!=888 && v!=777 && v!=555)
				{
					file<<v<<", ";
				}
				else 
					file<<0.0<<", ";
			}
			file<<";"<<endl;
		}
	}
	void writeMatProb(int k)
	{
		Mat S(Prob1.size(), Prob1.type());
		switch(k)
		{
			case 0: Prob1.copyTo(S); break;
			case 1: Prob2.copyTo(S); break;
			case 2: Prob3.copyTo(S); break;
			case 3: Prob4.copyTo(S); break;
			case 4: Prob5.copyTo(S); break;
			case 5: Prob6.copyTo(S); break;
			case 6: Prob7.copyTo(S); break;
			case 7: Prob8.copyTo(S); break;
		}
		ofstream file("ProbMat.txt");
		for(int i=0; i<S.rows; i++)
		{
			for(int j=0; j<S.cols; j++)
			{
				float v=S.at<float>(i,j);
				if(v!=999 && v!=888 && v!=777 && v!=555)
				{
					file<<v<<", ";
				}
				else 
					file<<0.0<<", ";
			}
			file<<";"<<endl;
		}

			

	}
	void scaleUp(float max, Mat& M)
	{
		for(int i=0; i<M.rows; i++)
			for(int j=0; j<M.cols; j++)
			{
				float v = M.at<float>(i,j);
				if(v!=999 && v!=888 && v!=777 && v!=555)
				{
					v = v+(max);
//					M.at<float>(i, j) = v/(1.5*max);
				}
			}
	}
	void create_prob_maps()
	{
	
		float theta = 0;
		initialize(Prob1);
		updateVPProb(Prob1, theta, "Prob1.png");
		theta = 45;
		initialize(Prob2);
		updateVPProb(Prob2, theta, "Prob2.png");
		theta = 90;
		initialize(Prob3);
		updateVPProb(Prob3, theta, "Prob3.png");
		theta = 135;
		initialize(Prob4);
		updateVPProb(Prob4, theta, "Prob4.png");
		theta = 180;
		initialize(Prob5);
		updateVPProb(Prob5, theta, "Prob5.png");
		theta = 225;
		initialize(Prob6);
		updateVPProb(Prob6, theta, "Prob6.png");
		theta = 270;
		initialize(Prob7);
		updateVPProb(Prob7, theta, "Prob7.png");
		theta = 315;
		initialize(Prob8);
		updateVPProb(Prob8, theta, "Prob8.png");

		//Now get the max of all 8 orientations
		float max=-1;
		int ki;
		for(int i=0; i<8; i++)
		{
			float v=ViewPointMat_v.at<float>(i, 0);
			if(max<v)
			{
				max = v;
				ki = i;
			}
		}
		writeMatProb(ki);
		cout<<"max value is "<<max<<endl;
		normaliseProb(Prob1, max, 0, "Prob1.png", 0);
		normaliseProb(Prob2, max, 1, "Prob2.png", 0);
		normaliseProb(Prob3, max, 2, "Prob3.png", 0);
		normaliseProb(Prob4, max, 3, "Prob4.png", 0);
		normaliseProb(Prob5, max, 4, "Prob5.png", 0);
		normaliseProb(Prob6, max, 5, "Prob6.png", 0);
		normaliseProb(Prob7, max, 6, "Prob7.png", 0);
		normaliseProb(Prob8, max, 7, "Prob8.png", 0);

		

	}
	void multiplyProbTime()
	{
		multMat(Prob1, Time1);
		multMat(Prob2, Time2);
		multMat(Prob3, Time3);
		multMat(Prob4, Time4);
		multMat(Prob5, Time5);
		multMat(Prob6, Time6);
		multMat(Prob7, Time7);
		multMat(Prob8, Time8);
		disp_Probability(Prob1, 0, "Cum1.png", 0);
		disp_Probability(Prob2, 1, "Cum2.png", 0);
		disp_Probability(Prob3, 2, "Cum3.png", 0);
		disp_Probability(Prob4, 3, "Cum4.png", 0);
		disp_Probability(Prob5, 4, "Cum5.png", 0);
		disp_Probability(Prob6, 5, "Cum6.png", 0);
		disp_Probability(Prob7, 6, "Cum7.png", 0);
		disp_Probability(Prob8, 7, "Cum8.png", 0);

	}
	void initialize(Mat & Prob)
	{
		Prob.create(P_map.size(), CV_32F);
		for(int i=0; i<P_map.rows; i++)
			for(int j=0; j<P_map.cols; j++)
			{
				if(P_map.at<Vec3b>(i, j)[0]==205 && P_map.at<Vec3b>(i, j)[1]==205 && P_map.at<Vec3b>(i, j)[2]==205)
					Prob.at<float>(i, j) = 999; //Grey
				else if(P_map.at<Vec3b>(i, j)[0]==0 && P_map.at<Vec3b>(i, j)[1]==255 && P_map.at<Vec3b>(i, j)[2]==0)
					Prob.at<float>(i, j) = 555; //Green
				else if(P_map.at<Vec3b>(i, j)[0]==0 && P_map.at<Vec3b>(i, j)[1]==0 && P_map.at<Vec3b>(i, j)[2]==0)
					Prob.at<float>(i, j) = 888; //Black
				else if(P_map.at<Vec3b>(i, j)[0]==255 && P_map.at<Vec3b>(i, j)[1]==0 && P_map.at<Vec3b>(i, j)[2]==255)
					Prob.at<float>(i, j) = 777; //Black
				else
					Prob.at<float>(i, j) = 0; //Otherwise


			}
	}
	void updateVPTime(Mat & Time, float theta, string path)
	{
//		cout<<"In updateVPT"<<endl;
		for(int i=0; i<Time.rows; i++)
			for(int j=0; j<Time.cols; j++)
			{
				float t = Time.at<float>(i, j);
				if(t!=999 && t!=777 && t!=888 && t!=555)
				{
					Time.at<float>(i, j) = timeProbability(i, j, theta/45);
//					cout<<"Time prob value is "<<timeProbability(i, j, theta/45)<<endl;
				}
			}
		getMax5Loc(Time, theta, TimeMat_v, TimeMat_x, TimeMat_y);
		

//		disp_Probability(Time, theta/45, path);
	}
	void updateVPProb(Mat & Prob, float theta, string path)
	{
		int kar=0;

		buff2_map.create(P_map.size(), P_map.type());
		for(int i=0; i<Prob.rows; i++)
			for(int j=0; j<Prob.cols; j++)
			{
				float v = Prob.at<float>(i, j);
				if(v!=999 && v!=555 && v!=888 && v!=777)
				{
					double ViewA = 1.0*PatchTraceAt(j, i, theta);
					Prob.at<float>(i, j) = ViewA/TotalA;
					if(ViewA>=TotalA)
					{
						cout<<"ViewA is "<<ViewA<<endl;
						cout<<"Size of P_map is "<<P_map.rows<<" "<<P_map.cols<<endl;
						cout<<"Size of Prob is "<<Prob.rows<<" "<<Prob.cols<<endl;
					}
			        }
			}
		getMax5Loc(Prob, theta, ViewPointMat_v, ViewPointMat_x, ViewPointMat_y);
//		row++;
	}
	void normaliseProb(Mat& Prob, float v, int rot,  string path, bool flag)
	{
//		cout<<"In normaliseProb for"<<path<<endl;
		for(int i=0; i<Prob.rows; i++)
			for(int j=0; j<Prob.cols; j++)
			{
				float val = Prob.at<float>(i, j);
				if(val!=999 && val!=777 && val!=888 && val!=555)
				{
					Prob.at<float>(i, j) = val/v;
				}
			}
		disp_Probability(Prob, rot, path, flag);
	}
	double timeProbability(int i, int j, int rot)
	{
		double t1, t2;
		double v1 = 6, v2 = 1;
		double d1 = dist(indexi, indexj, i, j);
		double d2 = diff(currentOrientation, 1.0*rot*M_PI/4);
		t1 = d1/v1;
		t2 = d2/v2;
		double result;
		if(t1+t2<1)
			result = 1;
		else
			result = 1/(t1+t2);
		if(result ==0)
			return (-1);
		else
			return (result);
	}
	int PatchTraceAt(int x, int y, float theta)
	{
//		cout<<"In PatchTrace"<<endl;
		float min_theta =(theta - 45);
		float max_theta =(theta + 45);
		int count = 0;
//		kcount = 0;
		float si = max_theta;
		P_map.copyTo(buff2_map);
		Mat buff_map(P_map.size(), P_map.type());
		P_map.copyTo(buff_map);
		for(float si=min_theta; si<max_theta; si++)
		{

			int x2 = x + RANGE*cos(si*M_PI/180);
			int y2 = y - RANGE*sin(si*M_PI/180);
			int x1 = x;
			int y1 = y;

			count+=rayTrace2D(buff_map, buff2_map, x1, y1, x2, y2);

		}

//		cout<<"Kcount is "<<kcount<<endl;
//		cout<<"Count in PatchTrace is "<<count<<endl;
		return count;
	}
	int rayTrace2D(Mat & buff_map, Mat & buff2_map,int x1, int y1, int x2, int y2)
	{
//		cout<<"In rayTrace2D"<<endl;
		double distance = dist(x1, y1, x2, y2);
//		if(distance>RANGE)
//			return 0;
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
			int kx, ky;
			if(dx < 0.5)
				if(dy < 0.5) {
					      B = buff_map.at<Vec3b>(yi1, xi1)[0]; 
					      G = buff_map.at<Vec3b>(yi1, xi1)[1];
					      R = buff_map.at<Vec3b>(yi1, xi1)[2];
					      kx = xi1;
					      ky = yi1;

				}
				else         {
					      B = buff_map.at<Vec3b>(yi2, xi1)[0]; 
					      G = buff_map.at<Vec3b>(yi2, xi1)[1];
					      R = buff_map.at<Vec3b>(yi2, xi1)[2];
					      kx = xi1;
					      ky = yi2;
				}
			else
				if(dy < 0.5) {
					      B = buff_map.at<Vec3b>(yi1, xi2)[0]; 
					      G = buff_map.at<Vec3b>(yi1, xi2)[1];
					      R = buff_map.at<Vec3b>(yi1, xi2)[2];
					      kx = xi2;
					      ky = yi1;
				}
				else {
					      B = buff_map.at<Vec3b>(yi2, xi2)[0]; 
					      G = buff_map.at<Vec3b>(yi2, xi2)[1];
					      R = buff_map.at<Vec3b>(yi2, xi2)[2];
					      kx = xi2;
					      ky = yi2;
				}

			if(B==0&&G==255&&R==0)
			{
				count++;
				buff_map.at<Vec3b>(ky, kx)[0] = 205; 
				buff_map.at<Vec3b>(ky, kx)[1] = 205;
				buff_map.at<Vec3b>(ky, kx)[2] = 205;
				buff2_map.at<Vec3b>(ky, kx)[0] = 125; 
				buff2_map.at<Vec3b>(ky, kx)[1] = 125;
				buff2_map.at<Vec3b>(ky, kx)[2] = 255;



//				cout<<"Green "<<count<<endl;
			}
//			if(B==0&&G==0&&R==255)

			if(B==0&&G==0&&R==0)
				return count;
			x+= x_step;
			y+= y_step;
//			kcount++;

		}
//		cout<<"Count in rayTrace is "<<count<<endl;
		return count;

	}
	double dist(int a1, int b1, int a2, int b2)
	{
		double d = 1.0*(a1-a2)*(a1-a2) + 1.0*(b1-b2)*(b1-b2);
		return sqrt(d);
	}
	double diff(float A1, float A2)
	{
		double diff = A1 - A2;
		if(diff>M_PI)
			diff = 2*M_PI - diff;
		if(diff/(2*M_PI)>1)
			diff = 2*M_PI - diff;
		return diff;
	}
	void show_vector_map(string window_name)
	{
	}
	void disp_Probability(Mat& Prob, int rot, string path, bool flag)
	{
//		cout<<"In disp_prob for "<<path<<endl;
		for(int i=0; i<P_map.rows; i++)
			for(int j=0; j<P_map.cols; j++)
			{
				int B=0, G=255, R=255;
				cv::Point P1, P2;
				P1.x = j*SCALE; P1.y = i*SCALE;
				P2.x = (j+1)*SCALE-1; P2.y = (i+1)*SCALE-1;
				int v=0;
				float val = Prob.at<float>(i, j); //First edit here
				if((val!=999)&&(val!=888)&&(val!=555)&&(val!=777))
				{
					v = roundup(255*(Prob.at<float>(i, j)));
				}
				else if(val==999)
				{
					B = 205;G = 205;R = 205;
				}
				else if(val==555)
				{
					B = 0;G = 255;R = 0;
				}
				else if(val==777)
				{
					B = 255;G = 0;R = 255;
				}
				else
				{
					B = 0; G = 0; R = 0;
				}
				rectangle(disp_Prob, P1, P2, cv::Scalar(B, G-v, R), -1);
			}
		if(!flag)
		{
			getMax5Loc(Prob, rot*45, ViewPointMat_v, ViewPointMat_x, ViewPointMat_y);	
			for(int i=4; i>-1; i--)
			{
				cv::Point P1, P2, P, Q;
				float x, y;
				x = ViewPointMat_x.at<float>(rot, i);
				y = ViewPointMat_y.at<float>(rot, i);
				//			cout<<"ViewPointMat x = "<<x<<" y = "<<y<<endl;
				P1.x = (int)x*SCALE; P1.y = (int)y*SCALE;
				P2.x = (int)(x+1)*SCALE-1; P2.y = (int)(y+1)*SCALE-1;
				P.x = (P1.x+P2.x)/2; P.y = (P1.y+P2.y)/2;
				//			cout<<"ViewPointMat P1.x = "<<P1.x<<" P1.y = "<<P1.y<<endl;
				//			cout<<"ViewPointMat P2.x = "<<P2.x<<" P2.y = "<<P2.y<<endl;
				//			cout<<"ViewPointMat P.x = "<<P.x<<" P.y = "<<P.y<<endl;
				circle(disp_Prob, P, 20, cv::Scalar(255-i*50, 5+i*50, 0), -1);
				Q.x = P.x + 40*cos((rot)*M_PI/4);
				Q.y = P.y - 40*sin((rot)*M_PI/4);
				line(disp_Prob, P, Q, cv::Scalar(255, 255, 255));
			}
		}
		else	
		{
			getMax5Loc(Prob, rot*45, TimeMat_v, TimeMat_x, TimeMat_y);	
			for(int i=0; i<5; i++)
			{
				cv::Point P1, P2, P, Q;
				float x, y;
				x = TimeMat_x.at<float>(rot, i);
				y = TimeMat_y.at<float>(rot, i);
				//			cout<<"ViewPointMat x = "<<x<<" y = "<<y<<endl;
				P1.x = (int)x*SCALE; P1.y = (int)y*SCALE;
				P2.x = (int)(x+1)*SCALE-1; P2.y = (int)(y+1)*SCALE-1;
				P.x = (P1.x+P2.x)/2; P.y = (P1.y+P2.y)/2;
				//			cout<<"ViewPointMat P1.x = "<<P1.x<<" P1.y = "<<P1.y<<endl;
				//			cout<<"ViewPointMat P2.x = "<<P2.x<<" P2.y = "<<P2.y<<endl;
				//			cout<<"ViewPointMat P.x = "<<P.x<<" P.y = "<<P.y<<endl;
				circle(disp_Prob, P, 20, cv::Scalar(255-i*50, 5+i*50, 0), -1);
				Q.x = P.x + 40*cos((rot)*M_PI/4);
				Q.y = P.y - 40*sin((rot)*M_PI/4);
				line(disp_Prob, P, Q, cv::Scalar(255, 255, 255));
			}

		}
		//imwrite(path, disp_Prob);
	}
	int roundup(double v)
	{
		int buff = v;
		if(v-buff>=0/5)
			return buff+1;
		else
			return buff;
	}
	void maxLocationProb()
	{
		float max=-1;
		for(int i=0; i<8; i++)
		{
			float v = ViewPointMat_v.at<float>(i, 0);
			if(max<v)
			{
				max = v;
				maxLoc.x = ViewPointMat_x.at<float>(i, 0);
				maxLoc.y = ViewPointMat_y.at<float>(i, 0);
				maxOrientation = i*45;

			}
		}
	}
	void printPose(geometry_msgs::Pose goal)

	{
		cout<<"The location is "<<endl;
		cout<<"x = "<<goal.position.x<<" y = "<<goal.position.y<<" z = "<<goal.position.z<<endl;
		cout<<"The orientation is "<<endl;
		cout<<"x = "<<goal.orientation.x<<" y = "<<goal.orientation.y<<" z = "<<goal.orientation.z<<" w = "<<goal.orientation.w<<endl;
	}
	void print(geometry_msgs::PoseStamped goal)
	{
		cout<<"The location is "<<endl;
		cout<<"x = "<<goal.pose.position.x<<" y = "<<goal.pose.position.y<<" z = "<<goal.pose.position.z<<endl;
		cout<<"The orientation is "<<endl;
		cout<<"x = "<<goal.pose.orientation.x<<" y = "<<goal.pose.orientation.y<<" z = "<<goal.pose.orientation.z<<" w = "<<goal.pose.orientation.w<<endl;
	}
	void setGoal()
	{
		maxLocationProb();
		float goalX, goalY, goalZ;
		//Level one
		goalX = maxLoc.x; goalY = maxLoc.y; goalZ = 0.0;
		cout<<"Level 1 x="<<goalX<<" y="<<goalY<<endl;
		//Level two
		goalY = Prob1.cols - goalY;
		cout<<"Level 2 x="<<goalX<<" y="<<goalY<<endl;
		//Level three
		goalX = goalX - Prob1.rows/2;
		goalY = goalY - Prob1.cols/2;
		cout<<"Level 3 x="<<goalX<<" y="<<goalY<<endl;
		//Level four
		goalX = 0.05*goalX;
		goalY = 0.05*goalY;
		cout<<"Level 4 x="<<goalX<<" y="<<goalY<<endl;
		Mat Q(4, 1, CV_32FC1);
		float phi=0, theta=0, si=maxOrientation*M_PI/180;
		setQ(Q, phi, theta, si);
		
		goal.pose.position.x = goalX;
		goal.pose.position.y = goalY;
		goal.pose.position.z = goalZ;

		goal.pose.orientation.w = Q.at<float>(0, 0);
		goal.pose.orientation.x = Q.at<float>(1, 0);
		goal.pose.orientation.y = Q.at<float>(2, 0);
		goal.pose.orientation.z = Q.at<float>(3, 0);
	
		cout<<"maxOrientation is"<<maxOrientation<<endl;
		print(goal);

		out<<maxLoc.x<<" "<<maxLoc.y<<" "<<maxOrientation<<endl;
		goal_pub.publish(goal);
		
//		namedWindow("ima");
//		waitKey(5000);
		int br;
		cout<<"Enter for next viewpoint"<<endl;
		cin>>br;

	}
	void setQ(Mat& Q, float phi, float theta, float si)
	{
		Q.at<float>(0,0) = cos(phi/2)*cos(theta/2)*cos(si/2) + sin(phi/2)*sin(theta/2)*sin(si/2);
		Q.at<float>(1,0) = sin(phi/2)*cos(theta/2)*cos(si/2) - cos(phi/2)*sin(theta/2)*sin(si/2);
		Q.at<float>(2,0) = cos(phi/2)*sin(theta/2)*cos(si/2) + sin(phi/2)*cos(theta/2)*sin(si/2);
		Q.at<float>(3,0) = cos(phi/2)*cos(theta/2)*sin(si/2) - sin(phi/2)*sin(theta/2)*cos(si/2);

	}
	void fillBestOrientation(int i, int j)
	{
		float V[8];
		V[0] = Prob1.at<float>(i, j);
		V[1] = Prob2.at<float>(i, j);
		V[2] = Prob3.at<float>(i, j);
		V[3] = Prob4.at<float>(i, j);
		V[4] = Prob5.at<float>(i, j);
		V[5] = Prob6.at<float>(i, j);
		V[6] = Prob7.at<float>(i, j);
		V[7] = Prob8.at<float>(i, j);

		int l=0; 
		float max = -10;
		for(int k=0; k<8; k++)
		{
			if(V[k]>=max)
			{
				max = V[k];
				l = k;
			}
		}
		best_orientation.at<int>(i, j) = l+1;
		Prob_orientation.at<float>(i, j) = max;
//		cout<<"Best orientation is "<<best_orientation.at<int>(i, j)<<endl;
	}
	void getMax5Loc(Mat& M, float theta, Mat& V, Mat& X, Mat& Y)
	{
//		cout<<"In getMax5Loc"<<endl;
		Mat buff(M.size(), M.type());
		M.copyTo(buff);
		for(int i=0; i<buff.rows; i++)
			for(int j=0; j<buff.cols; j++)
			{
				float v = buff.at<float>(i, j);
				if(v>=554)
					buff.at<float>(i, j)=-1*v;
			}
		int k = theta/45;
		for(int i=0; i<5; i++)
		{
//			cout<<"i "<<i<<endl;
			cv::Point A;
			double v;
			minMaxLoc(buff, NULL, &v, NULL, &A);
	//		cout<<"Location of the max is "<<A.x<<" "<<A.y<<" with value "<<v<<endl;
			V.at<float>(k, i)=(float)v;
			X.at<float>(k, i)=A.x;
			Y.at<float>(k, i)=A.y;
			buff.at<float>(A.y, A.x) = -1;
			
		}

	}
	float fuse(float A, float B)
	{
		float t1 = A*B;
		float t2 = (1-A)*(1-B);
		return (t1/(t1+t2));
		
	}
	void multScalar(Mat& M, float c)
	{
		for(int i=0; i<M.rows; i++)
			for(int j=0; j<M.cols; j++)
			{
				float v = M.at<float>(i, j);
				if(v!=999 && v!=777 && v!=888 && v!=555)
					M.at<float>(i, j) = v/c;
			}
	}
	void multMat(Mat& M, Mat& N)
	{
		for(int i=0; i<M.rows; i++)
			for(int j=0; j<M.cols; j++)
			{
				float v = M.at<float>(i, j);
				if(v!=999 && v!=777 && v!=888 && v!=555)
					M.at<float>(i, j) = v*N.at<float>(i, j);
			}
	}
};


int main(int argc, char* argv[])
{

	Mat potential_map;
	if(argc<3)
	{
		cout<<"Enter the potential map path with the binary"<<endl;
		exit(0);
	}
		
	ros::init(argc, argv, "probability_update");
	image1_path = argv[1];
	image2_path = argv[2];
	Potential_map P;
	cout<<"Spinning"<<endl;

	ros::spin();


}
