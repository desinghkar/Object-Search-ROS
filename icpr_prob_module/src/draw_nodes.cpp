#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include <math.h>
#define SCALE 2
#define RANGE 40
#define PI 3.1416
using namespace std;
using namespace cv;
int kcount=0;
int item_no=0;
ofstream fout("locations_with_time.txt");
int seen[10] = {0,0,0,0,0,0,0,0,0,0};
class draw
{
	public:
		Mat image;
		Mat IMAGE;
		string image_path;
		string file_path;
		int Bl, Gl, Rl;
		int Bc, Gc, Rc;
		draw(string image_p, string file_p)
		{

			image_path = image_p;
			file_path = file_p;
			readImage();
			ifstream file1, file2;
			file1.open(file_path.c_str());
			file2.open(file_path.c_str());
			scaleUp();
			float x1, y1, x2, y2;
			file1>>x1; 
			file1>>y1;
			int count=0;
			while(!file1.eof())
			{
				float temp;
				file1>>temp;
				file1>>temp;
				file1>>x2; 
				file1>>y2;
				cv::Point P1;
				P1.x =(int)x1*SCALE; P1.y=(int)y1*SCALE;
				cv::Point P2;
				P2.x=(int)(x1+1)*SCALE-1; P2.y=(int)(y1+1)*SCALE-1;
				cv::Point P((P1.x+P2.x)/2, (P1.y+P2.y)/2);
				cv::Point Q1;
				Q1.x =(int)x2*SCALE; Q1.y=(int)y2*SCALE;
				cv::Point Q2;
				Q2.x=(int)(x2+1)*SCALE-1; Q2.y=(int)(y2+1)*SCALE-1;
				cv::Point Q((Q1.x+Q2.x)/2, (Q1.y+Q2.y)/2);
				line(IMAGE, P, Q, cv::Scalar(Bl, Gl, Rl), 1);
				x1=x2;
				y1=y2;
				namedWindow("kar");
				imshow("kar", IMAGE);
				waitKey(0);


			}

			while(!file2.eof())
			{
//				cout<<"In while "<<endl;
				float x, y; float th, tem;
				file2>>x;
				file2>>y;
				file2>>th;
				file2>>tem;
				float time = funTime(tem);
				time = time - count*30.0;
				fout<<x<<" "<<y<<" "<<th<<" "<<time<<endl;
				printObjectSeen(x, y, th);
				cout<<x<<" "<<y<<" "<<th<<" "<<time<<endl;
				//cout<<x<<" "<<y<<" "<<" "<<th<<endl;
				cv::Point P1;
				P1.x =(int)x*SCALE; P1.y=(int)y*SCALE;
				cv::Point P2;
				P2.x=(int)(x+1)*SCALE-1; P2.y=(int)(y+1)*SCALE-1;
				cv::Point P;
				P.x = (P1.x+P2.x)/2; P.y= (P1.y+P2.y)/2;
				circle(IMAGE, P, 5, cv::Scalar(Bc, Gc, Rc), -1);
				cv::Point Q(P.x+10*cos(th*M_PI/180), P.y-10*sin(th*M_PI/180));
				line(IMAGE, P, Q, cv::Scalar(Bl, Gl, Rl));
				count++;
				kcount++;
			}

			namedWindow("image");
			namedWindow("IMAGE");
			imshow("image", image);
			imshow("IMAGE", IMAGE);
			imwrite("drawn_image.png", IMAGE);
			waitKey(0);

		}
		double dist(int a1, int b1, int a2, int b2)
		{
			double d = 1.0*(a1-a2)*(a1-a2) + 1.0*(b1-b2)*(b1-b2);
			return sqrt(d);
		}
		draw(string image_p, string file_p, bool flag)
		{
			image_path = image_p;
			file_path = file_p;
			readIMAGE();
			ifstream file1, file2;
			file1.open(file_path.c_str());
			file2.open(file_path.c_str());
			float x1, y1, x2, y2;
			file1>>x1; 
			file1>>y1;
			int count=0;
			while(!file1.eof())
			{

				float temp;
				file1>>temp;
				file1>>temp;
				file1>>x2; 
				file1>>y2;
				cv::Point P1;
				P1.x =(int)x1*SCALE; P1.y=(int)y1*SCALE;
				cv::Point P2;
				P2.x=(int)(x1+1)*SCALE-1; P2.y=(int)(y1+1)*SCALE-1;
				cv::Point P((P1.x+P2.x)/2, (P1.y+P2.y)/2);
				cv::Point Q1;
				Q1.x =(int)x2*SCALE; Q1.y=(int)y2*SCALE;
				cv::Point Q2;
				Q2.x=(int)(x2+1)*SCALE-1; Q2.y=(int)(y2+1)*SCALE-1;
				cv::Point Q((Q1.x+Q2.x)/2, (Q1.y+Q2.y)/2);
				line(IMAGE, P, Q, cv::Scalar(Bl, Gl, Rl), 1);
				x1=x2;
				y1=y2;


			}
			while(!file2.eof())
			{
//				cout<<"In while "<<endl;
				float x, y; float th, tem;
				file2>>x;
				file2>>y;
				file2>>th;
				file2>>tem;
				float time = funTime(tem);
				time = time - count*30.0;
				fout<<x<<" "<<y<<" "<<th<<" "<<time<<endl;
				cout<<x<<" "<<y<<" "<<th<<" "<<time<<endl;

				printObjectSeen(x, y, th);
//				cout<<x<<" "<<y<<" "<<" "<<th<<endl;
				cv::Point P1;
				P1.x =(int)x*SCALE; P1.y=(int)y*SCALE;
				cv::Point P2;
				P2.x=(int)(x+1)*SCALE-1; P2.y=(int)(y+1)*SCALE-1;
				cv::Point P;
				P.x = (P1.x+P2.x)/2; P.y= (P1.y+P2.y)/2;
				circle(IMAGE, P, 5, cv::Scalar(Bc, Gc, Rc), -1);
				cv::Point Q(P.x+10*cos(th*M_PI/180), P.y-10*sin(th*M_PI/180));
				line(IMAGE, P, Q, cv::Scalar(Bl, Gl, Rl));
				count++;
				kcount++;
			}

			//			namedWindow("image");
			namedWindow("IMAGE");
			//			imshow("image", image);
			imshow("IMAGE", IMAGE);

			imwrite("drawn_image.png", IMAGE);
			waitKey(0);
		}
		void printObjectSeen(int x, int y, float th)
		{
			PatchTraceAt(x, y, th);
		}
		void PatchTraceAt(int x, int y, float theta)
		{
			//		cout<<"In PatchTrace"<<endl;
			float min_theta =(theta - 45);
			float max_theta =(theta + 45);
			int count = 0;
//			float s = max_theta;
			for(int s=min_theta; s<max_theta; s++)
			{

				int x2 = x+RANGE*cos(s*PI/180.0);
				int y2 = y-RANGE*sin(s*PI/180.0);
				int x1 = x;
				int y1 = y;

				count+=rayTrace2D(x1, y1, x2, y2);

			}			
//			return count;
		}
		int rayTrace2D(int x1, int y1, int x2, int y2)
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
				xi1 = (int)x; xi2 = (xi1<image.cols - 1)? xi1+1: xi1;
				yi1 = (int)y; yi2 = (yi1<image.rows - 1)? yi1+1: yi1;

				dx = x - xi1; dy = y - yi1;
				int kx, ky;
				if(dx < 0.5)
					if(dy < 0.5) {
						B = image.at<Vec3b>(yi1, xi1)[0]; 
						G = image.at<Vec3b>(yi1, xi1)[1];
						R = image.at<Vec3b>(yi1, xi1)[2];
						kx = xi1;
						ky = yi1;
					}
					else         {
						B = image.at<Vec3b>(yi2, xi1)[0]; 
						G = image.at<Vec3b>(yi2, xi1)[1];
						R = image.at<Vec3b>(yi2, xi1)[2];
						kx = xi1;
						ky = yi2;
					}
				else
					if(dy < 0.5) {
						B = image.at<Vec3b>(yi1, xi2)[0]; 
						G = image.at<Vec3b>(yi1, xi2)[1];
						R = image.at<Vec3b>(yi1, xi2)[2];
						kx = xi2;
						ky = yi1;
					}
					else {
						B = image.at<Vec3b>(yi2, xi2)[0]; 
						G = image.at<Vec3b>(yi2, xi2)[1];
						R = image.at<Vec3b>(yi2, xi2)[2];
						kx = xi2;
						ky = yi2;
					}
#if 0
				updatedPotentialMap.at<Vec3b>(ky, kx)[0] = 255;
				updatedPotentialMap.at<Vec3b>(ky, kx)[1] = 0;=
					updatedPotentialMap.at<Vec3b>(ky, kx)[2] = 255;
#endif
				if(B==0&&G==255&&R==0)
				{
					count++;
//	//				updatedPotentialMap.at<Vec3b>(ky, kx)[0] = 255;
		//			updatedPotentialMap.at<Vec3b>(ky, kx)[1] = 0;
		//			updatedPotentialMap.at<Vec3b>(ky, kx)[2] = 255;


					//				cout<<"Green "<<count<<endl;
				}

				if(object(ky, kx))
				{
					cout<<x1<<" "<<y1<<" "<<item_no<<" "<<kcount<<endl;
				}
				else if(B==0&&G==0&&R==0)
					return count;
				x+= x_step;
				y+= y_step;


			}
			return count;

		}
		bool object(int i, int j)
		{
			int B = image.at<Vec3b>(i, j)[0]; 
			int G = image.at<Vec3b>(i, j)[1];
			int R = image.at<Vec3b>(i, j)[2];
			for(int i=25; i<255; i+=25)
			{
				if(B==0&&G==0&&R==i)
				{
					item_no = i/25;
					if(seen[item_no-1]==0)
					{
						cout<<"Object no "<<item_no<<" found "<<endl;
						seen[item_no-1]=1;
						return true;
					}

				}
			}
			return false;

		}

		float funTime(float m)
		{
			int t1 = (int)m;
			float s1 = m-1.0*t1;
			float t2 = t1*60 + s1*100;
			return t2;

		}
		void readColors()
		{	
			cout<<"Enter the colors of the nodes"<<endl;
			cin>>Bc;
			cin>>Gc;
			cin>>Rc;
			cout<<"Enter the colors of the lines"<<endl;
			cin>>Bl;
			cin>>Gl;
			cin>>Rl;


		}
		void readImage()
		{
			image = imread(image_path);
			readColors();
		}
		void readIMAGE()
		{
			IMAGE = imread(image_path);
			readColors();
		}
		void scaleUp()
		{
			int row = image.rows*SCALE;
			int col = image.cols*SCALE;
			IMAGE.create(row, col, image.type());
			for(int i=0; i<image.rows; i++)
				for(int j=0; j<image.cols; j++)
				{
					int x=j, y=i;
					int b, g, r;
					cv::Point P1;
					P1.x =x*SCALE; P1.y=y*SCALE;
					cv::Point P2;
					P2.x=(x+1)*SCALE-1; P2.y=(y+1)*SCALE-1;
					rectangle(IMAGE, P1, P2, cv::Scalar(image.at<Vec3b>(i, j)[0], image.at<Vec3b>(i, j)[1], image.at<Vec3b>(i, j)[2]), -1);

					cout<<image.at<Vec3b>(i, j)[0]<<" "<<image.at<Vec3b>(i, j)[1]<<" "<<image.at<Vec3b>(i, j)[2]<<endl;
				}
		}

};
int main(int argc, char* argv[])
{
	if(argc<3)
	{
		cout<<"Enter the path of the image and path of the file"<<endl;
		return 0;
	}
	string image_p = argv[1];
	string file_p = argv[2];
	stringstream temp;
	temp<<argv[3];
	bool f;
	temp>>f;
//	temp.clear();
	if(argc==4)
		draw d(image_p, file_p,f);
	else
		draw d(image_p, file_p);


	return 0;
}
