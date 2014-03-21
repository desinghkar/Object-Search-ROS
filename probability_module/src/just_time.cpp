#include "just_time/just_time.h"
#include "just_vp_prob/just_vp_prob.h"

MapTimeProb::MapTimeProb(string inpath, string outpath, Point P, double theta)
{
	cout<<"In the MapTimeProb ..."<<endl;
	if(readExploredMap(inpath))
	{
		setCurrentLocation(P, theta);
		initialize();
		for(int id=0; id<360; id+=45)
			updateTimeProb(id);


	//	for(int id=0; id<ORIENT; id++)
	//		cout<<"Time["<<id<<"] at is "<<Time[id].at<double>(P.y, P.x)<<endl;
		saveResultImages(outpath);
//		writeTimeMap(outpath);
	}

}

bool MapTimeProb::readExploredMap(string path)
{
	explored_map = imread(path);
	if(explored_map.data == NULL)
	{
		cout<<"File not found at "<<path<<endl;
		return false;
	}
	m_rows = explored_map.rows;
	m_cols = explored_map.cols;
	m_size = explored_map.size();
	m_type = explored_map.type();

	return true;

}

void MapTimeProb::setCurrentLocation(Point P, double theta)
{

	current_location.x = P.x;
	current_location.y = P.y;
	current_orientation = theta; //in degrees
	cout<<"Current location in MapTimeProb is "<<current_location.x<<" ,  "<<current_location.y<<" with angle "<<current_orientation<<endl;
}

void MapTimeProb::initialize()
{
	for(int k=0; k<ORIENT; k++)
	{
		Mat Time_temp;
		Time_temp.create(m_size, CV_64F);
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
			{
				if(isGrey(explored_map, i, j))
					Time_temp.at<double>(i, j) = GREY;
				else if(isGreen(explored_map, i, j))
					Time_temp.at<double>(i, j) = GREEN;
				else if (isBlack(explored_map, i, j))
					Time_temp.at<double>(i, j) = BLACK;
				else if (isPink(explored_map, i, j))
					Time_temp.at<double>(i, j) = PINK;
				else 
					Time_temp.at<double>(i, j) = FREE;
			}
		Time.push_back(Time_temp);
	}
}

void MapTimeProb::updateTimeProb(double theta)
{
	int angle_interval = 360/ORIENT;
	int index = theta/angle_interval;
//	cout<<"Index is "<<index<<endl;
	for(int i=0; i<m_rows; i++)
		for(int j=0; j<m_cols; j++)
		{
			if(isNavigable(Time[index], i, j))
			{
				Time[index].at<double>(i, j) = timeProbability(i, j, index);
			}
				
		}
}

double MapTimeProb::timeProbability(int i, int j, int angle)
{

	double t1, t2;
	double v1 = 6, v2 = 1;
	double d1 = dist(current_location.y, current_location.x, i, j); //doubt in this stage
	double d2 = diff(current_orientation*M_PI/180, 1.0*angle*M_PI/4);
	t1 = d1/v1;
	t2 = d2/v2;

	double result = 1/(t1+t2+1);
	if(i>=169&&i<=171&&j>=169&&j<=171)
	{
//		cout<<"Current orientation is "<<current_orientation<<" and angle of patch "<<angle<<" is "<<angle*45<<" with angle in rad as"<<angle*45*M_PI/180<<endl;
//		cout<<"At "<<i<<" , "<<j<<" t1 is "<<t1<<" and t2 is "<<t2<<" and result is "<<result<<endl;
	}
//	if(t1+t2<1)
//		result = 1;
//	else
//		result = 1/(t1+t2);
	if(result==0)
		return -1;
	else
		return result;
}

void MapTimeProb::saveResultImages(string path)
{
	normalizeMat();
	
	for(int index=0; index<ORIENT; ++index)
		saveTimeImage(Time[index], path, index);
}

void MapTimeProb::normalizeMat()
{
	vector<double> V;

	Point A;
	for(int id=0; id<ORIENT; id++)
	{
		double v=0;
		Mat temp(m_size, CV_64F);
		Time[id].copyTo(temp);
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
				if(!isNavigable(temp, i, j))
					temp.at<double>(i, j)*=(-1);
		minMaxLoc(temp, NULL, &v, NULL, &A);
		V.push_back(v);
	}
//	cout<<"Size of V is "<<V.size()<<endl;

	double max = maxOf(V);

	for(int id=0; id<ORIENT; id++)
	{
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
			{
				if(isNavigable(Time[id], i, j))
				{
					Time[id].at<double>(i, j)/=max;
				}

			}
	}
}

double maxOf(vector<double> V)
{
	double max = -99;
	for(int i =0; i<V.size(); i++)
	{
//		cout<<"V["<<i<<"] is "<<V[i]<<endl;
		if(max<V[i])
			max=V[i];
	}
	return max;
}
void MapTimeProb::saveTimeImage(Mat &M, string outpath, int id)
{
	Mat image(m_rows*10, m_cols*10, m_type);
	Mat T(m_size, CV_64F);
	M.copyTo(T);
//	normalize(T);
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			int B=0, G=255, R=255, v=0;
			Point P1, P2;
			P1.x = j*SCALE; P1.y = i*SCALE;
			P2.x = (j+1)*SCALE-1; P2.y = (i+1)*SCALE-1;
			double val = Time[0].at<double>(i, j);
			if(isNavigable(Time[0], i, j))//(val!=999)&&(val!=888)&&(val!=555)&&(val!=777))
			{
				v = roundup(255*(T.at<double>(i, j)));
			}
			else if(val==999)
				B = G = R = 205; 
			else if(val==888)
				B = G = R = 0;
			else if(val==555)
			{
				B = R = 0; G = 255;
			}
			else if(val==777)
			{
				B = R = 255;
				G = 0;
			}
			rectangle(image, P1, P2, Scalar(B, G-v, R), -1);
		}	

	stringstream ss;
	ss<<id;
	string path;
	if(id==ORIENT)
		path = outpath+"ResultProb.png";
	else
		path = outpath+"Time"+ss.str()+".png";
	imwrite(path, image);
}

void MapTimeProb::writeTimeMap(string outpath)
{
	//As of now writing the file so as to read in the Matlab and display the plot
	for(int id=0; id<ORIENT; id++)
	{
		ofstream file;
		stringstream ss;
		ss<<id;
		string path = outpath+"Time"+ss.str()+".txt";
		file.open(path.c_str());

		for(int i=0; i<m_rows; i++)
		{
			for(int j=0; j<m_cols; j++)
			{
				if(isNavigable(Time[id], i, j))
					file<<Time[id].at<double>(i, j)<<", ";
				else
					file<<"0.0, ";
			}
			file<<";"<<endl;
		}
	}
/*	
	ofstream file1;
	string path1 = outpath+"ResultTime.txt";
	file1.open(path1.c_str());
	for(int i=0; i<m_rows; ++i)
	{
		for(int j=0; j<m_cols; ++j)
			file1<<result_time.at<double>(i,j)<<", ";
		file1<<";"<<endl;
	}
*/
}

void MapTimeProb::normalize(Mat& M)
 {
//	cout<<"In normaize"<<endl;
	double v; Point A;
	Mat M_temp(m_size, CV_64F);
	M.copyTo(M_temp);
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
			if(!isNavigable(M_temp, i, j))
				M_temp.at<double>(i, j)*=(-1);

	minMaxLoc(M_temp, NULL, &v, NULL, &A);
//	cout<<"V is "<<v<<endl;
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			if(isNavigable(M, i, j))
				M.at<double>(i, j)/=v;

		}
}

vector<Mat> MapTimeProb::getResultTime()
{
	return Time;

}

double diff(double A1, double A2)
{ 
	double diff = abs(A1 - A2);
	if(diff>M_PI || diff>(2*M_PI))
		diff = 2*M_PI - diff;
	return abs(diff);
}

/*
int main(int ac, char* av[])
{
	MapTimeProb T(av[1], av[2], Point(170, 170), 90);
	vector<Mat> Time = T.getResultTime();

	MapViewProb P(av[1], av[2]);
	vector<Mat> VP = P.getResultProb();

	for(int id=0; id<ORIENT; id++)
	{
		cout<<"Time["<<id<<"] at 200, 220 is "<<Time[id].at<double>(200, 220)<<endl;
		cout<<"Prob["<<id<<"] at 200, 220 is "<<VP[id].at<double>(200, 220)<<endl;
	}
	
	return 0;
}
*/
