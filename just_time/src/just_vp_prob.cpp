#include "just_vp_prob/just_vp_prob.h"

#define GREY 999
#define GREEN 555
#define PINK 777
#define BLACK 888
#define FREE 0
#define ORIENT 8

#define RANGE 30
#define SCALE 10

MapViewProb::MapViewProb(string inpath, string outpath)
{
	if(readExploredMap(inpath))
	{
		if(findTableArea())
		{
			initializeProb();
			for(int i=0; i<360; i+=45)
				updateVPProb(i);
			resultProbability();
			top10Loc();
			writeProbMap(outpath);
			saveResultImages(outpath);
		}

	}
}

bool MapViewProb::readExploredMap(string path)
{
	explored_map = imread(path);
	if(explored_map.data==NULL)
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

bool MapViewProb::findTableArea()
{
	Total_tableA = 0;
	for(int i=0; i<m_rows; i++)
		for(int j=0; j<m_cols; j++)
		{
			if(isGreen(explored_map, i, j))
				Total_tableA++;
		}
	cout<<"TotalA in the input map is "<<Total_tableA<<endl;
	if(Total_tableA<5)
	{
		cout<<"All table regions are explored!!!!! Object in search is not found!!! :( "<<endl;
		return false;
	}
	return true;
}

void MapViewProb::initializeProb()
{

	for(int k=0; k<ORIENT; k++)
	{
		Mat Prob_temp;
		Prob_temp.create(m_size, CV_64F);
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
			{
				if(isGrey(explored_map, i, j))
					Prob_temp.at<double>(i, j) = GREY; //Grey
				else if(isGreen(explored_map, i, j))
					Prob_temp.at<double>(i, j) = GREEN; //Green
				else if(isBlack(explored_map, i, j))
					Prob_temp.at<double>(i, j) = BLACK; //Black
				else if(isPink(explored_map, i, j))
					Prob_temp.at<double>(i, j) = PINK; //Pink
				else
					Prob_temp.at<double>(i, j) = FREE; //Otherwise
			}

		Prob.push_back(Prob_temp);
	}
	//	cout<<"Size of the Prob vector is "<<Prob.size()<<endl;
}

void MapViewProb::updateVPProb(double theta)
{
	int angle_interval = 360/ORIENT;
	int index = theta/angle_interval;
	cout<<"Index is "<<index<<endl;
	for(int i=0; i<m_rows; i++)
		for(int j=0; j<m_cols; j++)
		{
			if(isNavigable(Prob[index], i, j))
			{
				double ViewA = 1.0*patchTrace(j, i, theta);
				Prob[index].at<double>(i, j) = ViewA/Total_tableA;
			}
		}
}

int MapViewProb::patchTrace(int x, int y, double theta)
{	
	double min_theta =(theta - 45);
	double max_theta =(theta + 45);
	int count = 0;
	Mat buff_map(m_size, m_type);
	explored_map.copyTo(buff_map);
	for(double si=min_theta; si<max_theta; si++)
	{
		int x2 = x + RANGE*cos(si*M_PI/180);
		int y2 = y - RANGE*sin(si*M_PI/180);
		int x1 = x;
		int y1 = y;
		count+=rayTrace2D(buff_map,x1, y1, x2, y2);

	}

	return count;
}

int MapViewProb::rayTrace2D(Mat & buff_map, int x1, int y1, int x2, int y2)
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
			buff_map.at<Vec3b>(ky, kx)[0] = 205; 
			buff_map.at<Vec3b>(ky, kx)[1] = 205;
			buff_map.at<Vec3b>(ky, kx)[2] = 205;
		}
		if(isBlack(buff_map, ky, kx))
			return count;
		x+= x_step;
		y+= y_step;
	}
	return count;
}

void MapViewProb::top10Loc()
{
	Mat temp_v, temp_x, temp_y;
	temp_v.create(8, 10, CV_64F);
	temp_x.create(8, 10, CV_64F);
	temp_y.create(8, 10, CV_64F);

	for(int index=0; index<ORIENT; ++index)
	{
		Mat buff(m_size, m_type);
		Prob[index].copyTo(buff);
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
			{
				if(!isNavigable(buff, i, j))
					buff.at<double>(i, j)=-1*buff.at<double>(i, j);
			}
		for(int i=0; i<10; ++i)
		{
			cv::Point A; double v;
			minMaxLoc(buff, NULL, &v, NULL, &A);
			temp_v.at<double>(index, i) = v;
			temp_x.at<double>(index, i) = A.x;
			temp_y.at<double>(index, i) = A.y;
			buff.at<double>(A.y, A.x) = -1;
		}
		
	}
	Top.push_back(temp_v);
	Top.push_back(temp_x);
	Top.push_back(temp_y);
}

void MapViewProb::resultProbability()
{
	result_prob.create(m_size, CV_64F);
	for(int i=0; i<m_rows; i++)
		for(int j=0; j<m_cols; j++)
		{
			double v_max = 0;
			if(isNavigable(Prob[0], i, j))
			{
				for(int id=0; id<ORIENT; id++)
				{
					if(v_max<Prob[id].at<double>(i, j))
						v_max = Prob[id].at<double>(i, j);
				}
			}
			result_prob.at<double>(i, j) = v_max;
		}
}


void MapViewProb::writeProbMap(string outpath)
{
	//As of now writing the file so as to read in the Matlab and display the plot
	for(int id=0; id<ORIENT; id++)
	{
		ofstream file;
		stringstream ss;
		ss<<id;
		string path = outpath+"Prob"+ss.str()+".txt";
		file.open(path.c_str());

		for(int i=0; i<m_rows; i++)
		{
			for(int j=0; j<m_cols; j++)
			{
				if(isNavigable(Prob[id], i, j))
					file<<Prob[id].at<double>(i, j)<<", ";
				else
					file<<"0.0, ";
			}
			file<<";"<<endl;
		}
	}
	ofstream file1;
	string path1 = outpath+"ResultProb.txt";
	file1.open(path1.c_str());
	for(int i=0; i<m_rows; ++i)
	{
		for(int j=0; j<m_cols; ++j)
			file1<<result_prob.at<double>(i,j)<<", ";
		file1<<";"<<endl;
	}

}

void MapViewProb::saveResultImages(string outpath)
{	
	for(int index=0; index<ORIENT; ++index)
	{
		saveProbImage(Prob[index], outpath, index);
	}
	saveProbImage(result_prob, outpath, ORIENT);
}

void MapViewProb::saveProbImage(Mat& M, string outpath, int id)
{
	Mat image(m_rows*10, m_cols*10, m_type);
	Mat P(m_size, CV_64F);
	M.copyTo(P);
	normalize(P);
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			int B=0, G=255, R=255, v=0;
			Point P1, P2;
			P1.x = j*SCALE; P1.y = i*SCALE;
			P2.x = (j+1)*SCALE-1; P2.y = (i+1)*SCALE-1;
			double val = Prob[0].at<double>(i, j);
			if(isNavigable(Prob[0], i, j))//(val!=999)&&(val!=888)&&(val!=555)&&(val!=777))
			{
				v = roundup(255*(P.at<double>(i, j)));
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
		path = outpath+"Prob"+ss.str()+".png";
	imwrite(path, image);


}

void MapViewProb::normalize(Mat& M)
{
	cout<<"In normaize"<<endl;
	double v; Point A;
	Mat M_temp(m_size, CV_64F);
	M.copyTo(M_temp);
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
			if(!isNavigable(M_temp, i, j))
				M_temp.at<double>(i, j)*=(-1);

	minMaxLoc(M_temp, NULL, &v, NULL, &A);
	cout<<"V is "<<v<<endl;
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			if(isNavigable(M, i, j))
				M.at<double>(i, j)/=v;

		}
}
double MapViewProb::getTotalArea()
{
	return Total_tableA;
}

vector<Mat> MapViewProb::getResultProb()
{
	return Prob;
}
double dist(int a1, int b1, int a2, int b2)
{
	double d = 1.0*(a1-a2)*(a1-a2) + 1.0*(b1-b2)*(b1-b2);
	return sqrt(d);
}
int roundup(double v)
{
	int buff = v;
	if(v-buff>=0.5)
		return buff+1;
	else 
		return buff;
}
bool isGreen(Mat M, int i, int j)
{
	if(M.at<Vec3b>(i, j)[0]==0 && M.at<Vec3b>(i, j)[1]==255 && M.at<Vec3b>(i, j)[2]==0)
		return true;
	return false;
}
bool isBlack(Mat M, int i, int j)
{
	if(M.at<Vec3b>(i, j)[0]==0 && M.at<Vec3b>(i, j)[1]==0 && M.at<Vec3b>(i, j)[2]==0)
		return true;
	return false;
}
bool isPink(Mat M, int i, int j)
{
	if(M.at<Vec3b>(i, j)[0]==255 && M.at<Vec3b>(i, j)[1]==0 && M.at<Vec3b>(i, j)[2]==255)
		return true;
	return false;
}
bool isGrey(Mat M, int i, int j)
{
	if(M.at<Vec3b>(i, j)[0]==205 && M.at<Vec3b>(i, j)[1]==205 && M.at<Vec3b>(i, j)[2]==205)
		return true;
	return false;
}
bool isFree(Mat M, int i, int j)
{
	if((!isGreen(M, i, j)) && (!isBlack(M, i, j)) && (!isPink(M, i, j)) && (!isGrey(M, i, j)))
		return true;
	return false;
}

bool isNavigable(Mat M, int i, int j)
{
	double v = M.at<double>(i, j);
	if(v!=999 && v!=555 && v!=888 && v!=777)
		return true;
	return false;
}
