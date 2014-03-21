#include "fuseProbabilities/fuseProbabilities.h"

FuseProb::FuseProb(vector<Mat> P, vector<Mat> T, Mat& M, int o, string path)
{
	cout<<"Insite the FuseProb class"<<endl;
	m_rows = M.rows; m_cols = M.cols; 
	m_type = M.type(); m_size = M.size();
	orient = o;
	cout<<"rows cols type "<<m_rows<<" "<<m_cols<<endl;
	readProbTime(P, T);
//	normalizeProbs(); //This is wrong and hence commented... normalization should happen across the 8 orientations
	multProbTime();
	saveImages(path);
}

void FuseProb::readProbTime(vector<Mat> P, vector<Mat> T)
{
	cout<<"Size of P is "<<P.size()<<endl;
	cout<<"rows and cols are "<<P[0].rows<<" "<<P[0].cols<<endl;
	
	for(int id=0; id<orient; id++)
	{
		cout<<"At id"<<id<<endl;
		Mat temp_p(m_rows, m_cols, CV_64F);
		Mat temp_t(m_rows, m_cols, CV_64F);
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
			{
		//		cout<<"At i="<<i<<" j="<<j<<endl;
				temp_p.at<double>(i, j) = P[id].at<double>(i, j);
				temp_t.at<double>(i, j) = T[id].at<double>(i, j);
			}
		Prob.push_back(temp_p);
		Time.push_back(temp_t);
	}
}

void FuseProb::normalizeProbs() //This is not being used
{
	for(int id=0; id<orient; id++)
	{
		normalize(Prob[id]);
		normalize(Time[id]);
	}
}

void FuseProb::normalize(Mat& M) //Used only to display the images properly
{
	cout<<"In normalize"<<endl;
	double v; 
	Point A;
	Mat M_temp(m_rows, m_cols, CV_64F);
	M.copyTo(M_temp);
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			if(!isNavigable(M_temp, i, j))
				M_temp.at<double>(i, j)*=(-1);
		}

	minMaxLoc(M_temp, NULL, &v, NULL, &A);
	cout<<"V is "<<v<<endl;
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			if(isNavigable(M, i, j))
				M.at<double>(i, j)/=v;

		}

	cout<<"Done in normalize"<<endl;
}
void FuseProb::multProbTime()
{
	for(int id=0; id<orient; id++)
	{
		Mat temp(m_size, CV_64F);
		for(int i=0; i<m_rows; i++)
			for(int j=0; j<m_cols; j++)
			{
				if(isNavigable(Prob[id], i, j))
				{
					double t, p;
					t = Time[id].at<double>(i, j);
					p = Prob[id].at<double>(i, j);
//					t = (t+1)/2;
					temp.at<double>(i, j) = t*p;//( t*p + (1-t)*(1-p) );
//					temp.at<double>(i, j) = Prob[id].at<double>(i, j) * Time[id].at<double>(i, j);//((Time[id].at<double>(i, j)+1)/2);

				}
			}
		ResultProb.push_back(temp);
	}
	cout<<"Size of the ResultProb after multProbTime is "<<ResultProb.size()<<endl;
}

vector<Mat> FuseProb::getResultFusedProb()
{
	return ResultProb;
}

void FuseProb::saveImages(string outpath)
{
	for(int index=0; index<orient; ++index)
	{
		cout<<"Saving the image "<<index<<endl;
		saveFusedImage(ResultProb[index], outpath, index);
	}
}
void FuseProb::saveFusedImage(Mat& M, string outpath, int id)
{
	Mat image(m_rows*10, m_cols*10, m_type);
	Mat Res(m_size, CV_64F);
	M.copyTo(Res);
//	normalize(Res);
	for(int i=0; i<m_rows; ++i)
		for(int j=0; j<m_cols; ++j)
		{
			int B=0, G=255, R=255, v=0;
			Point P1, P2;
			P1.x = j*SCALE; P1.y = i*SCALE;
			P2.x = (j+1)*SCALE-1; P2.y = (i+1)*SCALE-1;
			double val = Prob[0].at<double>(i, j);
			if(isNavigable(Prob[0], i, j))
				v = roundup(255*(Res.at<double>(i, j)));
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
				B = R = 255; G = 0;
			}
			rectangle(image, P1, P2, Scalar(B, G-v, R), -1);
		}
	stringstream ss;
	ss<<id;
	string path = outpath+"ResultFuseProb"+ss.str()+".png";
	imwrite(path, image);
}

void FuseProb::computeGoal()
{
	double maxProb = 0;
	for(int id=0; id<orient; ++id)
	{
		double id_max;
		Point id_goal;
		minMaxLoc(ResultProb[id], NULL, &id_max, NULL, &id_goal); //Doubt on the x, y and y, x
		if(maxProb<id_max)
		{
			maxProb = id_max;
			goal.x = id_goal.x;
			goal.y = id_goal.y;
			goal_orientation = id*45;
		}
	}
}

Point FuseProb::getGoalPosition()
{
	return goal;
}

double FuseProb::getGoalOrientation()
{
	return goal_orientation;
}
/*
int main(int ac, char** av)
{
	MapViewProb VP_M(av[1], av[2]);
	MapTimeProb T_M(av[1], av[2], Point(138, 98), 90);
	Mat img = imread(av[1]);
	FuseProb FP(VP_M.getResultProb(), T_M.getResultTime(), img, 8, av[2]);
	FP.computeGoal();
	Point g = FP.getGoalPosition();
	double gd = FP.getGoalOrientation();

	cout<<"goal is "<<g.x<<" "<<g.y<<" with orientation "<<gd<<endl;
	
	return 0;
}
*/
