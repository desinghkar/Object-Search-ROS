#include "localglobalmap/localglobalmap.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
string path_1, path_2;

ofstream file;

bool isGreen(Mat &M, int i, int j)
{ 
	if(M.at<Vec3b>(i, j)[0]==0 && M.at<Vec3b>(i, j)[1]==255 && M.at<Vec3b>(i, j)[2]==0)
		return true;
	return false;
}
bool isBlue(Mat &M, int i, int j)
{
	if(M.at<Vec3b>(i, j)[0]==255 && M.at<Vec3b>(i, j)[1]==0 && M.at<Vec3b>(i, j)[2]==0)
		return true;
	return false;
}
bool isBrown(Mat &M, int i, int j)
{
	if(M.at<Vec3b>(i, j)[0]==41 && M.at<Vec3b>(i, j)[1]==41 && M.at<Vec3b>(i, j)[2]==94)
		return true;
	return false;
}

void updatePotentialMap()
{

	LocalGlobalMap G;
	if(!G.readMap(path_1))
		return ;

	Mat M = imread(path_1);
	for(int i=0; i<M.rows; i++)
		for(int j=0; j<M.cols; j++)
		{
			if(isGreen(M, i, j)||isBrown(M, i, j)||isBlue(M, i, j))
			{
				//convert the map.x, map.y to geometry_msgs.x and geometry_msgs.y
				G.setLocalToGlobal(Point(j-4, i+5), 0);

				geometry_msgs::Pose g = G.getGlobal();
				if(isGreen(M, i, j))
					g.position.z = 0.5;
				if(isBrown(M, i, j))
					g.position.z = 0.25;
				if(isBlue(M, i, j))
					g.position.z = 1.0;
				file<<g.position.x<<" "<<g.position.y<<" "<<g.position.z<<endl;
			}

		}
}



int main(int ac, char** av)
{
	if(ac<2)
	{
		cout<<"Enter the path of map"<<endl;
		return -1;
	}
	file.open("pcd.dat");
	path_1 = av[1];
	updatePotentialMap();
	file.close();
	return 0;
}
