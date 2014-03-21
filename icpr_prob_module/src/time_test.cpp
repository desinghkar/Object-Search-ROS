#include <iostream>
#include <time.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main()
{
	ofstream file("time.txt");
	clock_t start, end;
	int key;
	start = clock();
	while(1)
	{
		namedWindow("display");
		key = waitKey(3);
		if(key==1048675)
		{
			end = clock() - start;
			cout<<"time taken is "<<(double)end/(double)CLOCKS_PER_SEC<<endl;
			start = clock();
			
		}
	}
	return 0;

}
