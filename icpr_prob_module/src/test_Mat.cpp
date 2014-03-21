#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
	Mat image(5, 5, CV_32F);
	Mat image1 = imread("../Buff_1.png");
	for(int i=0; i<image.cols; i++)
		for(int j=0; j<image.rows; j++)
		{
			image.at<float>(j, i) = i*(j+2);
		}

	cout<<image.at<float>(3, 4)<<endl;
	cout<<image<<endl;
	
	rectangle(image1, cv::Point(5, 1), cv::Point(50, 50), cv::Scalar(255, 0, 0), -1);

	namedWindow("image", 1);
	imshow("image", image1);
	waitKey(0);


	return 0;
}
