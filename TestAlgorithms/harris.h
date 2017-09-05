#pragma  once
#include"opencv2/opencv.hpp"
#include<iostream>
#include"ippi.h"
#include<vector>

using namespace cv;
using namespace std;


#ifdef TESTALGORITHMS_EXPORTS
#define TESTALGORITHMS_API __declspec(dllexport)
#else
#define TESTALGORITHMS_API __declspec(dllimport)
#endif


void drawCross(Mat img, Point pt, Scalar color, int size=2, int thickness=1);

class TESTALGORITHMS_API harris
{
public:
	Mat cornerStrength; //harris函数检测结果，每个像素角点响应函数数值
	Mat localMax;
	Mat cornerThd;
	int nNeighbourhood; //邻域窗口大小
	int nAperture;
	double k;
	double maxStrength;//角点响应最大值
	double threshold;
	int nonMaxSize;
	Mat kernel;
public:
	harris():nNeighbourhood(3),nAperture(3),maxStrength(0.0), threshold(0.01), nonMaxSize(3),k(0.01)
	{

	}
	void detect(Mat img);
	Mat getCornerMap(double qualityLevel);


	void getCorners(vector<Point>&pts, Mat&cornerMap);
	void corners(vector<Point>&pts, double qualityLevel);

	void drawCorners(const vector<Point>& pts, Mat src, Scalar color = Scalar(255,255,0), int radius =3, int thickness=2);

	void shi_TomasHarrisDetect(Mat img, vector<Point>&pts, const int maxCornerNum, const int minDistance, double qualityLevel);

	void fastHarris(Mat img, vector<Point>&pts, int threshold, bool bSuppression);
};