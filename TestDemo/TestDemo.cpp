// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"


int _tmain(int argc, _TCHAR* argv[])
{
	Mat img = imread("..\\Sample\\binggan.bmp",CV_LOAD_IMAGE_GRAYSCALE);

	for(int i =0; i<20; i++)
	{
		Mat inImg = img.clone();
		vector<vector<Point>>contours;
		const int64 nStart = getTickCount();
		int nConnNums = DetectConnFindContours(inImg, contours, 200, true);
		double nDuration = (getTickCount() - nStart)/getTickFrequency();
		cout<<"cost time: "<<nDuration<<endl;
	}
	

	
	return 0;
}



