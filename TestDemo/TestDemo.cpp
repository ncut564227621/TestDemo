// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"
#include<string>
#include "lsd.h"



int _tmain(int argc, _TCHAR* argv[])
{
	Mat img = imread("..\\Sample\\build.jpg",CV_LOAD_IMAGE_COLOR);//image size:720*580 px

	Mat _inImg(img.size(), CV_8UC1, Scalar(0));
	Mat _outImg(img.size(), CV_8UC1, Scalar(0));

	if(img.empty())
	{
		cout<<"input img is wrong!"<<endl;
		return -1;
	}
	if(img.channels() == 3)
	{
		cvtColor(img, _inImg, CV_RGB2GRAY);
	}
	else
	{
		  _inImg = img.clone();
	}
	

	

	for(int i =0; i<10; i++)
	{
		
	   Ptr<	LineSegmentDetector> pLsd = createLineSegmentDetector(LSD_REFINE_NONE,0.8,0.6,2.0,22.5,0,0.7,1024);
		vector<Vec4i>vecLinesParams;
		const int64 nStart = getTickCount();

		//to do something;
		
	     pLsd->detect(_inImg, vecLinesParams);
		 pLsd->drawSegments(img, vecLinesParams); //

		double nDuration = (getTickCount() - nStart)/getTickFrequency();
		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;
		imwrite("..\\sample\\result\\opencv_lsd_line.bmp", img);
		int a =0;
	}

	

	return 0;
}



