// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"


int _tmain(int argc, _TCHAR* argv[])
{
	Mat img = imread("..\\Sample\\qipange.jpg",CV_LOAD_IMAGE_COLOR);//image size:720*580 px
	
	for(int i =0; i<10; i++)
	{
		Mat inImg(img.size(), CV_8UC1, Scalar(0));
		Mat magnImg;
		Mat _outImg(img.size(), CV_8UC1, Scalar(0));
		cvtColor(img, inImg, CV_RGB2GRAY);

		int nRadius = inImg.cols>>3;
		int nRatio = 15;

		const int64 nStart = getTickCount();
		uchar ucThreshold =0;
		//otsuThreshold(inImg, _outImg, ucThreshold);
		threshold(inImg, _outImg, 0, 255, CV_THRESH_OTSU);
		//wellnerThresholdEx(inImg.data, _outImg.data, inImg.cols, inImg.rows);
		//wellnerThreshold(inImg, _outImg, nRadius, nRatio);
		double nDuration = (getTickCount() - nStart)/getTickFrequency();
		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;

		//normalize(_outImg, _outImg, 0, 255, CV_MINMAX);
		//imwrite("..\\sample\\result\\convolveTIME.BMP", _outImg);
		imwrite("..\\sample\\result\\wellner_threshold.bmp", _outImg);
		int a =0;
	}

	return 0;
}



