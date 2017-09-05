// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"
#include<string>
#include "lsd.h"

#include "CircleFitByHyper.h"
#include "floodFill.h"
#include "ParseCSV.h"
#include "MeanShift.h"
#include "../TestAlgorithms/harris.h"


void randomlineData(Mat& fitMap, vector<Point>& fitData, vector<Point>& fitData_2, const int ptNums);

int _tmain(int argc, _TCHAR* argv[])
{
	
	for(int i =0; i<10; i++)

	{
		Mat img = imread("..\\Sample\\camera.bmp",CV_LOAD_IMAGE_COLOR);//image size:720*580 px
		harris Harris;
		Mat gray;
		cvtColor(img, gray, CV_RGB2GRAY);
		const int64 nStart = getTickCount();
		//to do something

		//Harris.detect(gray);

		vector<Point>pts;

		Harris.fastHarris(gray, pts, 100, true);
		
		Harris.drawCorners(pts, img);
		
		double nDuration = (getTickCount() - nStart)/getTickFrequency();

		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;

	}

	

	return 0;
}

void randomlineData(Mat& fitMap, vector<Point>& fitData_1,vector<Point>& fitData_2, const int ptNums)
{
	if(fitMap.empty())
	{
		cout<<"fitData Mat is empty!"<<endl;
		return;
	}
	int a_1 = 2;
	int b_1 = 10;

	int a_2 = -1;
	int b_2 = 350;
	RNG rng(0xFFFFFFFF);
	RNG rng_eps(0xFFFFFFFF);

	for(int i =0; i<ptNums; i++)
	{
		Point pt_1;
		Point pt_2;
		
		pt_2.x = rng.uniform(3, fitMap.cols-3);
		pt_2.y = a_2*pt_2.x+b_2+rng_eps.gaussian(7);
		if(i%10 == 0)
			pt_2.y = a_2*(pt_2.x-rng_eps.gaussian(15))+b_2+rng_eps.gaussian(50);
		if(pt_2.y<=fitMap.rows-1&& pt_2.y>=0)
		{
			fitData_2.push_back(pt_2);
			fitMap.at<Vec3b>(pt_2.y,pt_2.x)[0] = 255;
			fitMap.at<Vec3b>(pt_2.y,pt_2.x)[1] = 255;
			fitMap.at<Vec3b>(pt_2.y,pt_2.x)[2] = 0;
		}	

		pt_1.x = rng.uniform(3, fitMap.cols-3);
		pt_1.y = a_1*pt_1.x+b_1+rng_eps.gaussian(7);
		if(i%5 == 0)
			pt_1.y = 0.5*a_2*(pt_1.x-rng_eps.gaussian(15))+b_2-rng_eps.gaussian(35);

		if(pt_1.y<=fitMap.rows-1&& pt_1.y>=0)
		{
			fitData_1.push_back(pt_1);
			fitMap.at<Vec3b>(pt_1.y,pt_1.x)[0] = 0;
			fitMap.at<Vec3b>(pt_1.y,pt_1.x)[1] = 0;
			fitMap.at<Vec3b>(pt_1.y,pt_1.x)[2] = 255;
		}	
	}
}



