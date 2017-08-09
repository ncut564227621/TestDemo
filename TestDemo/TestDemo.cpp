// TestDemo.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"
#include<string>
#include "lsd.h"

void randomlineData(Mat& fitMap, vector<Point>& fitData, vector<Point>& fitData_2, const int ptNums);


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
	
	Mat fitMap(img.size(), CV_8UC3, Scalar::all(50));
	int ptsNum = 1000;
	vector<Point>fitData_1;
	vector<Point>fitData_2;

	randomlineData(fitMap,fitData_1,fitData_2, ptsNum);
	
	imwrite("..\\sample\\result\\fitMap_raw_data.bmp",fitMap);

	//�����fitData_1
	Vec4f lineParam;
	lineFit(fitMap, fitData_1, lineParam, RANSAC_Fit);

	Point pt1, pt2;
	pt1.x = lineParam[2] - 300*lineParam[0];
	pt1.y = lineParam[3]-  300*lineParam[1];

	//
	pt2.x = pt1.x + 500*lineParam[0];
	pt2.y = pt1.y + 500*lineParam[1];

	line(fitMap, pt1, pt2, Scalar(0,255,0),2);
	

	lineFit(fitMap, fitData_2, lineParam, RANSAC_Fit);
	//���fitData_2
	//fitLine(fitData_2, lineParam, CV_DIST_L2,0,0,0);

	pt1.x = lineParam[2] - 200*lineParam[0];
	pt1.y = lineParam[3]-  200*lineParam[1];

	//
	pt2.x = pt1.x + 500*lineParam[0];
	pt2.y = pt1.y + 500*lineParam[1];

	line(fitMap, pt1, pt2, Scalar(255,255,255),2);


	imwrite("..\\sample\\result\\fitMap.bmp",fitMap);
	

	
	int a =0;
	for(int i =0; i<10; i++)
	{
		
		const int64 nStart = getTickCount();

		//to do something;
	

		double nDuration = (getTickCount() - nStart)/getTickFrequency();
		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;
		int a =0;
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



