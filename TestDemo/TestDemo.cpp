// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"
#include<string>
#include "lsd.h"

#include "CircleFitByHyper.h"
#include "floodFill.h"

//extern Mat src;
//extern Mat dst;


void randomlineData(Mat& fitMap, vector<Point>& fitData, vector<Point>& fitData_2, const int ptNums);


int _tmain(int argc, _TCHAR* argv[])
{
	Mat img = imread("..\\Sample\\handsome.jpg",CV_LOAD_IMAGE_COLOR);//image size:720*580 px

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

	Mat floodFillMat(img.size(), CV_8UC3, Scalar::all(0));

	Point seedPt(34,42);
	Point orgPt = seedPt;

	Scalar setScalar;
	

	Scalar curScalar = img.at<Vec3b>(42,34);
	setScalar = curScalar;
	Scalar seedColor(setScalar);

	uchar uLowLimit = 30, uUPLimit =30;

	CFloodFilled ObFloodFill(img, floodFillMat);
	Size roiwin = Size(img.cols,img.rows);

	for(int i = (orgPt.y - roiwin.height/2); i<orgPt.y+roiwin.height/2; i++)
	{
		for(int j = (orgPt.x - roiwin.width/2); j<orgPt.x+roiwin.width/2; j++)
		{
			
			if(i<0||i>img.rows-1)
				continue;
			if(j<0||j>img.cols-1)
				continue;

			bool bFlag= true;
			Vec3b dstVec = ObFloodFill.dst.at<Vec3b>(i, j);

			if(dstVec[0]== setScalar[0]&&dstVec[1] == setScalar[1] && dstVec[2] == setScalar[2])
			{
				//bFlag = false;
				continue;
			}

			bool bFlag_1 = false;
			Vec3b srcVec = ObFloodFill.src.at<Vec3b>(i, j);
			if(srcVec[0] >=(int)((int)seedColor.val[0]-(int)uLowLimit) && srcVec[0] <= (int)((int)seedColor.val[0]+ (int)uUPLimit) 
				&& srcVec[1] >=(int)((int)seedColor.val[1]-(int)uLowLimit) && srcVec[1] <= (int)((int)seedColor.val[1]+ (int)uUPLimit)&&
				srcVec[2] >=(int)((int)seedColor.val[2]-(int)uLowLimit) && srcVec[2] <= (int)((int)seedColor.val[2]+ (int)uUPLimit))
			{
				bFlag_1 = true;
			}

			if(bFlag && bFlag_1)
			{
	            seedPt = Point(j, i);
				ObFloodFill.floodFillScanline(seedPt, setScalar, seedColor, uLowLimit, uUPLimit, img.cols, img.rows);
				//ObFloodFill.floodFill8Stack(seedPt, setScalar, seedColor, uLowLimit, uUPLimit, img.cols, img.rows);
			}
			
		}
	}
	imwrite("..\\sample\\result\\flood8.bmp", ObFloodFill.dst);
	return 0;


	for(int i =0; i<10; i++)

	{

		const int64 nStart = getTickCount();
		//to do something


		double nDuration = (getTickCount() - nStart)/getTickFrequency();

		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;


	//	imwrite("..\\Sample\\result\\solve_fit_circle.bmp", plotMat);

		//imwrite("..\\Sample\\result\\ransac_circle.bmp", img);
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



