// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"
#include<string>



int _tmain(int argc, _TCHAR* argv[])
{
	Mat img = imread("..\\Sample\\lena.bmp",CV_LOAD_IMAGE_COLOR);//image size:720*580 px

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

	Mat filterMat = Mat(img.size(), CV_32FC1, Scalar(0));

	//gabor 滤波器

	double orientArray[4] = {0, 45,90, 135};
	//设置gabor 滤波器参数；
	int nGaborW = 100, nGaborH =100;
	float dFre = 0.05; //一个周期20个像素
	double dSigma = 10,dTheta = 45, dGamma =1.0;
    const int nPsi = 1;
	Mat RealGaborFilter,ImaginaryGaborFilter;
	string csFilePath = "..\\sample\\result\\Gabor\\";
	for(int i = 0; i<4; i++)
	{
		stringstream strStream;
		string csRealGabor;
		string csImGabor;
		string csGaborName ="GaborFilter_";
		dTheta = orientArray[i];

		strStream<<dTheta<<"_";
		csGaborName+=strStream.str();
		Common_Creat_GaborFilter(nGaborW, nGaborH, dFre, dSigma, dTheta, dGamma, nPsi, RealGaborFilter, ImaginaryGaborFilter);
		csRealGabor+=csGaborName;
		csRealGabor+="Real.bmp";
		csImGabor += csGaborName;
		csImGabor+="Im.bmp";

		filter2D(_inImg, filterMat, CV_32F, RealGaborFilter);

		//normalize(RealGaborFilter, RealGaborFilter, 0,255, CV_MINMAX);
		normalize(filterMat, filterMat,0,255, CV_MINMAX);

		//imwrite(csFilePath+csRealGabor, RealGaborFilter);
		//imwrite(csFilePath+csImGabor, ImaginaryGaborFilter);
		imwrite("..\\sample\\result\\Gabor_Filter.bmp", filterMat);

		int a =0;
		
	}

	return 0;

	for(int i =0; i<10; i++)
	{

		const int64 nStart = getTickCount();

		//to do something;
	
		double nDuration = (getTickCount() - nStart)/getTickFrequency();
		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;

		imwrite("..\\sample\\result\\sobel_3x3.bmp", _outImg);
		int a =0;
	}

	

	return 0;
}



