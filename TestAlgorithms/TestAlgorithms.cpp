// TestAlgorithms.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "TestAlgorithms.h"


TESTALGORITHMS_API int DetectConnFindContours(Mat grayImg, vector<vector<Point>>&contours, const uchar thd, const bool bInverse)
{
	if (grayImg.empty() || grayImg.channels() > 1)
	{
		return -1;
	}
	if (bInverse)
	{
		threshold(grayImg, grayImg, thd, 255, CV_THRESH_BINARY_INV);
	}
	findContours(grayImg, contours,
		CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	return contours.size();
}

TESTALGORITHMS_API int DrawFindContours(Mat Img, vector<vector<Point>> contours)
{
	if(Img.empty()||contours.size()==0)
		return -1;
	Mat result;
	if(Img.channels()==1)
		cvtColor(Img, result,CV_GRAY2BGR);
	else
		result=Img;
	drawContours(result,contours,-1,Scalar(255,255,0),1,8);
	imwrite("..\\Sample\\result\\contours.bmp",result);

	vector<vector<Point>>::iterator iter = contours.begin();
	for(;iter!=contours.end(); iter++)
	{
		Rect out_rect = boundingRect(*iter);
		rectangle(result, out_rect, Scalar(0), 1);
	}
	imwrite("..\\Sample\\result\\boudingRect.bmp",result);

	CvBox2D minRect;
	for(iter = contours.begin();iter!=contours.end(); iter++)
	{
		if(iter->size()<50)
			continue;
		minRect = minAreaRect(*iter);
		cout<<"rotate angle:"<<minRect.angle<<endl;
	}

	return 0;
}
