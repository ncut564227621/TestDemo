// TestDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"opencv2/opencv.hpp"
#include "../TestAlgorithms/TestAlgorithms.h"


int _tmain(int argc, _TCHAR* argv[])
{
	Mat img = imread("..\\Sample\\binggan.bmp",CV_LOAD_IMAGE_GRAYSCALE);

	/*vector<vector<int>>equal_label;

	vector<int> vec_0;
	vec_0.push_back(0);

	vector<int> vec_1;
	vec_1.push_back(1);
	vec_1.push_back(3);
	vec_1.push_back(5);
	vec_1.push_back(7);

	vector<int> vec_2;
	vec_2.push_back(2);
	vec_2.push_back(4);
	vec_2.push_back(6);

	vector<int> vec_3;
	vec_3.push_back(3);
	vec_3.push_back(4);
	vec_3.push_back(6);

	vector<int> vec_4;
	vec_4.push_back(4);

	vector<int> vec_5;
	vec_5.push_back(5);

	vector<int> vec_6;
	vec_6.push_back(6);

	vector<int> vec_7;
	vec_7.push_back(7);

	equal_label.push_back(vec_0);
	equal_label.push_back(vec_1);
	equal_label.push_back(vec_2);
	equal_label.push_back(vec_3);	
	equal_label.push_back(vec_4);
	equal_label.push_back(vec_5);
	equal_label.push_back(vec_6);
	equal_label.push_back(vec_7);


	bool bVisitFlag[8] = {false};
	for(int i =1; i<equal_label.size(); i++)
	{
	if(bVisitFlag[i]) 
	continue;
	int set_label = equal_label[i][0];
	secondPass(equal_label, bVisitFlag, i, set_label);
	}*/
	Mat magnImg;
	DFTtransform(img,   magnImg);

	for(int i =0; i<10; i++)
	{
		Mat inImg = img.clone();
		vector<cnnBlob>blobs;
		int nNeighCount = 1;
		uchar maskArr[9]= {0,1,0,1,0,1,0,1,0};
		Mat labelMask(2*nNeighCount+1, 2*nNeighCount+1, CV_8UC1, maskArr);
		Mat labelImg;
		const int64 nStart = getTickCount();

		//int nConnNums = DetectConnFindContours(inImg, contours, 200, true);
		icvprCcaBySeedFill(inImg, labelImg);

		double nDuration = (getTickCount() - nStart)/getTickFrequency();
		cout<<"cost time: "<<nDuration*1000<<"ms"<<endl;
	}

	return 0;
}



