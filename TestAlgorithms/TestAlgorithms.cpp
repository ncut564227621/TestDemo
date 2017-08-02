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
	else
		threshold(grayImg,grayImg, thd,255, CV_THRESH_BINARY);

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
		int connArea = contourArea(*iter, false);
		cout<<"rotate angle: "<<minRect.angle<<"  contourArea size:"<<connArea<<endl;
	}

	return 0;
}

TESTALGORITHMS_API int DetectConnTowPass(Mat grayImg, vector<vector<Point>>&contours, const uchar thd, const Mat labelMask, const bool bInverse)
{
	if (grayImg.empty() || grayImg.channels() > 1)
	{
		return -1;
	}
	if (bInverse)
	{
		threshold(grayImg, grayImg, thd, 255, CV_THRESH_BINARY_INV);
	}
	else
		threshold(grayImg,grayImg, thd,255, CV_THRESH_BINARY);

	//fist pass

	vector<vector<int>>equalLabel; //等价连通域
	vector<int>zeroVec;
	zeroVec.push_back(0);
	equalLabel.push_back(zeroVec);

	int label = 1;
	Mat labelMat(grayImg.rows,grayImg.cols,CV_32SC1, Scalar(0));
	int nRows = grayImg.rows;
	int nCols = grayImg.cols;

	int NeighborCount = (int)((labelMask.rows-1)/2);//假定labelMask.rows = labelMask.cols

	uchar* ptr = NULL;
	int* labelPtr = NULL;
	for(int i =0; i<nRows; i++)
	{
		if(i<0||i>=nRows-1)
			continue;
		ptr = grayImg.ptr<uchar>(i);//第i行地址指针 BINiMAGE
		labelPtr = labelMat.ptr<int>(i); //第i行地址指针 labelMat

		for(int j =0; j<nCols; j++)
		{
			uchar ucPixl= ptr[j];


		   bool bNeighborNotAllZero = false;
		   int uiMinLabelValue = 0xffffff;

			if(ucPixl!=0)
			{
	            vector<int>neighLabelVec;
				for(int k = i-NeighborCount; k<=i+NeighborCount; k++)
				{
					if(k<0||k>labelMat.rows-1)
						continue;
					int* ptr = labelMat.ptr<int>(k);
					for(int l = j- NeighborCount; l<=j+NeighborCount; l++)
					{
			            if(l<0||l>labelMat.cols-1)
							continue;
						uchar maskVal = labelMask.at<uchar>(k-i+NeighborCount,l-j+NeighborCount);
						int labelVal = ptr[l];
						if(maskVal == 0)
							continue;
						if(labelVal!=0) //labelVal初始化成0
						{
				            bNeighborNotAllZero = true;
							if(labelVal<uiMinLabelValue)
							     uiMinLabelValue = labelVal;
							neighLabelVec.push_back(labelVal);
						}

					}
		
				}
				if(bNeighborNotAllZero)
				{
		            labelPtr[j] = uiMinLabelValue;
					upDateEqualLabel(equalLabel, neighLabelVec, uiMinLabelValue);
				}
				else
				{
					vector<int> vecLabel;
					labelPtr[j] = label;
					vecLabel.push_back(label);
					equalLabel.push_back(vecLabel);
					label+=1;
				}
			}
		}
	}

	//second pass;
	bool *bVisitFlag = new bool[equalLabel.size()];
	memset(bVisitFlag, false, equalLabel.size()*sizeof(bool));
	vector<vector<int>>::iterator iter = equalLabel.begin();
	vector<vector<int>>::iterator iterEnd = equalLabel.end();
	
	for(;iter!=iterEnd; iter++)
	{
		vector<int> curLabelVec = *iter;
		int first_label = curLabelVec[0];
		if(bVisitFlag[first_label])
		{
			continue;
		}
		else
		{
			for(int i =1; i<curLabelVec.size(); i++)
			{
	            
			}
		}
	}
	 

	return 0;
}

bool upDateEqualLabel(vector<vector<int>>&equalLabel, vector<int>neighborLabel, const int minLabel)
{
	
	vector<int>::iterator ngb_iter = neighborLabel.begin();
	for(;ngb_iter!=neighborLabel.end(); ngb_iter++)
	{
		int ngb_label = *ngb_iter;
		int i =0;
		for(i =0;i<equalLabel[minLabel].size(); i++)
		{
			if(ngb_label== equalLabel[minLabel][i])
				break;
			
		}
		if(i>=equalLabel[minLabel].size())
			equalLabel[minLabel].push_back(ngb_label);
	}
	return true;
}
