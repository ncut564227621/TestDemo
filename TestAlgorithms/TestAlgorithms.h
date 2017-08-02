#pragma  once
#include"opencv2/opencv.hpp"
#include<iostream>

#ifdef TESTALGORITHMS_EXPORTS
#define TESTALGORITHMS_API __declspec(dllexport)
#else
#define TESTALGORITHMS_API __declspec(dllimport)
#endif

using namespace cv;
using namespace std;

TESTALGORITHMS_API int DetectConnFindContours(Mat grayImg, vector<vector<Point>>&contours, const uchar threshold, const bool bInverse);
TESTALGORITHMS_API int DrawFindContours(Mat Img, vector<vector<Point>> contours);

//Two-Pass（两遍扫描算法）
/*/////////////////////////////////////////////////////////////////////////
//第一遍扫描：假设前景像素灰度值=1， 背景像素灰度值=0，图像B,像素位置:x,y
  （1）访问当前像素B(x,y),如果B(x,y)==1
	  a.如果B(x,y)邻域内的像素都为0:（可以选择4领域或者8邻域）
	     B(x,y) = label; label+=1
	  b.如果B(x,y)邻域内的像素存在一个值>=1;将邻域内最小的label值赋给B(x,y)
         B(x,y) = min(Neighbor)
	  c.记录邻域内各个（label）之间的关系，这些label同属一个连通域
   (2)第二次扫描
	   访问当前像素B(x,y)，如果B(x,y) > 1：
	        找到与label = B(x,y)同属相等关系的一个最小label值，赋予给B(x,y)；
	   完成扫描后，图像中具有相同label值的像素就组成了同一个连通区域
/////////////////////////////////////////////////////////////////////////*/
TESTALGORITHMS_API int DetectConnTowPass(Mat grayImg, vector<vector<Point>>&contours, const uchar thd, const Mat labelMask, const bool bInverse);

bool upDateEqualLabel(vector<vector<int>>&equalLabel, vector<int>neighborLabel, const int minLabel);