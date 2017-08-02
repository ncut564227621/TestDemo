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

//Two-Pass������ɨ���㷨��
/*/////////////////////////////////////////////////////////////////////////
//��һ��ɨ�裺����ǰ�����ػҶ�ֵ=1�� �������ػҶ�ֵ=0��ͼ��B,����λ��:x,y
  ��1�����ʵ�ǰ����B(x,y),���B(x,y)==1
	  a.���B(x,y)�����ڵ����ض�Ϊ0:������ѡ��4�������8����
	     B(x,y) = label; label+=1
	  b.���B(x,y)�����ڵ����ش���һ��ֵ>=1;����������С��labelֵ����B(x,y)
         B(x,y) = min(Neighbor)
	  c.��¼�����ڸ�����label��֮��Ĺ�ϵ����Щlabelͬ��һ����ͨ��
   (2)�ڶ���ɨ��
	   ���ʵ�ǰ����B(x,y)�����B(x,y) > 1��
	        �ҵ���label = B(x,y)ͬ����ȹ�ϵ��һ����Сlabelֵ�������B(x,y)��
	   ���ɨ���ͼ���о�����ͬlabelֵ�����ؾ������ͬһ����ͨ����
/////////////////////////////////////////////////////////////////////////*/
TESTALGORITHMS_API int DetectConnTowPass(Mat grayImg, vector<vector<Point>>&contours, const uchar thd, const Mat labelMask, const bool bInverse);

bool upDateEqualLabel(vector<vector<int>>&equalLabel, vector<int>neighborLabel, const int minLabel);