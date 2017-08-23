#pragma  once
#include"opencv2/opencv.hpp"
#include<iostream>
#include"stack"
#include<vector>

using namespace cv;
using namespace std;



//flood fill algorithms
class CFloodFilled
{
public:
	CFloodFilled(Mat& _src, Mat& _dst);
	Mat src;
	Mat dst;
	stack<Point, vector<Point>> m_floodFillStack;
	void floodFill8(Point seedPt, Scalar newColor, Scalar oldColor,  uchar ucLower, uchar ucHigher, const int w, const int h);

	//基于栈的实现
	//定义一个栈
	//1. 如果当前点满足为背景 seedcolor-lower<curcolor<seedColor+higer, 当前点curPt = setColor
	//2. 将当前点curPt的八邻域点加入到栈中
	//3. curPt=  stack.pop()
	//4. 重复1、2， 直到堆栈中所有的点不再有满足条件的八邻域，退出。
	//seedPt 种子点
	//setColor 需要设置的目标颜色
	//seedColor 种子点的颜色
	// ucLower 满足条件的下限值
	// ucHigh  满足条件的上限值
	void floodFill8Stack(Point  seedPt, Scalar setColor,Scalar seedColor,  uchar ucLower, uchar ucHighert, const int w, const int h);

	//行程编码算法（线扫描算法）
	//seedPt 种子点坐标
	//setColor 设置的颜色值
	//seedColor 种子点的颜色值
	//线扫描递归的方法实现

	void  floodFillScanline(Point seedPt, Scalar setColor, Scalar seedColor, const uchar ucLow, const uchar ucHigh, const int w, const int h);

	//基于栈的线扫描算法的实现

};

