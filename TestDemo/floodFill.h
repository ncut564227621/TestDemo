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

	//����ջ��ʵ��
	//����һ��ջ
	//1. �����ǰ������Ϊ���� seedcolor-lower<curcolor<seedColor+higer, ��ǰ��curPt = setColor
	//2. ����ǰ��curPt�İ��������뵽ջ��
	//3. curPt=  stack.pop()
	//4. �ظ�1��2�� ֱ����ջ�����еĵ㲻�������������İ������˳���
	//seedPt ���ӵ�
	//setColor ��Ҫ���õ�Ŀ����ɫ
	//seedColor ���ӵ����ɫ
	// ucLower ��������������ֵ
	// ucHigh  ��������������ֵ
	void floodFill8Stack(Point  seedPt, Scalar setColor,Scalar seedColor,  uchar ucLower, uchar ucHighert, const int w, const int h);

	//�г̱����㷨����ɨ���㷨��
	//seedPt ���ӵ�����
	//setColor ���õ���ɫֵ
	//seedColor ���ӵ����ɫֵ
	//��ɨ��ݹ�ķ���ʵ��

	void  floodFillScanline(Point seedPt, Scalar setColor, Scalar seedColor, const uchar ucLow, const uchar ucHigh, const int w, const int h);

	//����ջ����ɨ���㷨��ʵ��

};

