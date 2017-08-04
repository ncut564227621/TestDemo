#pragma  once
#include"opencv2/opencv.hpp"
#include<iostream>
#include"ippi.h"

#ifdef TESTALGORITHMS_EXPORTS
#define TESTALGORITHMS_API __declspec(dllexport)
#else
#define TESTALGORITHMS_API __declspec(dllimport)
#endif

using namespace cv;
using namespace std;

typedef struct tagcnnBlob
{
	int label;
	vector<Point>blob_pts;
}cnnBlob;

TESTALGORITHMS_API int DetectConnFindContours(Mat grayImg, vector<vector<Point>>&contours, const uchar threshold, const bool bInverse);
TESTALGORITHMS_API int DrawFindContours(Mat Img, vector<vector<Point>> contours);

//Two-Pass������ɨ���㷨��
/*/////////////////////////////////////////////////////////////////////////
//��һ��ɨ�裺����ǰ�����ػҶ�ֵ=255�� �������ػҶ�ֵ=0��ͼ��B,����λ��:x,y
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
TESTALGORITHMS_API int DetectConnTowPass(Mat grayImg, vector<cnnBlob>&blobs, const uchar thd, const Mat labelMask, const bool bInverse);

bool upDateEqualLabel(vector<vector<int>>&equalLabel, vector<int>neighborLabel, const int minLabel);

bool upDateBlobs(vector<cnnBlob>&blobs, const int i, const int j, const int label_index);

TESTALGORITHMS_API bool secondPass(vector<vector<int>>&equalLabel, bool* bVisitFlag, const int equal_index,  const int set_label);

//Seed Fill�㷨
/*/////////////////////////////////////////////////////////////////////////
��1��ɨ��ͼ��ֱ����ǰ���ص�B(x,y) == 255��
   a����B(x,y)��Ϊ���ӣ�����λ�ã�����������һ��label��Ȼ�󽫸��������ڵ�����ǰ�����ض�ѹ��ջ�У�
   b������ջ�����أ���������ͬ��label��Ȼ���ٽ����ջ���������ڵ�����ǰ�����ض�ѹ��ջ�У�
   c���ظ�b���裬ֱ��ջΪ�գ�
     ��ʱ�����ҵ���ͼ��B�е�һ����ͨ���򣬸������ڵ�����ֵ�����Ϊlabel��
��2���ظ��ڣ�1������ֱ��ɨ�������
     ɨ������󣬾Ϳ��Եõ�ͼ��B�����е���ͨ����
/////////////////////////////////////////////////////////////////////////*/
void TESTALGORITHMS_API icvprCcaBySeedFill(const Mat& _binImg, Mat& _lableImg);

/*/////////////////////////////////////////////////////////////////////////
DFT�任   OPENCV����ʵ��
/////////////////////////////////////////////////////////////////////////*/
void TESTALGORITHMS_API DFTtransform(const Mat _srcImg, Mat& _magnImg);


void TESTALGORITHMS_API DFTtransform_Ex(const Mat _srcImg, Mat &_magnImg);

void TESTALGORITHMS_API convolveDFT(Mat _inputImg, Mat _kernel, Mat& _outImg);
void TESTALGORITHMS_API convolveTIME(Mat _inputImg, Mat _kernel, Mat& _outImg);