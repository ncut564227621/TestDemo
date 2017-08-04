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

//Two-Pass（两遍扫描算法）
/*/////////////////////////////////////////////////////////////////////////
//第一遍扫描：假设前景像素灰度值=255， 背景像素灰度值=0，图像B,像素位置:x,y
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
TESTALGORITHMS_API int DetectConnTowPass(Mat grayImg, vector<cnnBlob>&blobs, const uchar thd, const Mat labelMask, const bool bInverse);

bool upDateEqualLabel(vector<vector<int>>&equalLabel, vector<int>neighborLabel, const int minLabel);

bool upDateBlobs(vector<cnnBlob>&blobs, const int i, const int j, const int label_index);

TESTALGORITHMS_API bool secondPass(vector<vector<int>>&equalLabel, bool* bVisitFlag, const int equal_index,  const int set_label);

//Seed Fill算法
/*/////////////////////////////////////////////////////////////////////////
（1）扫描图像，直到当前像素点B(x,y) == 255：
   a、将B(x,y)作为种子（像素位置），并赋予其一个label，然后将该种子相邻的所有前景像素都压入栈中；
   b、弹出栈顶像素，赋予其相同的label，然后再将与该栈顶像素相邻的所有前景像素都压入栈中；
   c、重复b步骤，直到栈为空；
     此时，便找到了图像B中的一个连通区域，该区域内的像素值被标记为label；
（2）重复第（1）步，直到扫描结束；
     扫描结束后，就可以得到图像B中所有的连通区域；
/////////////////////////////////////////////////////////////////////////*/
void TESTALGORITHMS_API icvprCcaBySeedFill(const Mat& _binImg, Mat& _lableImg);

/*/////////////////////////////////////////////////////////////////////////
DFT变换   OPENCV函数实现
/////////////////////////////////////////////////////////////////////////*/
void TESTALGORITHMS_API DFTtransform(const Mat _srcImg, Mat& _magnImg);


void TESTALGORITHMS_API DFTtransform_Ex(const Mat _srcImg, Mat &_magnImg);

void TESTALGORITHMS_API convolveDFT(Mat _inputImg, Mat _kernel, Mat& _outImg);
void TESTALGORITHMS_API convolveTIME(Mat _inputImg, Mat _kernel, Mat& _outImg);