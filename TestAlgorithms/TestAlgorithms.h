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

enum LineDetectType
{
	Hogh_Stand,
	Hogh_MultScale,
	Hough_Pro,
	LSD,
};

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



//ImageProcesssing

//optmize medianFilter
//基本思路：利用直方图确定中值
/*
算法流程：
   1. 设定门限：th = (N*M)/2+1;
   2. 将窗口移动到一个新航的开始，建立窗口像素直方图，通过直方图确定中值med,记下亮度小于或者等于med的像素的数目到mNum中
   3.对于最左侧列的每个像素，其灰度值设为glc, 去掉左侧像素：

                                              H[glc] = H[glc] - 1；
											  if(glc <= med), mNum = mNum -1;
   4. 将窗口右移一列，对于最右侧的像素，设其灰度值为grc,执行添加右侧像素：
											   H[grc] = H[grc] + 1；
											   if(grc <= med), mNum = mNum +1;
   5. 若mNum>th;则转到6；否则（mNum<=th）:
											   while(mNum<th)

											   {med=med+1;mNum=mNum+H[med];}

											   转 7
6.重复

                                              mNum=mNum−H[med]
                                              med=med−1
直到

                                              mNum≤th
7. 如果窗口的右侧不是图像的右边界，则转 3
8. 若果窗口的底行不是图象的下边界，则转 2
                                          
*/

void TESTALGORITHMS_API optimizeMedianBlur(Mat _inImg, Mat& _outImg, const int kernel_w, const int kernel_h);

//otsu算法的思想，实际上是求使类间方差最大化的时候的二值化阈值
void TESTALGORITHMS_API otsuThreshold(Mat _inImg, Mat & _outImg, uchar& ucThd);

//wellner二值化算法：
/*
//基本思想是:
    1. 求局部像素的滑动平均值 m
	2. 如果当前像素的值，明显低于滑动平均值，则，该像素置为0，否则255

*/
void TESTALGORITHMS_API wellnerThreshold(Mat _inImg, Mat& _outImg, const int nRadius, const int nRatio);

void TESTALGORITHMS_API wellnerThresholdEx(unsigned char* input, unsigned char*& bin, int width, int height);

//利用OPENCV求sobel近似梯度图像
void TESTALGORITHMS_API sobelGradient(Mat _inImg, Mat& grad);


//利用Gabor滤波器提取图像的方向梯度算法
/*
构造Gabor Filter
nGaborW:滤波器模板的宽
nGaborH:滤波器模板的高
dFre:   要求筛选的中心频率
dSigma: 滤波器的宽度，值越大，滤波器越宽，值越小，滤波器带宽越小
dGamma: 滤波器x和y两个空间方向的比率
nPsi:   设置的相位偏差
*/

TESTALGORITHMS_API bool Common_Creat_GaborFilter(const int nGaborW, const int nGaborH, const float dFre, const double dSigma, const double dTheta, const double dGamma,const int nPsi, Mat &RealGaborFilter,Mat& ImaginaryGaborFilter);


//Hough_Line_Transformation
/*

*/

TESTALGORITHMS_API void lineDetect(Mat _inImg, Mat& _outImg, vector<Vec2f>vecLineParams, const int _lineDetectType);