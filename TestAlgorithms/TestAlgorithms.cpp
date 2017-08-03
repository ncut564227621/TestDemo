// TestAlgorithms.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#include "TestAlgorithms.h"
#include<list>
#include<stack>
#include<map>

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

TESTALGORITHMS_API int DetectConnTowPass(Mat grayImg, vector<cnnBlob>&blobs, const uchar thd, const Mat labelMask, const bool bInverse)
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
	for(int i =1; i<equalLabel.size();  i++)
	{
		if(bVisitFlag[i]) 
			continue;
		int set_label = equalLabel[i][0];
		secondPass(equalLabel, bVisitFlag, i, set_label);
	}

	//标注所有的联通域
	//Mat colorLabelMat(labelMat.size(), CV_8UC3, Scalar::all(0));

	for(int i =0; i<nRows; i++)
	{
		int* ptr = labelMat.ptr<int>(i);
		//Vec3b* clrPtr = colorLabelMat.ptr<Vec3b>(i);
		for(int j =0; j<nCols; j++)
		{
			int label = ptr[j];
			ptr[j] = equalLabel[label][0];
			label = equalLabel[label][0];
			upDateBlobs(blobs, j,i,label);
			/*clrPtr[j][0] = label<<1&0xff;
			clrPtr[j][1] = label<<2&0xff;
			clrPtr[j][2] = label<<7;*/

		}
	}
	//imwrite("..\\sample\\result\\equal.bmp", colorLabelMat);
	/*cout<<"blob counts: "<<blobs.size()<<endl;
	vector<cnnBlob>::iterator iter = blobs.begin();
	int i =0;
	for(; iter!=blobs.end(); iter++, i++)
	{
	if(iter->blob_pts.size()<1000)
	continue;
	cout<<"blob_index_"<<i<<" size:"<<iter->blob_pts.size()<<endl;
	}
	*/
	if(bVisitFlag)
		delete [] bVisitFlag;
	bVisitFlag = NULL;
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
bool upDateBlobs(vector<cnnBlob>&blobs, const int i, const int j, const int label_index)
{
	if(label_index == 0)
		return true;
	if(blobs.size() == 0)
	{
		cnnBlob oneBlob;
		oneBlob.label = label_index;
		oneBlob.blob_pts.push_back(cvPoint(i,j));
		blobs.push_back(oneBlob);
		return true;
	}
	vector<cnnBlob>::iterator iter = blobs.begin();
	for(; iter!=blobs.end(); iter++)
	{
		if(label_index == iter->label)
		{
			iter->blob_pts.push_back(cvPoint(i,j));
			return true;
		}
	}
	if(iter == blobs.end())
	{
		cnnBlob oneBlob;
		oneBlob.label = label_index;
		oneBlob.blob_pts.push_back(cvPoint(i,j));
		blobs.push_back(oneBlob);
	}
	return true;
}

//递归算法标注equal连通域
TESTALGORITHMS_API bool secondPass(vector<vector<int>>&equalLabel, bool* bVisitFlag, const int equal_index, const int set_label)
{
	//递归遍历equalLabel
	
	if(equalLabel.size()<=0|| equal_index<0 || set_label<0)
	{
		return false;
	}
	if(bVisitFlag[equal_index])
	{
		if(set_label>equalLabel[equal_index][0])                               //如果set_label的值大于equal_index对应的0值         
		{
            equalLabel[set_label][0] = equalLabel[equal_index][0];             //把equalIndex的值赋给set_label
		}
		return true;
	}
	equalLabel[equal_index][0] = set_label;
	bVisitFlag[equal_index] = true;
	for(int i =0; i<equalLabel[equal_index].size(); i++)
	{
		secondPass(equalLabel, bVisitFlag, equalLabel[equal_index][i], set_label);
	}
	return true;
}

 TESTALGORITHMS_API void icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg)  
{  
    // connected component analysis (4-component)  
    // use seed filling algorithm  
    // 1. begin with a foreground pixel and push its foreground neighbors into a stack;  
    // 2. pop the top pixel on the stack and label it with the same label until the stack is empty  
    //   
    // foreground pixel: _binImg(x,y) = 1  
    // background pixel: _binImg(x,y) = 0  
  
  
    if (_binImg.empty() ||  
        _binImg.type() != CV_8UC1)  
    {  
        return ;  
    }  
     threshold(_binImg, _binImg, 200, 255, CV_THRESH_BINARY_INV);

    _lableImg.release() ;  
    _binImg.convertTo(_lableImg, CV_32SC1) ;  
  
    int label = 1 ;  // start by 2  
  
    int rows = _binImg.rows - 1 ;  
    int cols = _binImg.cols - 1 ;  
    for (int i = 1; i < rows-1; i++)  
    {  
        int* data= _lableImg.ptr<int>(i) ;  
        for (int j = 1; j < cols-1; j++)  
        {  
            if (data[j] == 255)  
            {  
                std::stack<std::pair<int,int>> neighborPixels ;     
                neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>  
                ++label ;  // begin with a new label  
                while (!neighborPixels.empty())  
                {  
                    // get the top pixel on the stack and label it with the same label  
                    std::pair<int,int> curPixel = neighborPixels.top() ;  
                    int curX = curPixel.first ;  
                    int curY = curPixel.second ;  
                    _lableImg.at<int>(curX, curY) = label ;  
  
                    // pop the top pixel  
                    neighborPixels.pop() ;  
  
                    // push the 4-neighbors (foreground pixels)  
                    if (_lableImg.at<int>(curX, curY-1) == 255)  
                    {// left pixel  
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;  
                    }  
                    if (_lableImg.at<int>(curX, curY+1) == 255)  
                    {// right pixel  
                        neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;  
                    }  
                    if (_lableImg.at<int>(curX-1, curY) == 255)  
                    {// up pixel  
                        neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;  
                    }  
                    if (_lableImg.at<int>(curX+1, curY) == 255)  
                    {// down pixel  
                        neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;  
                    }  
                }         
            }  
        }  
    }  
}

 void TESTALGORITHMS_API DFTtransform(const Mat _srcImg, Mat& _magnImg)
 {
	 if(_srcImg.rows<=0||_srcImg.cols<=0||_srcImg.depth()!=CV_8UC1)
	 {
		 cout<<"输入图像正确！"<<endl;
	 }

	 Mat padded;
	 int m = getOptimalDFTSize(_srcImg.rows);
	 int n = getOptimalDFTSize(_srcImg.cols);
	 copyMakeBorder(_srcImg, padded, 0, m-_srcImg.rows, 0, n-_srcImg.cols, BORDER_CONSTANT, Scalar(0));

	 //
	 Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32FC1)};
	 Mat complexI;
	 merge(planes,2, complexI);

	 dft(complexI, complexI);  

	// split(complexI, planes);

	// magnitude(planes[0], planes[1], planes[0]);
	// _magnImg = planes[0];

	// _magnImg+=Scalar::all(1);
	// log(_magnImg,_magnImg);

	// //在上面的步骤中我们进行了边界填充，所以后面的步骤中，我们需要进行CROP
	// //同时为了可视化，我们需要进行rearrange的操作，让图像的起点回到图像的中心位置显示

	// _magnImg = _magnImg(cvRect(0,0,_magnImg.cols & -2, _magnImg.rows & -2)); //-2= (0xfffffffe) //crop image



	// //将频谱图像进行中心偏移        （-1）^(x+y)[坐标偏移公式]
	// int cx = _magnImg.cols >>1;
	// int cy = _magnImg.rows >>1;

	// Mat q0(_magnImg, Rect(0,0,cx,cy));
	// Mat q1(_magnImg, Rect(cx,0,cx,cy));
	// Mat q2(_magnImg, Rect(0,cy,cx,cy));
	// Mat q3(_magnImg, Rect(cx,cy,cx,cy));

	// Mat temp;
	// q0.copyTo(temp);
	// q3.copyTo(q0);
	// temp.copyTo(q3);

	// q1.copyTo(temp);
	// q2.copyTo(q1);
	// temp.copyTo(q2);

	// normalize(_magnImg, _magnImg, 0, 255, CV_MINMAX);

	//imwrite("..\\sample\\result\\dct_magniute.bmp", _magnImg);

 }

 //调用IPP进行FFT运算
 void TESTALGORITHMS_API DFTtransform_Ex(const Mat _srcImg, Mat &_magnImg)
 {
	
	 Mat padded;
	 //int nMaxLength = _srcImg.cols>_srcImg.rows?_srcImg.cols:_srcImg.rows;
	 int m = 1;
	 int n = 1;
	 int nOrderY = 0;
	 int nOrderX =0;
	 int nCols = _srcImg.cols;
	 int nRows = _srcImg.rows;
	 while (nCols!=0)
	 {
	     nCols >>= 1;
		 n <<= 1;
		 nOrderY++;
	 }

	 while (nRows!=0)
	 {
		 nRows >>= 1;
		 m <<= 1;
		 nOrderX++;
	 }


	//m = getOptimalDFTSize(_srcImg.rows);
	//n = getOptimalDFTSize(_srcImg.cols);
	
	 copyMakeBorder(_srcImg, padded, 0, m-_srcImg.rows, 0, n-_srcImg.cols, BORDER_CONSTANT, Scalar(0));

	 Mat timeMat = Mat_<float>(padded);
	 Mat freMat = Mat(padded.size(), CV_32FC1, Scalar(0));


	// IplImage* pAlignDown=pAppearanceData->m_pAppearanceImage[SurfaceAppearanceImage_Align_DownSample];
	
	 IppiFFTSpec_R_32f*spec;
	 ippiFFTInitAlloc_R_32f( &spec, nOrderX, nOrderY, IPP_FFT_DIV_INV_BY_N, ippAlgHintAccurate);

	 ippiFFTFwd_RToPack_32f_C1R((Ipp32f*)(timeMat.data), n*sizeof(Ipp32f),(Ipp32f*)(freMat.data),n*sizeof(Ipp32f),spec,0);
	 int a =0;
 }

 //从时间效率上来看OPENCV做FFT的时间比IPP更短，因为OPENCV模块会自动加载IPP模块

