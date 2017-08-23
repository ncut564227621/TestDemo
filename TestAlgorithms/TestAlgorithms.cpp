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


 void TESTALGORITHMS_API convolveDFT(Mat _inputImg, Mat _kernel, Mat& _outImg)
 {
	 if(_inputImg.empty()||_kernel.empty())
	 {
	       cout<<"some error in input image and kernel image"<<endl;
	 }
	 _outImg.create(abs(_inputImg.rows - _kernel.rows)+1, abs(_inputImg.cols-_kernel.cols)+1, _inputImg.type());

	 Size dftSize;
	 dftSize.width = getOptimalDFTSize(_inputImg.cols+_kernel.cols -1);
	 dftSize.height = getOptimalDFTSize(_inputImg.rows+_kernel.rows-1);

	 Mat tempA(dftSize, _inputImg.type(), Scalar::all(0));
	 Mat tempB(dftSize, _kernel.type(), Scalar::all(0));

	 Mat roiA(tempA, Rect(0,0,_inputImg.cols, _inputImg.rows));
	 _inputImg.copyTo(roiA);

	 Mat roiB(tempB, Rect(0,0,_kernel.cols, _kernel.rows));
	 _kernel.copyTo(roiB);

	 dft(tempA, tempA, 0,  _inputImg.rows);
	 dft(tempB, tempB, 0, _kernel.rows);

	 mulSpectrums(tempA, tempB, tempA, 0);

	 dft(tempA, tempA, DFT_INVERSE+DFT_SCALE, _outImg.rows);
	

	 tempA(Rect(0,0,_outImg.cols,_outImg.rows)).copyTo(_outImg);
 }

 void TESTALGORITHMS_API convolveTIME(Mat _inputImg, Mat _kernel, Mat& _outImg)
 {
	 if(_inputImg.empty()||_kernel.empty())
	 {
		 cout<<"some error in input image and kernel image"<<endl;
	 }
	 _outImg.create(_inputImg.size(), CV_32FC1);
	 
	 int inRows = _inputImg.rows;
	 int inCols = _inputImg.cols;

	 int keRows = _kernel.rows;
	 int keCols = _kernel.cols;

	 int outRows = _outImg.rows;
	 int outCols = _outImg.cols;


	 for( int i =(keRows+1)/2; i<inRows-((keRows+1)/2); i++)
	 {
		
		 float* outPtr = _outImg.ptr<float>(i);
		 for(int j =(keCols+1)/2; j<inCols-((keCols+1)/2); j++)
		 {
			float convSum =0.0f;
			 for(int kr = 0; kr<keRows; kr++)
			 {
				 uchar* inPtr = _inputImg.ptr<uchar>(i-(keRows+1)/2+kr);
				 float* kePtr = _kernel.ptr<float>(kr);
				 for(int kc =0; kc<keCols; kc++)
				 {
					 convSum+=kePtr[kc]*inPtr[j-(keCols+1)/2+kc];
				 }
			 }
			 outPtr[j] = convSum;
		 }
	 }
 }


 void optimizeMedianBlur(Mat _inImg, Mat& _outImg, const int kernel_w, const int kernel_h)
 {
	   Mat padded;
	  copyMakeBorder(_inImg, padded, (kernel_h-1)/2, (kernel_h-1)/2, (kernel_w-1)/2, (kernel_w-1)/2, BORDER_REPLICATE);

	  int inRows = padded.rows;
	  int inCols = padded.cols;

	  int keRows = kernel_h;
	  int keCols = kernel_w;

	  int outRows = _outImg.rows;
	  int outCols = _outImg.cols;

	  uchar ucMed = 0;

	  int nthd = (keCols * keRows)/2+1;
	  int medNum = 0;


	  for(int i = (keRows-1)/2; i<inRows - (keRows-1)/2; i++)//行
	  {
		 if(i -(keRows-1)/2<0||i -(keRows-1)/2>outRows-1)
			 continue;
		  uchar* outPtr = _outImg.ptr<uchar>(i -(keRows-1)/2);
		  int hist[256] = {0};
		  bool bBeignNewRow = true;
		  medNum =0;
		  ucMed =0;
		 
		 // cout<<"************beign new row start**************"<<endl;
		  for(int j =(keCols-1)/2; j<inCols-((keCols-1)/2); j++)//列
		  {
			  
			  
			  if(bBeignNewRow) //每次重新开始一行的时候重新计算hist
			  {
				  
				  for(int kr = 0; kr<keRows; kr++)
				  {
					  uchar* inPtr = padded.ptr<uchar>(i-(keRows-1)/2+kr);
					  for(int kc =0; kc<keCols; kc++)
					  {
						  hist[inPtr[j-(keCols-1)/2+kc]]++;
					  }
				  }
				  ucMed+=hist[ucMed]; //此时ucMed = 0; 重新开始新的行数的时候
			  }
			  else
			  {
				   //处理最右侧列像素值
				  for(int kr = 0; kr<keRows; kr++)
				  {
					  uchar* inPtr = padded.ptr<uchar>(i-(keRows-1)/2+kr);
					  int rkc= keCols-1;
					   uchar rightVal = inPtr[j-(keCols-1)/2+rkc];
					   hist[rightVal] = hist[rightVal]+1;
					   if(rightVal<=ucMed)
						   medNum = medNum+1;

				  }
			    

			  }
			  //计算中值
			  if(medNum<nthd)
			  {
				  uchar ucTemp = ucMed;
				  for(; ucTemp<255; ucTemp++)
				  {
					  medNum+=hist[ucTemp+1];
					  if(medNum>=nthd)
					  {
						  ucMed = ucTemp + 1;
						  break;
					  }	
				  }
			  }
			  else if(medNum>nthd)
			  {
				  uchar ucTemp = ucMed;
				  for(;ucTemp>0; ucTemp--)
				  {
					  medNum-=hist[ucTemp-1];
					  if(medNum<=nthd)
					  {
						  ucMed = ucTemp -1;
						  break;
					  }
					  if(medNum<0)
						  medNum =0;
				  }
			  }
			  
			  

			  if(j -(keCols-1)/2<0||i -(keCols-1)/2>outCols-1)
				  continue;
			 
			  outPtr[j -(keCols-1)/2]  = ucMed;

			 //处理最左侧像素
			  for(int kr = 0; kr<keRows; kr++)
			  {
				  uchar* inPtr = padded.ptr<uchar>(i-(keRows-1)/2+kr);
				  int lkc =0;
				  uchar leftVal = inPtr[j-(keCols-1)/2+lkc];
				  hist[leftVal] = hist[leftVal]-1;
				  if(leftVal<=ucMed)
					  medNum = medNum-1;

			  }

			   bBeignNewRow = false;
		  }

	  }

 }

 void  TESTALGORITHMS_API otsuThreshold(Mat _inImg, Mat & _outImg, uchar& ucThd)
 {
	 if(_inImg.empty())
	 {
		 cout<<"some error in input image"<<endl;
		 return;
	 }
	 int hist[256] ={0};
	 int nRows = _inImg.rows;
	 int nCols = _inImg.cols;
	 int Total_sum =0;
	 for(int i = 0; i<nRows; i++)
	 {
		 uchar* ptr = _inImg.ptr<uchar>(i);
		 for(int j =0; j<nCols; j++)
		 {
			 hist[ptr[j]]++;
			 Total_sum+=ptr[j];
		 }
	 }
	 //
	 //设定初始的分割阈值；
	 int pixelArea = nRows*nCols;
	 float fW_back_Ration =0.0f, fW_fore_ratio =0.0f;
	 int Ave_back = 0, Ave_fore =0;
	 int back_counts =0, fore_counts =0;
	 float fvariation_between = 0.0f;

	 float fMax_variation_between =0.0f;

	 int Back_Sum =0;
	 int Fore_sum =Total_sum;
	
	 

	 for(int ucTempThd =0; ucTempThd<256; ucTempThd++)
	 {
		 if(ucTempThd == 100)
			 int a =0;
		 back_counts+=hist[ucTempThd];
		 fore_counts = pixelArea - back_counts;

		 Back_Sum+=ucTempThd*hist[ucTempThd];
		 Fore_sum = Total_sum - Back_Sum;

		 fW_back_Ration = back_counts*1.0/pixelArea;
		 fW_fore_ratio = fore_counts*1.0/pixelArea;

		 if(back_counts==0)
			 continue;
		 Ave_back = Back_Sum/back_counts;
		 if(fore_counts==0)
			 continue;
		  Ave_fore = Fore_sum/fore_counts;

		 fvariation_between = fW_fore_ratio*fW_back_Ration*(Ave_back-Ave_fore)*(Ave_back-Ave_fore);
		 if(fvariation_between>fMax_variation_between)
		 {
		      fMax_variation_between = fvariation_between;
			  ucThd = (uchar)ucTempThd;
			  if(ucTempThd == 236)
			  {
				  int a= 0;
			  }
		 }
		  //threshold(_inImg, _outImg, ucThd, 255, CV_THRESH_BINARY);
		  //int a=0;
	 }
	 threshold(_inImg, _outImg, ucThd, 255, CV_THRESH_BINARY);
	 return ;
 }

 void TESTALGORITHMS_API wellnerThreshold(Mat _inImg, Mat& _outImg, const int nRadius, const int nRatio)
 {
	 if(_inImg.empty())
	 {
		 cout<<"some error in input image"<<endl;
		 return;
	 }
	 Mat Blur(_inImg.size(), CV_32FC1);
	 Mat kernel = Mat::ones(nRadius, nRadius, CV_8UC1);
	 filter2D(_inImg, Blur, Blur.depth(), kernel);
	 int nWindowSize = nRadius*nRadius;
	 float factor = (100-nRatio)*1.0f/100;
	 
	// Blur = Blur/(nRadius*nRadius);
	 int nRows = Blur.rows;
	 int nCols = Blur.cols;
	 for(int i =0; i<nRows; i++)
	 {
		 float * blur_ptr = Blur.ptr<float>(i);
		 uchar * in_ptr = _inImg.ptr<uchar>(i);
		 uchar * out_ptr = _outImg.ptr<uchar>(i);
		 for(int j =0; j<nCols; j++)
		 {
			 int  inPixelVal = in_ptr[j]*(nWindowSize);
			 if(inPixelVal<=(blur_ptr[j]*(factor)))
			 {
				 out_ptr[j] = 0;
			 }
			 else
			 {
				 out_ptr[j] = 0xff;
			 }
		 }
	 }
 }

 void TESTALGORITHMS_API wellnerThresholdEx(unsigned char* input, unsigned char*& bin, int width, int height)
 {
	 int S = width >> 3;
	 int T = 15;

	 unsigned long* integralImg = 0;
	 int i, j;
	 long sum=0;
	 int count=0;
	 int index;
	 int x1, y1, x2, y2;
	 int s2 = S/2;

	 bin = new unsigned char[width*height];
	 // create the integral image
	 integralImg = (unsigned long*)malloc(width*height*sizeof(unsigned long*));
	 for (i=0; i<width; i++)
	 {
		 // reset this column sum
		 sum = 0;
		 for (j=0; j<height; j++)
		 {
			 index = j*width+i;
			 sum += input[index];
			 if (i==0)
				 integralImg[index] = sum;
			 else
				 integralImg[index] = integralImg[index-1] + sum;
		 }
	 }
	 // perform thresholding
	 for (i=0; i<width; i++)
	 {
		 for (j=0; j<height; j++)
		 {
			 index = j*width+i;
			 // set the SxS region
			 x1=i-s2; x2=i+s2;
			 y1=j-s2; y2=j+s2;
			 // check the border
			 if (x1 < 0) x1 = 0;
			 if (x2 >= width) x2 = width-1;
			 if (y1 < 0) y1 = 0;
			 if (y2 >= height) y2 = height-1;
			 count = (x2-x1)*(y2-y1);
			 // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
			 sum = integralImg[y2*width+x2] -
				 integralImg[y1*width+x2] -
				 integralImg[y2*width+x1] +
				 integralImg[y1*width+x1];
			 if ((long)(input[index]*count) < (long)(sum*(100-T)/100))
				 bin[index] = 0;
			 else
				 bin[index] = 255;
		 }
	 }
	 free (integralImg);
 }
 void TESTALGORITHMS_API sobelGradient(Mat _inImg, Mat& grad)
 {
	 if(_inImg.empty())
	 {
		 cout<<"some error in input image"<<endl;
		 return;
	 }
	 grad = Mat(_inImg.size(), CV_32FC1, Scalar(0));
	 /*
	 Mat sobel_x = (Mat_<int>(3,3)<<-1,0,1,
		                             -2,0,2,
									 -1,0,1);
	 Mat sobel_y = (Mat_<int>(3,3)<<-1,-2,-1,
		                             0, 0, 0,
									 1, 2, 1);
									 */
	 Mat grad_x, grad_y;
	 Mat absGradx, absGrady;
	 Sobel(_inImg, grad_x, CV_32F,1,0);
	 convertScaleAbs(grad_x, absGradx);

	 Sobel(_inImg, grad_y, CV_32F,0,1);
	 convertScaleAbs(grad_y, absGrady);

	 addWeighted(absGradx,0.5,absGrady,0.5,0,grad);

 }

 //Gabor Filter
 TESTALGORITHMS_API bool Common_Creat_GaborFilter(const int nGaborW, const int nGaborH, const float dFre, const double dSigma, const double dTheta, const double dGamma,const int nPsi, Mat &RealGaborFilter,Mat& ImaginaryGaborFilter)
 {
	 if(RealGaborFilter.empty())
	 {
		 RealGaborFilter = Mat(Size(nGaborW, nGaborH), CV_32FC1, Scalar(0));
	 }
	 if(ImaginaryGaborFilter.empty())
	 {
		 ImaginaryGaborFilter = Mat(Size(nGaborW, nGaborH), CV_32FC1, Scalar(0));

	 }
	 

	 float fTemp1=0.0f, fTemp2=0.0f, fTemp3=0.0f, fTemp4=0.0f;
	 double dCosVal=cos(dTheta*CV_PI/180);
	 double dSinVal=sin(dTheta*CV_PI/180);
	 float fGaborRealMean=0.0,fGaborImaginaryMean=0.0;

	 for(int i=0; i<=nGaborH-1; i++)
	 {
		float* real_Ptr = RealGaborFilter.ptr<float>(i);
		float* im_Ptr = ImaginaryGaborFilter.ptr<float>(i);

		 for(int j=0; j<=nGaborW-1; j++)
		 {
			 int x=j-(nGaborW-1)/2;
			 int y=i-(nGaborH-1)/2;

			 fTemp1=(float)(x*dCosVal+y*dSinVal);
			 fTemp2=(float)(-x*dSinVal+y*dCosVal);
			 fTemp3=(float)(exp(-(pow(fTemp1,2)+dGamma*pow(fTemp2,2))/(2*dSigma*dSigma))*cos(2*CV_PI*dFre*fTemp1+nPsi));
			 fTemp4=(float)(exp(-(pow(fTemp1,2)+dGamma*pow(fTemp2,2))/(2*dSigma*dSigma))*sin(2*CV_PI*dFre*fTemp1+nPsi));
			
			 real_Ptr[j]=fTemp3;
			 fGaborRealMean+=(fTemp3);
			 im_Ptr[j]=fTemp4;
			 fGaborImaginaryMean+=(fTemp4);
		 }
	 }
	 //减掉均值，是为了在平滑区域，Gabor输出为0
	 fGaborRealMean=fGaborRealMean/(nGaborW*nGaborH);
	 fGaborImaginaryMean=fGaborImaginaryMean/(nGaborH*nGaborW);

	 for(int i=0; i<=nGaborH-1; i++)
	 {
		 float* real_Ptr = RealGaborFilter.ptr<float>(i);
		 float* im_Ptr = ImaginaryGaborFilter.ptr<float>(i);
		 for(int j=0; j<=nGaborW-1; j++)
		 {
			real_Ptr[j]-= fGaborRealMean;
			im_Ptr[j] -= fGaborImaginaryMean;
		 }
	 }

	 return TRUE;
 }
 TESTALGORITHMS_API void lineDetect(Mat _inImg, Mat& _outImg, vector<Vec2f>vecLineParams, const int _lineDetectType)
 {
	//#define DRAW_HOUGH_LINES 
	 Mat edgeMat;
	 vector<Vec4i> vecProbaLinesParams;
	 Canny(_inImg, edgeMat, 50, 150);
	 _outImg = edgeMat.clone();

	 switch (_lineDetectType)
	 {
	 case Hogh_Stand:
		  HoughLines(edgeMat,vecLineParams, 1,CV_PI/90, 50);
		 break;
	 case Hough_Pro:
		 HoughLinesP(edgeMat,vecProbaLinesParams, 1, CV_PI/180, 10, 10, 5);
		 break;
	 case Hogh_MultScale:
		  HoughLines(edgeMat,vecLineParams, 1,CV_PI/90, 50, 2.0,2.0);
		 break;
	 case LSD:
		 break;
	 default:
		 break;
	 }
	 //

#ifdef DRAW_HOUGH_LINES
	 int nCols = edgeMat.cols;
	 int nRows = edgeMat.rows;

	 if(_lineDetectType==Hough_Pro)
	 {
		 for(int i =0; i<vecProbaLinesParams.size(); i++)
		 {
			 if(vecProbaLinesParams.size()>1)
			 {
				 for(int k =0; k<vecProbaLinesParams.size(); k++)
				 {
					 Point pt1, pt2;
					 pt1.x =vecProbaLinesParams[k][0];
					 pt1.y = vecProbaLinesParams[k][1];

					 pt2.x = vecProbaLinesParams[k][2];
					 pt2.y = vecProbaLinesParams[k][3];

					 line(_inImg,pt1,pt2,Scalar(255,0,0));
				 }
			 }

		 }
	 }
	 else
	 {
		 for(int i =0; i<vecLineParams.size(); i++)
		 {
			 float fRho = vecLineParams[i][0];
			 float fTheta = vecLineParams[i][1];

			 vector<Vec2i> vecLineCoordinates;
			 for(int rw =0; rw<nRows; rw++)
			 {
				 uchar* ptr = edgeMat.ptr<uchar>(rw);
				 for(int cl =0; cl<nCols; cl++)
				 {
					 if(ptr[cl]!=0)
					 {
						 float fcurRho  = cl*cos(fTheta)+rw*sin(fTheta);
						 if(fabs(fcurRho - fRho)<=1)
						 {
							 Vec2i pts; 
							 pts[0] = cl;
							 pts[1] = rw;
							 vecLineCoordinates.push_back(pts);
						 }
					 }
				 }
			 }
			 if(vecLineCoordinates.size()>1)
			 {
				 for(int k =1; k<vecLineCoordinates.size(); k++)
				 {
					 Point pt1, pt2;
					 pt1.x =vecLineCoordinates[k-1][0];
					 pt1.y = vecLineCoordinates[k-1][1];

					 pt2.x = vecLineCoordinates[k][0];
					 pt2.y = vecLineCoordinates[k][1];

					 line(_inImg,pt1,pt2,Scalar(255,0,0));
				 }
			 }

		 }
	 }
	
	 imwrite("..\\sample\\result\\detect_line.bmp", _inImg);
#endif

 }

 TESTALGORITHMS_API void lineFit(Mat& _outImg, vector<Point>fitData, Vec4f &vecLineParams, const int _lineFitType)
 {
	 if(_outImg.empty())
	 {
	     cout<<"out Img is empty"<<endl;
	 }
	 switch (_lineFitType)
	 {
	 case Emulator_Fit:
		 fitLine(fitData, vecLineParams, CV_DIST_L2,0,0,0);
		 break;
	 case RANSAC_Fit:
		 RansancFit(fitData, vecLineParams);
		 break;
	 default:
		 break;
	 }
 }


 Vec4f TotalLeastSquares(vector<cv::Point>& nzPoints, vector<int> ptOnLine)
 {
	 //if there are enough inliers calculate model
	 float x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
	 float dx2, dy2, dxy;
	 float t;
	 for( size_t i = 0; i < nzPoints.size(); ++i )
	 {
		 x += ptOnLine[i] * nzPoints[i].x;
		 y += ptOnLine[i] * nzPoints[i].y;
		 x2 += ptOnLine[i] * nzPoints[i].x * nzPoints[i].x;
		 y2 += ptOnLine[i] * nzPoints[i].y * nzPoints[i].y;
		 xy += ptOnLine[i] * nzPoints[i].x * nzPoints[i].y;
		 w += ptOnLine[i];
	 }

	 x /= w;
	 y /= w;
	 x2 /= w;
	 y2 /= w;
	 xy /= w;

	 //Covariance matrix
	 dx2 = x2 - x * x;
	 dy2 = y2 - y * y;
	 dxy = xy - x * y;

	 t = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;
	 cv::Vec4f line;
	 line[0] = (float) cos( t );
	 line[1] = (float) sin( t );

	 line[2] = (float) x;
	 line[3] = (float) y;

	 return line;
 }
 void RansancFit(vector<Point>fitData, Vec4f&vecLineParams)
 {
	 float t = 100;
	 float p =0.01;
	 float e = 1-p;
	 int T = 300;
	 SLine lineParam = LineFitRANSAC(t, p, e, 
		 T,fitData);
	 vecLineParams = lineParam.params;

 }

 SLine LineFitRANSAC(
	 float t,//distance from main line
	 float p,//chance of hitting a valid pair//successifullProbability
	 float e,//percentage of outliers
	 int T,//number of expected minimum inliers 
	 vector<Point>& nzPoints)
 {
	 int s = 2;//number of points required by the model
	 int N = (int)ceilf(log(1-p)/log(1 - pow(1-e, s)));//number of independent trials

	 vector<SLine> lineCandidates;
	 vector<int> ptOnLine(nzPoints.size());//is inlier
	 RNG rng((uint64)-1);
	 SLine line;
	 for (int i = 0; i < N; i++)
	 {
		 //pick two points
		 int idx1 = (int)rng.uniform(0, (int)nzPoints.size());
		 int idx2 = (int)rng.uniform(0, (int)nzPoints.size());
		 cv::Point p1 = nzPoints[idx1];
		 cv::Point p2 = nzPoints[idx2];

		 //points too close - discard
		 if (norm(p1- p2) < t)
		 {
			 continue;
		 }

		 //line equation ->  (y1 - y2)X + (x2 - x1)Y + x1y2 - x2y1 = 0 
		 float a = static_cast<float>(p1.y - p2.y);
		 float b = static_cast<float>(p2.x - p1.x);
		 float c = static_cast<float>(p1.x*p2.y - p2.x*p1.y);
		 //normalize them
		 float scale = 1.f/sqrt(a*a + b*b);
		 a *= scale;
		 b *= scale;
		 c *= scale;

		 //count inliers
		 int numOfInliers = 0;
		 for (size_t i = 0; i < nzPoints.size(); ++i)
		 {
			 cv::Point& p0 = nzPoints[i];
			 float rho      = abs(a*p0.x + b*p0.y + c);
			 bool isInlier  = rho  < t;
			 if ( isInlier ) numOfInliers++;
			 ptOnLine[i]    = isInlier;
		 }

		 if ( numOfInliers < T)
		 {
			 continue;
		 }

		 line.params = TotalLeastSquares( nzPoints, ptOnLine);
		 line.numOfValidPoints = numOfInliers;
		 lineCandidates.push_back(line);
	 }

	 int bestLineIdx = 0;
	 int bestLineScore = 0;
	 for (size_t i = 0; i < lineCandidates.size(); i++)
	 {
		 if (lineCandidates[i].numOfValidPoints > bestLineScore)
		 {
			 bestLineIdx = i;
			 bestLineScore = lineCandidates[i].numOfValidPoints;
		 }
	 }

	 if ( lineCandidates.empty() )
	 {
		 return SLine();
	 }
	 else
	 {
		 return lineCandidates[bestLineIdx];
	 }
 }

 TESTALGORITHMS_API int circleDetect(Mat _inImg, Mat& _outImg, vector<Vec3f>&vecCircleParams, const int _circleDetectType)
 {
	double canny_threshold = 150;
	double circle_threshold = 0.8;
	int numIterations = 10000;
	int index =-1;
	circleRANSAC(_inImg, vecCircleParams, canny_threshold, circle_threshold,numIterations, index);
	return index;
 }

 void circleRANSACEx(Mat &image, vector<Vec3f> &circles, double circle_threshold, int numIterations, int &nBiggestCircleIndex)
 {
	 CV_Assert(image.type() == CV_8UC1);
	 circles.clear();

	 // Edge Detection
	// Mat edges;
	// Canny(image, edges, MAX(canny_threshold/2,1), canny_threshold, 3);

	 // Create point set from Canny Output
	 uchar ucThd = 100;
	 int nCountThd = 25;

	 vector<Point2d> points;
	 for(int r = 0; r < image.rows; r++)
	 {
		uchar *ptr = image.ptr<uchar>(r);
		 for(int c = 0; c < image.cols; c++)
		 {
			 if(ptr[c] >ucThd)
			 {
				 points.push_back(Point2d(c,r));
			 }
		 }	
	 }


	 //产生4优选参数点
	 int nSum =0;
	 int topy,bottomy,leftx,rightx; //几个方面

	 //定位上边界
	 for(int r = 0; r < image.rows; r++)
	 {
		 nSum =0;
		 uchar *ptr = image.ptr<uchar>(r);
		 for(int c = 0; c < image.cols; c++)
		 {
			 if(ptr[c] >ucThd)
			 {
				//points.push_back(Point2d(c,r));
				 nSum++;
				 if(nSum>nCountThd)
				 {
					 topy = r;
					 break;
				 }
			 }
		 }	
	 }

	 //定位下边界
	 for(int r = image.rows-1; r > 0; r--)
	 {
		 nSum =0;
		 uchar *ptr = image.ptr<uchar>(r);
		 for(int c = 0; c < image.cols; c++)
		 {
			 if(ptr[c] >ucThd)
			 {
				 //points.push_back(Point2d(c,r));
				 nSum++;
				 if(nSum>nCountThd)
				 {
					 bottomy = r;
					 break;
				 }
					 
			 }
		 }	
	 }
	 //定位左边界
	
	 int nStep = image.step;
	 for(int c = image.cols-1; c > 0; c--)
	 {
		 nSum =0;
		 uchar * ptr = image.data;
		 for(int r = 0; r < image.rows; r++)
		 {
			 if((ptr)[c] >ucThd)
			 {
				 //points.push_back(Point2d(c,r));
				 nSum++;
				 if(nSum>nCountThd)
				 {
					 rightx = c;
					 break;
				 }
					 
			 }
			 ptr +=nStep;
		 }	
	 }

	 //定位右边界

	
	 nStep = image.step;
	 for(int c = 0; c < image.cols; c++)
	 {
		 nSum =0;
		  uchar * ptr = image.data;
		 for(int r = 0; r < image.rows; r++)
		 {
			 if((ptr)[c] >ucThd)
			 {
				 //points.push_back(Point2d(c,r));
				 nSum++;
				 if(nSum>nCountThd)
				 {
					 leftx = c;
					 break;
				 }
					 
			 }
			 ptr +=nStep;
		 }	
	 }

	 //计算centerX,ceterY, nRadius
	 Point2d Center;
	 Center.x = (leftx+rightx)/2;
	 Center.y = (bottomy+topy)/2;
	 double Radius = (abs(rightx - leftx)+abs(bottomy-topy))/4;

	 double radius_tolerance = 3;
	 int    points_dist_tolerance = 10;

	 int nBiggestCircleSize =0;
	 int points_threshold = 100;

	 RNG rng;

	 for(int iteration = 0; iteration < numIterations; iteration++) 
	 {
		 
		 if(abs(abs(rightx - leftx) - abs(bottomy-topy))>points_dist_tolerance)
			 continue;
		 
		 // vote
		 vector<int> votes;
		 vector<int> no_votes;
		 for(int i = 0; i < (int)points.size(); i++) 
		 {
			 double vote_radius = norm(points[i] - Center);

			 if(abs(vote_radius - Radius) < radius_tolerance) 
			 {
				 votes.push_back(i);
			 }
			 else
			 {
				 no_votes.push_back(i);
			 }
		 }
		 if( (float)votes.size() / (2.0*CV_PI*Radius) >= circle_threshold )
		 {
			 circles.push_back(Vec3f(Center.x,Center.y,Radius));
			 if(votes.size()>nBiggestCircleSize)
			 {
				 nBiggestCircleSize = votes.size();
				 nBiggestCircleIndex  = circles.size()-1;
			 }

			 // remove points from the set so they can't vote on multiple circles
			 std::vector<Point2d> new_points;
			 for(int i = 0; i < (int)no_votes.size(); i++)
			 {
				 new_points.push_back(points[no_votes[i]]);
			 }
			 points.clear();
			 points = new_points;		
		 }

		 // stop RANSAC if there are few points left
		 if((int)points.size() < points_threshold)
			 break;

		 Center.x = rng.uniform(-5, 5); //随机生成
		 Center.y = rng.uniform(-5,5);   
		 Radius =   rng.uniform(-5,5);

		 //
	 }
	 return;
	
 }

 void circleRANSAC(Mat &image, vector<Vec3f> &circles, double canny_threshold, double circle_threshold, int numIterations, int &nBiggestCircleIndex)
 {
	 CV_Assert(image.type() == CV_8UC1 || image.type() == CV_8UC3);
	 circles.clear();

	 // Edge Detection
	 Mat edges;
	 Canny(image, edges, MAX(canny_threshold/2,1), canny_threshold, 3);

	 // Create point set from Canny Output
	 std::vector<Point2d> points;
	 for(int r = 0; r < edges.rows; r++)
	 {
		 for(int c = 0; c < edges.cols; c++)
		 {
			 if(edges.at<unsigned char>(r,c) == 255)
			 {
				 points.push_back(cv::Point2d(c,r));
			 }
		 }	
	 }

	 // 4 point objects to hold the random samples
	 Point2d pointA;
	 Point2d pointB;
	 Point2d pointC;
	 Point2d pointD;

	 // distances between points
	 double AB;
	 double BC;
	 double CA;
	 double DC;

	 // varibales for line equations y = mx + b
	 double m_AB;
	 double b_AB;
	 double m_BC;
	 double b_BC;

	 // varibles for line midpoints
	 double XmidPoint_AB;
	 double YmidPoint_AB;
	 double XmidPoint_BC;
	 double YmidPoint_BC;

	 // variables for perpendicular bisectors
	 double m2_AB;
	 double m2_BC;
	 double b2_AB;
	 double b2_BC;

	 // RANSAC
	 cv::RNG rng; 
	 int min_point_separation = 10; // change to be relative to image size?
	 int colinear_tolerance = 1; // make sure points are not on a line
	 int radius_tolerance = 3; // change to be relative to image size?
	 int points_threshold = 100; //should always be greater than 4
	 //double min_circle_separation = 10; //reject a circle if it is too close to a previously found circle
	 //double min_radius = 10.0; //minimum radius for a circle to not be rejected

	 int x,y;
	 Point2d center;
	 double radius;

	 nBiggestCircleIndex =-1;
	 int nBiggestCircleSize=0;
	 // Iterate
	 for(int iteration = 0; iteration < numIterations; iteration++) 
	 {
		 //std::cout << "RANSAC iteration: " << iteration << std::endl;

		 // get 4 random points
		 pointA = points[rng.uniform((int)0, (int)points.size())];
		 pointB = points[rng.uniform((int)0, (int)points.size())];
		 pointC = points[rng.uniform((int)0, (int)points.size())];
		 pointD = points[rng.uniform((int)0, (int)points.size())];

		 // calc lines
		 AB = norm(pointA - pointB);
		 BC = norm(pointB - pointC);
		 CA = norm(pointC - pointA);
		 DC = norm(pointD - pointC);

		 // one or more random points are too close together
		 if(AB < min_point_separation || BC < min_point_separation || CA < min_point_separation || DC < min_point_separation)
			 continue;

		 //find line equations for AB and BC
		 //AB
		 m_AB = (pointB.y - pointA.y) / (pointB.x - pointA.x + 0.000000001); //avoid divide by 0
		 b_AB = pointB.y - m_AB*pointB.x;

		 //BC
		 m_BC = (pointC.y - pointB.y) / (pointC.x - pointB.x + 0.000000001); //avoid divide by 0
		 b_BC = pointC.y - m_BC*pointC.x;


		 //test colinearity (ie the points are not all on the same line)
		 if(abs(pointC.y - (m_AB*pointC.x + b_AB + colinear_tolerance)) < colinear_tolerance) 
			 continue;

		 //find perpendicular bisector
		 //AB
		 //midpoint
		 XmidPoint_AB = (pointB.x + pointA.x) / 2.0;
		 YmidPoint_AB = m_AB * XmidPoint_AB + b_AB;
		 //perpendicular slope
		 m2_AB = -1.0 / m_AB;
		 //find b2
		 b2_AB = YmidPoint_AB - m2_AB*XmidPoint_AB;

		 //BC
		 //midpoint
		 XmidPoint_BC = (pointC.x + pointB.x) / 2.0;
		 YmidPoint_BC = m_BC * XmidPoint_BC + b_BC;
		 //perpendicular slope
		 m2_BC = -1.0 / m_BC;
		 //find b2
		 b2_BC = YmidPoint_BC - m2_BC*XmidPoint_BC;

		 //find intersection = circle center
		 x = (b2_AB - b2_BC) / (m2_BC - m2_AB);
		 y = m2_AB * x + b2_AB;	
		 center = Point2d(x,y);
		 radius = cv::norm(center - pointB);

		 /// geometry debug image
		 if(false)
		 {
			 Mat debug_image = edges.clone();
			 cvtColor(debug_image, debug_image, CV_GRAY2RGB);

			 Scalar pink(255,0,255);
			 Scalar blue(255,0,0);
			 Scalar green(0,255,0);
			 Scalar yellow(0,255,255);
			 Scalar red(0,0,255);

			 // the 3 points from which the circle is calculated in pink
			 circle(debug_image, pointA, 3, pink);
			 circle(debug_image, pointB, 3, pink);
			 circle(debug_image, pointC, 3, pink);

			 // the 2 lines (blue) and the perpendicular bisectors (green)
			 line(debug_image,pointA,pointB,blue);
			 line(debug_image,pointB,pointC,blue);
			 line(debug_image,Point(XmidPoint_AB,YmidPoint_AB),center,green);
			 line(debug_image,Point(XmidPoint_BC,YmidPoint_BC),center,green);

			 circle(debug_image, center, 3, yellow); // center
			 circle(debug_image, center, radius, yellow);// circle

			 // 4th point check
			 //circle(debug_image, pointD, 3, red);

			 imshow("ransac debug", debug_image);
			 waitKey(0);
		 }

		 //check if the 4 point is on the circle
		 if(abs(cv::norm(pointD - center) - radius) > radius_tolerance) 
		        continue;

		 // vote
		 std::vector<int> votes;
		 std::vector<int> no_votes;
		 for(int i = 0; i < (int)points.size(); i++) 
		 {
			 double vote_radius = norm(points[i] - center);

			 if(abs(vote_radius - radius) < radius_tolerance) 
			 {
				 votes.push_back(i);
			 }
			 else
			 {
				 no_votes.push_back(i);
			 }
		 }

		 // check votes vs circle_threshold
		 if( (float)votes.size() / (2.0*CV_PI*radius) >= circle_threshold )
		 {
			 circles.push_back(Vec3f(x,y,radius));
			 if(votes.size()>nBiggestCircleSize)
			 {
				 nBiggestCircleSize = votes.size();
				 nBiggestCircleIndex  = circles.size()-1;
			 }
			 
			 // voting debug image
			 if(false)
			 {
				 Mat debug_image2 = edges.clone();
				 cvtColor(debug_image2, debug_image2, CV_GRAY2RGB);

				 Scalar yellow(0,255,255);
				 Scalar green(0,255,0);

				 circle(debug_image2, center, 3, yellow); // center
				 circle(debug_image2, center, radius, yellow);// circle

				 // draw points that voted
				 for(int i = 0; i < (int)votes.size(); i++)
				 {
					 circle(debug_image2, points[votes[i]], 1, green);
				 }

				 imshow("ransac debug", debug_image2);
				 waitKey(0);
			 }

			 // remove points from the set so they can't vote on multiple circles
			 std::vector<Point2d> new_points;
			 for(int i = 0; i < (int)no_votes.size(); i++)
			 {
				 new_points.push_back(points[no_votes[i]]);
			 }
			 points.clear();
			 points = new_points;		
		 }

		 // stop RANSAC if there are few points left
		 if((int)points.size() < points_threshold)
			 break;
	 }

	 return;
 }
 void TESTALGORITHMS_API circleFit(Mat& plotMat, vector<Point>circleData)
 {
	 //Mat plotMat(1024,1024, CV_8UC3, Scalar::all(100));

	 RotatedRect rotRect = fitEllipse(circleData);
	 Point2f center = rotRect.center;
	 Size2f sz = rotRect.size;

	 int nRadius = (int)(sz.width+sz.height)>>2;

	 circle(plotMat, center, nRadius, Scalar(0,0,255));
	 
	 imwrite("..\\sample\\result\\circle_data_fit.bmp", plotMat);
 }

 TESTALGORITHMS_API bool Common_CircleFit(vector<Point> circleData, float & fCenterX,float & fCenterY,float & fRad)
 {
	 if (circleData.empty())
	 {
		 return FALSE;
	 }

	 int nPtSize = circleData.size();
   // #define DataParam_MinCirclePoint 1800

	 float fCoefficent[3];
	 float *fPointData = new float [nPtSize*3];
	 
	 float *fConst = new float[nPtSize];
	 memset(fPointData,0x00,sizeof(fPointData));
	 memset(fConst,0x00,sizeof(fConst));
	 memset(fCoefficent,0x00,sizeof(fCoefficent));

	 int nCount= 0;

	 for ( int i=0;i<nPtSize;i++)
	 {

		 fPointData[nCount*3+0] = (float)circleData[i].x;
		 fPointData[nCount*3+1] = (float)circleData[i].y;
		 fPointData[nCount*3+2] = 1.0f;
		 fConst[nCount] =(float)(circleData[i].x *circleData[i].x + circleData[i].y * circleData[i].y);
		 nCount++;
	 }

	 CvMat A = cvMat(nCount, 3, CV_32F, fPointData);
	 CvMat B = cvMat(nCount, 1, CV_32F, fConst);
	 CvMat X = cvMat(3, 1, CV_32F, fCoefficent);

	 cvSolve(&A, &B, &X);
	 float fx = fCoefficent[0]/2.0f;
	 float fy = fCoefficent[1]/2.0f;
	 float fr2 = fCoefficent[2] + fx*fx + fy*fy;
	 float fr  = fr2>0 ? sqrt(fr2) : 0;

	 if (fx > 0 && fy>0 && fr >0 )
	 {
		 fCenterX = fx;
		 fCenterY = fy;
		 fRad = fr;
		 return TRUE;
	 }
	 else
	 {
		 fCenterX = 0.0f;
		 fCenterY = 0.0f;
		 fRad = 0.0f;
		 return FALSE;
	 }
 }
 void randomCircleData(Mat& _plotMat, Vec3f circle, vector<Point>&circleData, const int data_nums)
 {
	 if(_plotMat.empty() ||data_nums<5)
	 {
		 cout<<"random circle data is wrong"<<endl;
		 return;
	 }

	 circleData.clear();
	 Point2f center;
	 float fCenterX = circle[0];
	 float fCenterY = circle[1];
	 float fRadius =  circle[2];

	 //int nAngNum = 360/data_nums
	 double* dSin = new double[data_nums];
	 double* dCos = new double[data_nums];


	 RNG rng(0xFFFFFFFF);

	 for (int i=0;i<data_nums;i++)
	 {
		 dCos[i] = cos((float)i*2.0f*CV_PI/data_nums);
		 dSin[i] = sin((float)i*2.0f*CV_PI/data_nums);

		 int x = fCenterX + fRadius*dSin[i]+rng.gaussian(20);
		 int y = fCenterY + fRadius*dCos[i]+rng.gaussian(10);

		 //角度
		 if(x<0||x>_plotMat.cols-1)
			 continue;
		 if(y<0||y>_plotMat.rows-1)
			 continue;
		 _plotMat.at<Vec3b>(x,y)[0] = 0;
		 _plotMat.at<Vec3b>(x,y)[1] = 255;
		 _plotMat.at<Vec3b>(x,y)[2] = 0;

		 circleData.push_back(Point(x,y));

	 }

	 //

	 //
	 if(dSin)
		 delete [] dSin;
	 dSin = NULL;
	 if(dCos)
		 delete [] dCos;
	 dCos = NULL;
 }

 void TESTALGORITHMS_API drawHist(Mat& histMat, int* pHist, const int cnt, const int nMapWidth, const int nMapHeight, const Scalar scalar, const int hist_type)
 {
	 if(histMat.empty())
	 {
		  histMat=Mat(Size(nMapWidth, nMapHeight), CV_8UC3, Scalar::all(30));
	 }
	
	 int min = 0xffffffff;
	 int max = 0x00000000;
	 for(int i =0; i<cnt; i++)
	 {
	     if(min>pHist[i])
		 {
	         min = pHist[i];
		 }
		 if(max<pHist[i])
		 {
			 max = pHist[i];
		 }
	 }

	 //normalization
	 for(int i =0; i<cnt; i++)
	 {
		 pHist[i] =(int) (0.8*nMapHeight*(pHist[i]-min)/(max- min));
	 }

	 int k =0;
	 int step = nMapWidth/cnt;
	 for(int i =0; i<nMapWidth; i+=step, k++)
	 {
		 int y = pHist[k];
		 if(hist_type == Hist_Bar)
		 {
	          line(histMat, Point(i, nMapHeight-y-1), Point(i,nMapHeight-1), scalar,2);
		 }
		 else if(hist_type == Hist_Line)
		 {
	         if(k<1||k>=cnt)
				 continue;
			 line(histMat, Point(i-3,nMapHeight-pHist[k-1]-1), Point(i, nMapHeight-pHist[k]-1), scalar, 2);
		 }
		 
	 }
 }

 //反向投影算法

 void calcBackProject(Mat src,  Rect src_roi,  const int nBinNum,  Mat inMat,   Mat& backProjMat)
 {
	 if(src.empty()||src_roi.width<=0||src_roi.height<=0)
		 return;

	 //计算模板图像直方图

	 
	 int * pHist = new int[nBinNum];
	 memset(pHist, 0x00, sizeof(int)*nBinNum);
	 int nBinStep = 256/nBinNum;
	 
	 if(src.channels() == 1)
	 {
		 for(int i= src_roi.y; i<src_roi.y+src_roi.height-1; i++)
		 {
			 uchar * ptr = src.ptr<uchar>(i);
			 for(int j = src_roi.x; j<src_roi.x+src_roi.width; j++)
			 {
				int nBinIndex = cvRound(ptr[j]/(nBinStep));
				 pHist[nBinIndex]++;
			 }
		 }
	 }
	


	 int min = 0xffffffff;
	 int max = 0x00000000;
	 for(int i =0; i<nBinNum; i++)
	 {
		 if(min>pHist[i])
		 {
			 min = pHist[i];
		 }
		 if(max<pHist[i])
		 {
			 max = pHist[i];
		 }
	 }

	 //normalization
	 for(int i =0; i<nBinNum; i++)
	 {
		 pHist[i] =(int) (255*(pHist[i]-min)/(max- min));
	 }

	 for(int i= 0; i<inMat.rows; i++)
	 {
		 uchar * inptr = inMat.ptr<uchar>(i);
		 uchar * backPtr = backProjMat.ptr<uchar>(i);
		 for(int j = 0; j<inMat.cols; j++)
		 {
			int nBinIndex =  cvRound(inptr[j]/nBinStep);
			backPtr[j] = (uchar)(pHist[nBinIndex]);
		 }
	 }

	 //imwrite("..\\sample\\result\\BackProjection_Image.bmp", backProjMat);
 }

 //基于递归的实现
 //基于8领域
 void TESTALGORITHMS_API  FloodFilled(Point seedPt, Mat src, Mat& dst, Scalar curScalar, Scalar scalar, uchar& uLowThd, uchar& uHighThd)
 {
	 if(src.empty()|| dst.empty())
		 return;
	 if(src.depth()!=dst.depth())
	 {
		 return;
	 }
	 int nX =  seedPt.x;
	 int nY =  seedPt.y;

	 //确保高值不越界
	 if(255-curScalar.val[0]<uHighThd)
	 {
		  uHighThd = 255 - curScalar.val[0];
	 }
	 if(255-curScalar.val[1]<uHighThd)
	 {
		 uHighThd = 255 - curScalar.val[1];
	 }
	 if(255-curScalar.val[2]<uHighThd)
	 {
		 uHighThd = 255 - curScalar.val[2];
	 }


	 //确保低值不越界
	 if(curScalar.val[0] -0 <uLowThd)
	 {
		 uLowThd = curScalar.val[0];
	 }
	 if(curScalar.val[1] -0 <uLowThd)
	 {
		 uLowThd = curScalar.val[1];
	 }

	 if(curScalar.val[2] -0 <uLowThd)
	 {
		 uLowThd = curScalar.val[2];
	 }
	Vec3b srcPixelScalar = src.at<Vec3b>(nY, nX);

	 if(   srcPixelScalar[0]<=curScalar.val[0]+uHighThd &&   srcPixelScalar[0]>=curScalar.val[0]-uLowThd
		 && srcPixelScalar[1]<=curScalar.val[1]+uHighThd && srcPixelScalar[1]>=curScalar.val[1]-uLowThd
		 && srcPixelScalar[2]<=curScalar.val[2]+uHighThd && srcPixelScalar[2]>=curScalar.val[2]-uLowThd)
	 {
		 
		 dst.at<Vec3b>(nY,nX) = Vec3b(scalar.val[0],scalar.val[1],scalar.val[2]);

		 


		 Point leftPt, topLeftPt, topPt, topRightPt, rightPt, rightDnPt, dnPt,leftDnPt;
		 if(nX>0&&nX<src.cols&&nY>0&&nY<src.rows)
		 {
			 leftDnPt.x = nX -1;
			 leftDnPt.y = nY;
			 // if(dst.at<Vec3b>(nY,nX-1)[0] == 0&&dst.at<Vec3b>(nY,nX-1)[1] == 0&&dst.at<Vec3b>(nY,nX-1)[2] == 0)
			 {
				 FloodFilled(leftDnPt, src,  dst, curScalar, scalar, uLowThd, uHighThd);
			 }

			 topLeftPt.x = nX -1;
			 topLeftPt.y = nY -1;
			 //if(dst.at<Vec3b>(nY-1,nX-1)[0] == 0&&dst.at<Vec3b>(nY-1,nX-1)[1] == 0&&dst.at<Vec3b>(nY-1,nX-1)[2] == 0)
			 {
				 FloodFilled(topLeftPt, src,  dst, curScalar, scalar, uLowThd, uHighThd);
			 }

			 topPt.x = nX;
			 topPt.y = nY -1;
			 // if(dst.at<Vec3b>(nY-1,nX)[0] == 0&&dst.at<Vec3b>(nY-1,nX)[1] == 0&&dst.at<Vec3b>(nY-1,nX)[2] == 0)
			 {
				 FloodFilled(topLeftPt, src, dst, curScalar, scalar, uLowThd, uHighThd);
			 }


			 topRightPt.x = nX+1;
			 topRightPt.y = nY -1;
			 // if(dst.at<Vec3b>(nY-1,nX+1)[0] == 0&&dst.at<Vec3b>(nY-1,nX+1)[1] == 0&&dst.at<Vec3b>(nY-1,nX+1)[2] == 0)
			 {
				 FloodFilled(topRightPt, src,  dst, curScalar, scalar, uLowThd, uHighThd);
			 }

			 rightPt.x = nX+1;
			 rightPt.y = nY;
			 // if(dst.at<Vec3b>(nY,nX+1)[0] == 0&&dst.at<Vec3b>(nY,nX+1)[1] == 0&&dst.at<Vec3b>(nY,nX+1)[2] == 0)
			 {
				 FloodFilled(rightPt, src,  dst, curScalar, scalar, uLowThd, uHighThd);
			 }

			 rightDnPt.x = nX+1;
			 rightDnPt.y = nY+1;
			 // if(dst.at<Vec3b>(nY+1,nX+1)[0] == 0&&dst.at<Vec3b>(nY+1,nX+1)[1] == 0&&dst.at<Vec3b>(nY+1,nX+1)[2] == 0)
			 {
				 FloodFilled(rightDnPt, src,  dst, curScalar, scalar, uLowThd, uHighThd);
			 }

			 dnPt.x = nX;
			 dnPt.y = nY+1;
			 // if(dst.at<Vec3b>(nY+1,nX)[0] == 0&&dst.at<Vec3b>(nY+1,nX)[1] == 0&&dst.at<Vec3b>(nY+1,nX)[2] == 0)
			 {
				 FloodFilled(dnPt, src,  dst, curScalar, scalar, uLowThd, uHighThd);
			 }

			 leftDnPt.x = nX-1;
			 leftDnPt.y = nY+1;
			 // if(dst.at<Vec3b>(nY+1,nX-1)[0] == 0&&dst.at<Vec3b>(nY+1,nX-1)[1] == 0&&dst.at<Vec3b>(nY+1,nX-1)[2] == 0)
			 {
				 FloodFilled(leftDnPt, src,  dst,curScalar, scalar, uLowThd, uHighThd);
			 }

		 }
	 }
 }
	 