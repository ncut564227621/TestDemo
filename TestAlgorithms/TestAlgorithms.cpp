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

