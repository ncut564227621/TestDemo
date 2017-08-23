#include "stdafx.h"
#include "floodFill.h"
#include<vector>

CFloodFilled::CFloodFilled(Mat& _src, Mat& _dst)
{
	src = _src;
	dst = _dst;
}

void CFloodFilled::floodFill8(Point seedPt, Scalar newColor, Scalar oldColor, uchar ucLower, uchar ucHigher, const int w, const int h)
{
	
	
	Vec3b newVecVal(newColor.val[0], newColor.val[1], newColor.val[2]);

	Vec3b lower(ucLower, ucLower,ucLower);
	Vec3b higher(ucHigher, ucHigher,ucHigher);

	int Hchar0 = (int)((int)newVecVal[0]+(int)higher[0]);
	int Hchar1 = (int)((int)newVecVal[1]+(int)higher[1]);
	int Hchar2 = (int)((int)newVecVal[2]+(int)higher[2]);

	Hchar0>255?Hchar0=255:Hchar0;
	Hchar1>255?Hchar1=255:Hchar1;
	Hchar2>255?Hchar2=255:Hchar2;

	int Lchar0 = (int)((int)newVecVal[0] - (int)lower[0]);
	int Lchar1 = (int)((int)newVecVal[1] - (int)lower[1]);
	int Lchar2 = (int)((int)newVecVal[2] - (int)lower[2]);

	Lchar0<0?Lchar0=0:Lchar0;
	Lchar1<0?Lchar1=0:Lchar1;
	Lchar2<0?Lchar2=0:Lchar2;

	bool bFlag0 = false;
	Vec3b srcVec = src.at<Vec3b>(seedPt.y, seedPt.x);

	
	if((Lchar0<=(int)srcVec[0] && (int)srcVec[0]<=Hchar0) && 
		(Lchar1<=(int)srcVec[1]&&(int)srcVec[1]<=Hchar1)&&
		(Lchar2<=(int)srcVec[2]&&(int)srcVec[2]<=Hchar2))
	{
		bFlag0 = true;
	}

	bool bFlag1 = true;
	Vec3b dstVec = dst.at<Vec3b>(seedPt.y, seedPt.x);
	if(dstVec[0]== newVecVal[0]&&dstVec[1] == newVecVal[1] && dstVec[2] == newVecVal[2])
	{
		bFlag1 = false;
	}

	if(seedPt.x>0 && seedPt.x<w && seedPt.y>0&& seedPt.y<h && bFlag0 && bFlag1)
	{
		dst.at<Vec3b>(seedPt.y, seedPt.x) = newVecVal;

		seedPt = Point(seedPt.x-1, seedPt.y);  // 左边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x-1, seedPt.y-1);  // 左上边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x-1, seedPt.y+1);  // 左下边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x+1, seedPt.y);  //右边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x+1, seedPt.y-1);  //右上边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x+1, seedPt.y+1);  //右下边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x, seedPt.y-1);  //上边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);

		seedPt = Point(seedPt.x, seedPt.y+1);  //下边
		floodFill8(seedPt,  newColor, oldColor, ucLower, ucHigher, w, h);
	}
}


void CFloodFilled::floodFill8Stack(Point  seedPt, Scalar setColor, Scalar seedColor, uchar ucLower, uchar ucHighert, const int w, const int h)
{
	// 目前该算法只适用于3通道的彩色图像
	
	if(w<0||w>src.cols||h<0||h>src.rows)
		return;
	if(seedPt.x<src.cols&&seedPt.x>=0&&seedPt.y<src.rows&&seedPt.y>=0)
	{
		m_floodFillStack.push(seedPt);
	}
	//
	int dx[8] = {-1,-1,0,1,1,1,0,-1};
	int dy[8] = {0,-1,-1,-1,0,1,1,1};

	Vec3b UpLimit(ucHighert, ucHighert, ucHighert);
	Vec3b LowLimit(ucLower,  ucLower,   ucLower);
	Vec3b seedVal(setColor.val[0], setColor.val[1], setColor.val[2]);


	while(!m_floodFillStack.empty())
	{
		Point pt = m_floodFillStack.top();


		m_floodFillStack.pop();

		for(int ii =0; ii<8; ii++)
		{
			int xx = pt.x + dx[ii];
			int yy = pt.y + dy[ii];
			if(xx<0||xx>dst.cols-1||yy<0||yy>dst.rows-1)
				continue;
			bool bFlag_0 = false;
			bool bFlag_1 = false;

			//目标像素值
			Vec3b dstColor = dst.at<Vec3b>(yy,xx);
			if(dstColor[0]!=seedVal[0] && dstColor[1]!=seedVal[1] && dstColor[2]!=seedVal[2])
			{
				bFlag_1 = true;
			}
			else
			{
	            continue;
			}

			Vec3b curColor = src.at<Vec3b>(yy, xx);
			if((int)curColor[0]<=(int)seedColor.val[0]+(int)UpLimit[0] && (int)curColor[0]>=(int)seedColor.val[0]-(int)LowLimit[0] &&
			   (int)curColor[1]<=(int)seedColor.val[1]+(int)UpLimit[1] && (int)curColor[1]>=(int)seedColor.val[1]-(int)LowLimit[1] &&
			   (int)curColor[2]<=(int)seedColor.val[2]+(int)UpLimit[2] && (int)curColor[2]>=(int)seedColor.val[2]-(int)LowLimit[2])
			{
	              bFlag_0 = true;
			}
			else
			{
				continue;
			}

			
			if(bFlag_0 && bFlag_1)
			{
	              m_floodFillStack.push(Point(xx,yy));
				  dst.at<Vec3b>(yy,xx) = seedVal;
			}
		}
		///for(int xx = pt.x + dx[ii])
	}
}

//scanLine FloodFill Algorithm
//stack friendly and fast floodfill algorithm
void CFloodFilled:: floodFillScanline(Point seedPt, Scalar setColor, Scalar seedColor, const uchar ucLow, const uchar ucHigh, const int w, const int h)
{
	int x = seedPt.x;
	int y = seedPt.y;

	Vec3b seedVal = Vec3b(seedColor.val[0], seedColor.val[1], seedColor.val[2]);
	Vec3b setVal =  Vec3b(setColor.val[0], setColor.val[1], setColor.val[2]);

	


	Vec3b src3b = src.at<Vec3b>(y, x);
	if(!((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
		&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
		(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh) ))
		return;
	
	Vec3b dst3b = dst.at<Vec3b>(y, x);
	if(dst3b ==setVal)
		return;

	int x1;
	//draw current scanline from start position to the right


	x1 = x;
	while(x1 < w  &&  ((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
		&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
		(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh) ))
	{
		dst.at<Vec3b>(y, x1) = setVal;
		x1++;
		src3b = src.at<Vec3b>(y, x1);
	}

	//此时的x1为连通域右边界
	//draw current scanline from start position to the left
	x1 = x - 1;
	src3b = src.at<Vec3b>(y, x1);
	while(x1 >= 0 &&  ((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
		&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
		(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh) ))
	{
		dst.at<Vec3b>(y, x1) = setVal;
		x1--;
		src3b = src.at<Vec3b>(y, x1);
	}
	//此时x1为连通域左边界

	//test for new scanlines above
	x1 = x;
	dst3b = dst.at<Vec3b>(y, x1);
	while(x1 < w &&  dst3b == setVal)
	{
		src3b = src.at<Vec3b>(y-1,x1);
		if(y > 0 && ((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
			&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
			(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh) ))
		{
			seedPt=Point(x1, y-1);
			floodFillScanline(seedPt, setColor, seedColor, ucLow, ucHigh, w, h);
		}
		x1++;
		dst3b = dst.at<Vec3b>(y-1, x1);
	}

	x1 = x - 1;
	dst3b = dst.at<Vec3b>(y, x1);
	while(x1 >= 0 &&  dst3b == setVal)
	{
		src3b = src.at<Vec3b>(y-1,x1);
		if(y > 0 && ((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
			&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
			(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh) ))
		{
			seedPt=Point(x1, y-1);
			floodFillScanline(seedPt, setColor, seedColor, ucLow, ucHigh, w, h);
		}
		x1--;
		dst3b = dst.at<Vec3b>(y-1, x1);
	}

	//test for new scanlines below
	x1 = x;
	dst3b = dst.at<Vec3b>(y, x1);
	while(x1 < w &&  dst3b == setVal)
	{
		src3b = src.at<Vec3b>(y+1,x1);
		if(y < h-1 && ((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
			&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
			(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh) ))
		{
			seedPt=Point(x1, y+1);
			floodFillScanline(seedPt, setColor, seedColor, ucLow, ucHigh, w, h);
		}
		x1++;
		dst3b = dst.at<Vec3b>(y+1, x1);
	}

	x1 = x - 1;
	dst3b = dst.at<Vec3b>(y, x1);
	while(x1 >= 0 &&  dst3b == setVal)
	{
		src3b = src.at<Vec3b>(y+1,x1);
		if(y < h-1 && ((int)src3b[0] <= ((int)seedVal[0] + (int)ucHigh) && (int)src3b[0] >= ((int)seedVal[0] - (int)ucHigh) 
			&& (int)src3b[1] <= ((int)seedVal[1] + (int)ucHigh) && (int)src3b[1] >= ((int)seedVal[1] - (int)ucHigh) &&
			(int)src3b[2] <= ((int)seedVal[2] + (int)ucHigh) && (int)src3b[2] >= ((int)seedVal[2] - (int)ucHigh)))
		{
			seedPt=Point(x1, y+1);
			floodFillScanline(seedPt, setColor, seedColor, ucLow, ucHigh, w, h);
		}
		x1--;
		dst3b = dst.at<Vec3b>(y+1, x1);
	}
}
