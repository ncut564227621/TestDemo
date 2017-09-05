#include"stdafx.h"
#include "harris.h"

//ºÏ≤‚Ω«µ„
void harris::detect(Mat img)
{
	cornerHarris(img, cornerStrength, nNeighbourhood, nAperture, k); //R= det(M)-k*trace(M)
	double minStrength;
	minMaxLoc(cornerStrength, &minStrength, &maxStrength);

	Mat dilated;
	dilate(cornerStrength, dilated, cv::Mat());

	compare(cornerStrength, dilated, localMax, CMP_EQ);
}

Mat harris::getCornerMap(double qualityLevel)
{
	Mat cornerMap;
	threshold = qualityLevel*maxStrength;

	cv::threshold(cornerStrength, cornerThd, 
		threshold, 255, THRESH_BINARY);

	cornerThd.convertTo(cornerMap, CV_8U);

	bitwise_and(cornerMap, localMax, cornerMap);

	return cornerMap;
}

void harris::getCorners(vector<Point>&pts, Mat&cornerMap)
{
	for(int yy =0; yy<cornerMap.rows; yy++)
	{
		uchar * ptr = cornerMap.ptr<uchar>(yy);
		for(int xx =0; xx<cornerMap.cols; xx++)
		{
			if(ptr[xx]!=0)
			{
	            pts.push_back(Point(xx,yy));
			}
		}
	}
}

void harris::corners(vector<Point>&pts, double qualityLevel)
{
	Mat cornerMap = getCornerMap(qualityLevel);
	getCorners(pts, cornerMap);
}

void harris::shi_TomasHarrisDetect(Mat img,vector<Point>&pts, const int maxCornerNum, 
								   const int minDistance, double qualityLevel)
{
	goodFeaturesToTrack(img, pts, 100,0.1,10,noArray(),3,false,0.01);
}
void harris::fastHarris(Mat img, vector<Point>&pts, int threshold, bool bSuppression)
{
	vector<KeyPoint> keyPts;
	FAST(img, keyPts, threshold, true);
	for(int i =0; i<keyPts.size(); i++)
	{
		pts.push_back((Point)(keyPts[i].pt));
	}
}



void harris::drawCorners(const vector<Point>& pts, Mat src, Scalar color, int radius, int thickness)
{
	vector<Point>::const_iterator iter = pts.begin();
	for(; iter!=pts.end(); iter++)
	{
		//circle(src, *iter, radius,  color, thickness);
		drawCross(src, *iter, color);
		
	}

	imwrite("..\\Sample\\Result\\fast-corner.bmp", src);
}

void drawCross(Mat img, Point pt, Scalar color, int size, int thickness)
{
	if(pt.x-size/2<0||pt.x+size/2>=img.cols||pt.y-size/2<0||pt.y+size/2>=img.rows)
		return;
	line(img, Point(pt.x-size/2,pt.y), Point(pt.x+size/2, pt.y), color);//ª≠∫·œﬂ
	line(img, Point(pt.x, pt.y-size/2), Point(pt.x, pt.y+size/2),color);//ª≠ ˙œﬂ
}




