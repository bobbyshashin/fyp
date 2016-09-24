#include "Marker.h"

const int Marker::m_idVerify[4][5]={
	{1,0,0,0,0},
	{1,0,1,1,1},
	{0,1,0,0,1},
	{0,1,1,1,0}
};

Marker::Marker()
{
	m_id=-1;
	//m_rotation.eye();
	//m_translation.zeros();
}

Marker::~Marker()
{
}

Mat Marker::rotate(Mat input)
{
	Mat output;
	input.copyTo(output);
	for(int i=0; i<input.rows; i++)
	{
		for(int j=0; j<input.cols; j++)
		{
			output.at<uchar>(i,j)=input.at<uchar>(input.cols-j-1,i);
		}
	}
	return output;
}

int Marker::hammDist(Mat code)
{
	//每一行和4个hamm中的每一个比较，只要有完全匹配就minSun=0
	//如果每一行都能和4个中的匹配，则返回dist=0
	int dist=0;
	for (int i=0; i<5 ;i++)
	{
		int minSum=10000;
		for(int p=0; p<4 ;p++)
		{
			int sum=0;
			for (int j=0; j<5; j++)
			{
				sum+=(code.at<uchar>(i,j)==m_idVerify[p][j])?0:1;
			}
			if(minSum>sum) minSum=sum;
		}
		dist+= minSum;
	}
	return dist;
}

int Marker::decode(Mat input,int& _nRotation)
{
	assert(input.rows==input.cols);
	assert(input.type()==CV_8UC1);

	Mat grey;
	threshold(input, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//imshow("marker::decode",grey);
	//7*7 regions is black
	int cellSize=input.rows/7;
	for (int i=0; i<7; i++)
	{
		int step = 6;
		//第1和7行有7个，其他2个
		if (i==0 || i==6) 
			step = 1;
		for (int j=0; j<7; j+=step)
		{
			int x = j * cellSize;
			int y = i * cellSize;
			Mat patch = grey(Rect(x, y, cellSize, cellSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (cellSize*cellSize/2) )
			{
				return -1;	// 周围一圈不是全黑不是marker
			}
		}
	}

	//中心5*5区域decode
	//得到5*5bit点阵
	Mat code55 = Mat::zeros(5, 5, CV_8UC1);
	for (int i=0; i<5; i++)
	{
		for (int j=0; j<5; j++)
		{
			int x = (j+1) * cellSize;
			int y = (i+1) * cellSize;
			Mat patch = grey(Rect(x, y, cellSize, cellSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (cellSize*cellSize/2) )
			{
				code55.at<uchar>(i,j) = 1;
			}
		}
	}
	//4个旋转模式都解码
	int minDist = 10000;
	int rotationIndex = -1;
	Mat code55Rotate[4];
	code55Rotate[0] = code55;
	for (int i=0; i<4; i++)	// 每次旋转90°，所以旋转4次
	{
		int distTemp = hammDist(code55Rotate[i]);
		if (distTemp < minDist)
		{
			minDist = distTemp;
			rotationIndex = i;
		}
		code55Rotate[(i+1)%4] = rotate(code55Rotate[i]);
	}

	_nRotation = rotationIndex;
	if (minDist == 0)	// 0完全匹配,返回id
	{
		//第2和4位为编码id
		int val = 0;
		for (int i=0; i<5; i++)
		{
			val <<= 1;//移位1
			if (code55Rotate[rotationIndex].at<uchar>(i,1))
				val |= 1;
			val <<= 1;
			if (code55Rotate[rotationIndex].at<uchar>(i,3))
			{
				val |= 1;
			}
		}
		return val;
	}
	return -1;
}

float Marker::perimeter()
{
	float perimeter=0,dx,dy;
	for (int i=0; i<m_points.size(); i++)
	{
		dx=m_points[i].x-m_points[(i+1)%m_points.size()].x;
		dy=m_points[i].y-m_points[(i+1)%m_points.size()].y;
		perimeter+=sqrt(dx*dx+dy*dy);	
	}
	return perimeter;
}