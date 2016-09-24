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
	//ÿһ�к�4��hamm�е�ÿһ���Ƚϣ�ֻҪ����ȫƥ���minSun=0
	//���ÿһ�ж��ܺ�4���е�ƥ�䣬�򷵻�dist=0
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
		//��1��7����7��������2��
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
				return -1;	// ��ΧһȦ����ȫ�ڲ���marker
			}
		}
	}

	//����5*5����decode
	//�õ�5*5bit����
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
	//4����תģʽ������
	int minDist = 10000;
	int rotationIndex = -1;
	Mat code55Rotate[4];
	code55Rotate[0] = code55;
	for (int i=0; i<4; i++)	// ÿ����ת90�㣬������ת4��
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
	if (minDist == 0)	// 0��ȫƥ��,����id
	{
		//��2��4λΪ����id
		int val = 0;
		for (int i=0; i<5; i++)
		{
			val <<= 1;//��λ1
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