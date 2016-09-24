/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	Marker.h
* Brief: ±Í«©¿‡,Marker class 
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/2/15 22:00
* History:
************************************************************************/
#pragma once
#include <opencv2/opencv.hpp>
#include "Marker.h"

using namespace cv;

class Marker
{
public:
	//marker id
	int m_id;
	//marker 4points
	vector<Point2f> m_points;
	Matx33f m_rotation;
	Vec3f m_translation;

	Marker();
	~Marker();
	static int decode(Mat input, int& _nRotation);
	float perimeter();
private:
	static const int m_idVerify[4][5];
	//rotate 90 degree
	static Mat rotate(Mat input);
	//hammcode
	static int hammDist(Mat code);
};

