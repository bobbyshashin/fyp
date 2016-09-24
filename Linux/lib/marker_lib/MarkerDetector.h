/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	MarkerDetector.h
* Brief: ±Í«©ÃΩ≤‚¿‡,MarkerDetector class 
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

class MarkerDetector
{
public:
	Mat m_imgGray;
	Mat m_imgThreshold;
	Mat raux, taux;
	vector<vector<Point> > m_contours;

	vector<Marker> m_markers;

	MarkerDetector();
	~MarkerDetector();
	void processFrame(Mat frame);
	void drawMarkers(Mat& _frame);
	void drawCube(Mat& _frame);
private:
	float m_minContourLengthAllowed;	
	Size m_markerSize;
	vector<Point2f> m_markerCorners2d;	
	vector<Point3f> m_markerCorners3d;	
	Mat camMatrix;
	Mat distCoeff;

	bool findMarkers(Mat frame, vector<Marker>& _detectedMarkers);
	void detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers);
	void estimatePosition(vector<Marker>& _detectedMarkers);

};
