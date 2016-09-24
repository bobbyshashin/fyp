#include "MarkerDetector.h"

#define THRESHOLD 40
//测试显示处理过程
void showCont(String a,Mat frame,vector<vector<Point> > allContours)
{
	Mat frame2=frame.clone();
	for(int i=0;i<allContours.size(); i++)
	{
		drawContours(frame2,allContours,i,Scalar(255,0,0),2);
	}
	imshow(a,frame2);
}

void showCont2(String a,Mat frame,vector<Marker> mark)
{
	Mat frame2=frame.clone();
	for(size_t i=0;i<mark.size(); i++)
	{
		for (size_t j=0; j<4; j++)
			{
				line(frame2,mark[i].m_points[j],mark[i].m_points[(j+1)%4],Scalar(255,0,0),2);
			}
	}
	imshow(a,frame2);
}

MarkerDetector::MarkerDetector()
{
	m_minContourLengthAllowed = 100.0f;
	// 摄像头标定数据
	camMatrix=(Mat_<float>(3,3)<<
		180.2346315,0,159.5,
		0,180.2346315,119.5,
		0,0,1);
	distCoeff = (Mat_<float>(5,1) << -0.276089956,0.1087447314,0.0,0.0,-0.024036442);

	m_markerSize = Size(100, 100);
	// 默认marker的size为100*100,markercorner在2d空间为100*100的矩形
	m_markerCorners2d.push_back(Point2f(0, 0));
	m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, 0));
	m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
	m_markerCorners2d.push_back(Point2f(0, m_markerSize.height-1));

	// 3d corner所在坐标为以marker中心为原点
	m_markerCorners3d.push_back(cv::Point3f(-95.25f,-95.25f,0));
	m_markerCorners3d.push_back(cv::Point3f(+95.25f,-95.25f,0));
	m_markerCorners3d.push_back(cv::Point3f(+95.25f,+95.25f,0));
	m_markerCorners3d.push_back(cv::Point3f(-95.25f,+95.25f,0));
}

MarkerDetector::~MarkerDetector()
{
}

void MarkerDetector::processFrame(Mat frame)
{
	m_markers.clear();
	findMarkers(frame, m_markers);
}


bool MarkerDetector::findMarkers(Mat frame, vector<Marker>& _detectedMarkers)
{
	//to grey
	cvtColor(frame, m_imgGray, CV_BGR2GRAY);

	//Binarization
	//threshold(m_imgGray, m_imgThreshold, THRESHOLD, 255, cv::THRESH_BINARY_INV);
	adaptiveThreshold ( m_imgGray,m_imgThreshold,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,7,7 );
	
	imshow("threshold", m_imgThreshold);

	//counter detect
	vector<vector<Point> > allContours;
	// 把m_imgThreshold中的轮廓点放入allContours
	findContours(m_imgThreshold, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	m_contours.clear();
	for (size_t i=0; i<allContours.size(); i++)
	{
		int contourSize = allContours[i].size();
		if (contourSize > m_imgGray.cols/5)
		{
			m_contours.push_back(allContours[i]);
		}
	}
	//显示处理过程
	//showCont("allContours",frame,allContours);



	// 筛选contours，4点围成的四边形contour为候选marker
	vector<Point> approxCurve; //可能的
	vector<Marker> markerPossible;
	//筛选最小边长大于100的凸四边形
	for (size_t i=0; i<m_contours.size(); i++)
	{
		// 得到近似多边形
		approxPolyDP(m_contours[i], approxCurve, double(m_contours[i].size())*0.05, true);
		// 我们只关心那些4边形的轮廓，因为只有他们才可能是marker
		if (approxCurve.size() != 4)
			continue;
		// 并且是凸4边形
		if (!isContourConvex(approxCurve))
			continue;
		// 找个4个边最小的边
		float minDist = FLT_MAX;
		for (int i=0; i<4; i++)
		{
			Point vecTemp = approxCurve[i] - approxCurve[(i+1)%4];
			float distSquared = vecTemp.dot(vecTemp);
			minDist = std::min(minDist, distSquared);
		}
		// 如果这个最小边不够长，这个contour不是marker
		if (minDist > m_minContourLengthAllowed)//100.f
		{
			Marker markerTemp;
			for (int i=0; i<4; i++)
			{
				markerTemp.m_points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
			}
			markerPossible.push_back(markerTemp);
		}
	}

	//显示处理过程
	showCont2("markerPossible",frame,markerPossible);



	// 把得到的markerPossible按照逆时针排序
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		// 检测第三个点在一二点组成的线的左边还是右边
		Point v1 = markerPossible[i].m_points[1] - markerPossible[i].m_points[0];
		Point v2 = markerPossible[i].m_points[2] - markerPossible[i].m_points[0];
		double theta = (v1.x * v2.y) - (v1.y * v2.x);
		// 左边为逆时针，右边顺时针，变换2、4点顺序
		if (theta < 0.0)
		{
			std::swap(markerPossible[i].m_points[1], markerPossible[i].m_points[3]);
		}
	}
	// 找到corner非常接近的contour,放到tooNearCandidates
	vector<std::pair<int, int> > tooNearCandidates;
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		for (size_t j=i+1; j<markerPossible.size(); j++)
		{
			float distSquared = 0.0f;
			for (int k=0; k<4; k++)
			{
				Point vec = markerPossible[i].m_points[k] - markerPossible[j].m_points[k];
				distSquared += vec.dot(vec);
			}
			if (distSquared < 400)
			{
				tooNearCandidates.push_back(std::pair<int, int>(i,j));
			}
		}
	}
	// 选择tooNearCadidates中周长更小的作为移除的对象
	vector<bool> markerRemoveIndex(markerPossible.size(), false);
	for (size_t i=0; i<tooNearCandidates.size(); i++)
	{
		float length1 = markerPossible[tooNearCandidates[i].first].perimeter();
		float length2 = markerPossible[tooNearCandidates[i].second].perimeter();
		markerRemoveIndex[(length1>length2) ? tooNearCandidates[i].second : tooNearCandidates[i].first] = true;
	}
	// 去掉markerRemoveIndex得到最终的候选者
	_detectedMarkers.clear();
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		if (!markerRemoveIndex[i])
		{
			_detectedMarkers.push_back(markerPossible[i]);
		}
	}

	detectMarkers(m_imgGray, _detectedMarkers);

	return true;
}

void MarkerDetector::detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers)
{
	Mat canonicalImg;	// 去除投影变换恢复的3维视图
	vector<Marker> goodMarkers;

	// 根据id，验证marker
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		// 得到当前marker的透视变换矩阵M
		Mat M = getPerspectiveTransform(_detectedMarkers[i].m_points, m_markerCorners2d);
		// 将当前的marker变换为正交投影
		warpPerspective(_imgGray, canonicalImg, M, m_markerSize);
// 		imshow(str, canonicalImg);
		// 解码marker
		int nRotations;
		int idMarker = Marker::decode(canonicalImg, nRotations);
		if (idMarker != -1)
		{
			_detectedMarkers[i].m_id = idMarker;
			// 将对应的corner point也进行旋转
			std::rotate(_detectedMarkers[i].m_points.begin(), _detectedMarkers[i].m_points.begin()+4-nRotations, _detectedMarkers[i].m_points.end());
			goodMarkers.push_back(_detectedMarkers[i]);
		}
	}

	// 用subpixel提高marker corner的精度
	if (goodMarkers.size() > 0)
	{
		// 把各corner整理为一个数组，因为cornerSubPix函数很慢，尽力不要多次调用
		vector<Point2f> preciseCorners(4*goodMarkers.size());

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				preciseCorners[4*i+j] = goodMarkers[i].m_points[j];
			}
		}

		cornerSubPix(_imgGray, preciseCorners, Size(5,5), Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				goodMarkers[i].m_points[j] = preciseCorners[4*i+j];
			}
		}
	}

	_detectedMarkers = goodMarkers;
}

void MarkerDetector::estimatePosition(vector<Marker>& _detectedMarkers)
{
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		
		Mat Rvec;
// 		Mat_<float> Tvec;
		Vec3f Tvec;
		// 建立一个3d to 2d的映射
		solvePnP(m_markerCorners3d, _detectedMarkers[i].m_points, camMatrix, distCoeff, raux, taux);
		raux.convertTo(Rvec, CV_32F);
		taux.convertTo(Tvec, CV_32F);



// 		Mat_<float> rotMat(3,3);
		Matx33f rotMat;
		Rodrigues(Rvec, rotMat);

		_detectedMarkers[i].m_rotation = rotMat.t();
		_detectedMarkers[i].m_translation = -Tvec;

		//std::cout<<"平移矢量: "<<_detectedMarkers[i].m_translation<<std::endl;
		//std::cout<<"旋转矩阵: "<<std::endl<<_detectedMarkers[i].m_rotation<<std::endl;
		//std::cout<<"距离：    "<<sqrt(Tvec[0]*Tvec[0]+Tvec[1]*Tvec[1]+Tvec[2]*Tvec[2])<<std::endl;

	}
}

void MarkerDetector::drawMarkers(Mat& _frame)
{
	for(size_t i=0;i<m_markers.size(); i++)
	{
		for(size_t j=0;j<m_markers[i].m_points.size();j++)
		line(_frame,m_markers[i].m_points[j],m_markers[i].m_points[(j+1)%4],Scalar(0,0,255),2);
		circle(_frame,m_markers[i].m_points[0],3,Scalar(0,255,255),4,8);
		circle(_frame,m_markers[i].m_points[1],3,Scalar(255,0,255),4,8);
		circle(_frame,m_markers[i].m_points[2],3,Scalar(255,100,100),4,8);
		circle(_frame,m_markers[i].m_points[3],3,Scalar(100,100,255),4,8);

		Point centre;
		centre.x=(m_markers[i].m_points[0].x+m_markers[i].m_points[2].x)/2;
		centre.y=(m_markers[i].m_points[0].y+m_markers[i].m_points[2].y)/2;
		line(_frame,centre,Point(320/2,240/2),Scalar(255,255,255),1);
	}
	
}


void MarkerDetector::drawCube(Mat& _frame)
{
	if(m_markers.size()>0)
	{
	estimatePosition(m_markers);
	vector<Point3f> cube3D;
	vector<Point2f> cube2D;
	cube3D.clear();
	cube2D.clear();
	cube3D.push_back(Point3f(-0.5f,-0.5f,0));
	cube3D.push_back(Point3f(+0.5f,-0.5f,0));
	cube3D.push_back(Point3f(+0.5f,+0.5f,0));
	cube3D.push_back(Point3f(-0.5f,+0.5f,0));
	cube3D.push_back(Point3f(-0.5f,-0.5f,0.25));
	cube3D.push_back(Point3f(+0.5f,-0.5f,0.25));
	cube3D.push_back(Point3f(+0.5f,+0.5f,0.25));
	cube3D.push_back(Point3f(-0.5f,+0.5f,0.25));
	cube3D.push_back(Point3f(0.f,0.f,0.f));
	
	Mat Tvec=(Mat_<float>(3,1)<<0,0,-4);
	Mat Rvec;
	raux.convertTo(Rvec, CV_32F);
	projectPoints(cube3D,Rvec,Tvec,camMatrix,distCoeff,cube2D);

	line(_frame,cube2D[0],cube2D[1],Scalar(255,255,0),1);
	line(_frame,cube2D[1],cube2D[2],Scalar(255,255,0),1);
	line(_frame,cube2D[2],cube2D[3],Scalar(255,255,0),1);
	line(_frame,cube2D[3],cube2D[0],Scalar(255,255,0),1);
	line(_frame,cube2D[0],cube2D[2],Scalar(255,255,0),1);
	line(_frame,cube2D[1],cube2D[3],Scalar(255,255,0),1);
	//line(_frame,cube2D[4],cube2D[5],Scalar(255,255,0),2);
	//line(_frame,cube2D[5],cube2D[6],Scalar(255,255,0),2);
	//line(_frame,cube2D[6],cube2D[7],Scalar(255,255,0),2);
	//line(_frame,cube2D[7],cube2D[4],Scalar(255,255,0),2);
	line(_frame,cube2D[0],cube2D[4],Scalar(255,255,0),1);
	line(_frame,cube2D[1],cube2D[5],Scalar(255,255,0),1);
	line(_frame,cube2D[2],cube2D[6],Scalar(255,255,0),1);
	line(_frame,cube2D[3],cube2D[7],Scalar(255,255,0),1);

	circle(_frame,cube2D[4],3,Scalar(0,255,255  ),1,8);
	circle(_frame,cube2D[5],3,Scalar(255,0,255  ),1,8);
	circle(_frame,cube2D[6],3,Scalar(255,100,100),1,8);
	circle(_frame,cube2D[7],3,Scalar(100,100,255),1,8);


	//for(size_t j=0;j<4;j++)
		//line(_frame,cube2D[j],cube2D[(j+1)%4],Scalar(0,0,255),2);

	//飞机
	//ellipse( _frame, cube2D[0],Size( 20,20 ),90,0,180,Scalar( 255, 0, 0 ),2);


	

	}



}
