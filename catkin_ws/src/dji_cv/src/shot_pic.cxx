#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "aruco.h"
#include "math.h"
#include "cvdrawingutils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#define CAMERA_NUM 1

using namespace cv;
using namespace std;
using namespace aruco;



int width = 640;
int height = 480;
double exposure = 0.8;
double brightness = 0.5;
double contrast = 0.6;
double saturation = 0.2;
double hue = 0.5;
double gain = 0.5;
vector< Marker > TheMarkers;
CameraParameters TheCameraParameters;
MarkerDetector MDetector;
float TheMarkerSize = 0.15;
string TheIntrinsicFile="usbcam.yml";
Point2f cen;



int main(int argc,char** argv)
{
    int count=1;
    char num_char[3];

    string num_str;
    Mat read_view,view,img_aruco;
    int CAMERA_NUM=0;
    if(argc>1)
    {
        CAMERA_NUM = atof(argv[1]);
    }
    else
    {
        cout<<"usage: --\"camera index\" --\"The IntrinsicFile\" --\"enable IR receive from ROS(true/false) \" "<<endl;
        return 0;
    }


    VideoCapture cap(CAMERA_NUM);


    cout<<"frame_width  "<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<<endl;
    cout<<"-----------------"<<endl;
    cout<<"frame_height  "<<cap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;


    //cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
    //cap.set(CV_CAP_PROP_FRAME_WIDTH ,width);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT ,height);
    cap.set(CV_CAP_PROP_EXPOSURE ,exposure);
    cap.set(CV_CAP_PROP_BRIGHTNESS ,brightness);
    cap.set(CV_CAP_PROP_CONTRAST,contrast);
    cap.set(CV_CAP_PROP_SATURATION,saturation);
    cap.set(CV_CAP_PROP_HUE,hue);
    cap.set(CV_CAP_PROP_GAIN,gain);
    //cap.set(CV_CAP_PROP_FPS, 25);

    char c;

    Mat CM = Mat(3, 3, CV_32FC1);
    Mat D;
    Mat viewUndistort;

    FileStorage fs2(TheIntrinsicFile,FileStorage::READ);
    fs2["camera_matrix"]>>CM;
    fs2["distortion_coefficients"]>>D;
    fs2.release();
    Mat RM,TM;
    //Mat RT(3,1);
    Mat pos_tmp;
    Mat _RM,_TM;
    Mat map1,map2;
    cap>>view;
    img_aruco=view.clone();
    // read camera parameters if passed
    TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
    TheCameraParameters.resize(img_aruco.size());

    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);

    initUndistortRectifyMap(CM,D,Mat(),Mat(),img_aruco.size(),CV_32FC1,map1,map2);
    namedWindow("Aruco", CV_WINDOW_NORMAL);

    while(c!='c')
    {
    c=waitKey(5);
    cap>>view;

    //img_aruco = view.clone();

    //remap(view,viewUndistort,map1,map2,INTER_LINEAR);
    //undistort(view,viewUndistort,CM,D);
    //img_aruco=viewUndistort.clone();
    img_aruco=view.clone();
    // Detection of markers in the image passed
    MDetector.detect(img_aruco, TheMarkers, TheCameraParameters, TheMarkerSize);

    //cout<<"buffer: "<<buffer<<endl;


    for (unsigned int i = 0; i < TheMarkers.size(); i++)
    {
        //cout << endl << TheMarkers[i];
        cen = TheMarkers[i].getCenter();
        cout<<"cen["<<i<<"] "<<cen<<endl;
        RM=TheMarkers[i].Rvec;
        TM=TheMarkers[i].Tvec;
        cout<<"Rvec "<<RM<<endl;
        cout<<"Tvec "<<TM<<endl;
        float dis = TM.at<float>(0,0)*TM.at<float>(0,0)+TM.at<float>(1,0)*TM.at<float>(1,0)+TM.at<float>(2,0)*TM.at<float>(2,0);
        Rodrigues(RM,_RM);
        pos_tmp = _RM*TM;
        cout <<"POS_X "<<pos_tmp.at<float>(0,0)<<endl;
        cout <<"POS_Y "<<pos_tmp.at<float>(0,1)<<endl;
        cout <<"POS_Z "<<pos_tmp.at<float>(0,2)<<endl;
        cout<<"Rotation Matrix: "<< _RM<<endl;
        cout<<"distance  "<<sqrt(dis)<<endl;
        TheMarkers[i].draw(img_aruco, Scalar(0, 0, 255), 1);
    }
    if (TheMarkers.size() != 0)
        cout << endl;

    // draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i = 0; i < TheMarkers.size(); i++)
        {
            CvDrawingUtils::draw3dCube(img_aruco, TheMarkers[i], TheCameraParameters);
            CvDrawingUtils::draw3dAxis(img_aruco, TheMarkers[i], TheCameraParameters);
        }
    imshow("view",view);
    imshow("Aruco",img_aruco);
    //imshow("thres", MDetector.getThresholdedImage());
    //imshow("Video",view);
    //imshow("Video_undistort",viewUndistort);
    //setWindowProperty("Aruco", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    if(c=='s')
        {
        sprintf(num_char,"%d",count);
        num_str=num_char;
        string Name_string="/home/chens/ghost27/ar/img/shot_"+num_str;

        Name_string= Name_string + ".jpg";
        imwrite(Name_string, view);
        namedWindow("Shot View");
        read_view=imread(Name_string);
        imshow("Shot_View",read_view);
        count++;
        }
    }

    return 0;
}




