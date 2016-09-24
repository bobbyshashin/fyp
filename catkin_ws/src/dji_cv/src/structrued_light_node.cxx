//include ros library
#include "ros/ros.h"

//include std/string libraries
#include<sstream>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>

//include messege libraries
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
//#include "Matrix3x3.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//include aruco libraries
#include "aruco.h"
#include "boarddetector.h"
#include "cvdrawingutils.h"

//include eigen library
#include "Dense"

using namespace Eigen;
using namespace cv;
using namespace std;
using namespace aruco;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "structured_light");
    ros::NodeHandle n;


    ros::Publisher pos_pub = n.advertise<geometry_msgs::Vector3>("board_pos",20);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("board_pose",20);

    ros::Publisher rot_pub = n.advertise<geometry_msgs::Vector3>("board_rot",20);

/*
 * -0.84439421;
  -0.64554375;
  2.7416008]
TheBoard1[-0.078701362;
  -0.5612489;
  2.3771942]
TheBoard2[0.66041666;
  -0.60780728;
  2.5090501]
TheBoard3[-0.74187148;
  -0.099132359;
  2.4843953]
TheBoard4[-0.086619943;
  -0.095523894;
  2.7411728]
TheBoard5[0.58632576;
  -0.071167566;
  2.2965338]
TheBoard6[-0.84689581;
  0.421929;
  2.870157]
TheBoard7[-0.07848113;
  0.3858512;
  2.4599414]
TheBoard8[0.66264492;
  0.44806364;
  2.644855]
 * */

     MatrixXd M, X, D, M_T, M_T_M, M_T_M_I, M_T_M_I_MT;
     M.resize(9,3);
     M_T.resize(3,9);
     M_T_M.resize(3,3);
     M_T_M_I.resize(3,3);
     M_T_M_I_MT.resize(3,9);
     X.resize(3,1);
     D.resize(9,1);
     M << -0.84439421, -0.64554375, 1,      //board 0
             -0.078701362, -0.5612489, 1,   //board 1
             0.66041666, -0.60780728, 1,    //board 2
             -0.74187148, -0.099132359, 1,  //board 3
             -0.086619943, -0.095523894, 1, //board 4
             0.58632576, -0.071167566, 1,   //board 5
             -0.84689581, 0.421929, 1,      //board 6
             -0.07848113, 0.3858512, 1,     //board 7
             0.66264492, 0.44806364, 1;     //board 8


     cout << "M =" << endl << M << endl;


     D << 2.7416008, 2.3771942, 2.5090501,
          2.4843953, 2.7411728, 2.2965338,
          2.870157, 2.4599414, 2.644855;
     cout << "D =" << endl << D << endl;




    /******************var new***************************************************/
    int CAMERA_NUM;

    Mat img,gray_img,blur_img,corner_img,nor_img,scal_img;

    Mat img_point(6,2,CV_8UC1);
    Mat obj_point(6,3,CV_8UC1);
    Mat rvec,tvec;
    tf::TransformBroadcaster bc;
    tf::Transform transform;
    Mat CM,DC;  //Camera Matrix

    vector< Marker > Markers;
    CameraParameters TheCameraParameters;
    MarkerDetector MDetector;
    float TheMarkerSize = 0.15;
    Point2f cen;

    std_msgs::String str_msg;
    geometry_msgs::Vector3 pos_msg;
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::Vector3 ang_msg;


    BoardConfiguration TheBoardConfig[10];


    BoardDetector TheBoardDetector;
    Board TheBoardDetected[10];



    Mat map1,map2;
    Mat Rvec[10],Tvec[10];

    Mat RM[10],TM[10];
//**************para read*****************************************************/
    cout<<"here is all begin"<<endl;

    string TheIntrinsicFile;
    string TheBoardConfigFile;
    if(argc>1)
    {

        CAMERA_NUM = atof(argv[1]);
        if(argc>2)
        {
            TheIntrinsicFile = argv[2];
            if(argc>3)
            {
               TheBoardConfigFile = argv[3];
               if(argc>4)
               {
                TheMarkerSize = atof(argv[4]);
               }
            }
        }
        else
        {
            return 0;
        }

    }
    else
    {
        cout<<"usage: --\"camera index\" --\"intrinsic file\" --\"board config\" "<<endl;
        return 0;
    }

    VideoCapture cap(CAMERA_NUM);
    string  boardcfg[10];


    FileStorage fs1(TheBoardConfigFile,FileStorage::READ) ;

    fs1["board_config_file_1"] >> boardcfg[0] ;
    fs1["board_config_file_2"] >> boardcfg[1] ;
    fs1["board_config_file_3"] >> boardcfg[2] ;
    fs1["board_config_file_4"] >> boardcfg[3] ;
    fs1["board_config_file_5"] >> boardcfg[4] ;
    fs1["board_config_file_6"] >> boardcfg[5] ;
    fs1["board_config_file_7"] >> boardcfg[6] ;
    fs1["board_config_file_8"] >> boardcfg[7] ;
    fs1["board_config_file_9"] >> boardcfg[8] ;

/**************************************************/
    for(int i=0;i<9;i++) cout<<boardcfg[i] << endl;

/**************************************************/
    fs1.release();

    for(int i=0;i<9;i++) TheBoardConfig[i].readFromFile ( boardcfg[i] ) ;

    string calibration_date_str;
    FileStorage fs2(TheIntrinsicFile,FileStorage::READ);
    fs2["camera_matrix"]>>CM;
    fs2["distortion_coefficients"]>>DC;
    fs2["calibration_time"]>>calibration_date_str;
    cout<<calibration_date_str<<endl;
    fs2.release();
    cout<<CM<<endl<<DC<<endl;

    cap>>img;
    TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
    TheCameraParameters.resize(img.size());

    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);
    initUndistortRectifyMap(CM,DC,Mat(),Mat(),img.size(),CV_32FC1,map1,map2);

    while(n.ok() && waitKey(30) != 'q')
    {

        cap >> img;
        //GaussianBlur(img,blur_img,Size(5,5),0,0);
        cvtColor(img,gray_img,CV_RGB2GRAY);
/**************************************************************************************/
/***********************PnP Sovle******************************************************/
        /*
        cornerHarris(gray_img,corner_img,2,3,0.04);
        normalize(corner_img , nor_img , 0,255 , NORM_MINMAX , CV_32FC1 , Mat() );
        convertScaleAbs(nor_img,scal_img);
        for(int i=0 ; i < nor_img.rows ; i++)
        {
            for(int j=0 ; j < nor_img.cols ; j++)
            {
                if( (int)nor_img.at<float>(i,j) > 150 )
                {
                    circle(gray_img,Point(j,i),5,Scalar(10,10,255),1,8,0);
                }

            }

        }
        */
        //solvePnP(obj_point , img_point , CM , DC , rvec , tvec );
/**************************************************************************************/



/*******************************singal maker********************************************/
//        double t = (double)cvGetTickCount();

        // Detection of the marker
        MDetector.detect(gray_img, Markers, TheCameraParameters, TheMarkerSize);

        // Detection of the board

          float probDetect[10];
          bool is_calc_reliable;
          is_calc_reliable = true;
          for(int i=0;i<9;i++)
          {
            probDetect[i] = TheBoardDetector.detect(Markers, TheBoardConfig[i], TheBoardDetected[i], TheCameraParameters, TheMarkerSize);
            if((probDetect[i]) == 0) { is_calc_reliable = false;}
            cout<<"prob ["<<i<<"] = "<<probDetect[i]<<endl;
          }
//          t = (double)cvGetTickCount() - t;
//              cout << "eigen "<<" time: " << t / ((double)cvGetTickFrequency()*1000.) <<"ms"<< endl;

        for (unsigned int i = 0; i < Markers.size(); i++) {
            //cout << Markers[i] << endl;
            Markers[i].draw(img, Scalar(0, 0, 255), 2);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (TheCameraParameters.isValid() && TheMarkerSize != -1)
        {
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                CvDrawingUtils::draw3dCube(img, Markers[i], TheCameraParameters);
                CvDrawingUtils::draw3dAxis(img, Markers[i], TheCameraParameters);
            }

//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_1, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_2, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_3, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_4, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_5, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_6, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_7, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_8, TheCameraParameters);
//            CvDrawingUtils::draw3dAxis(img, TheBoardDetected_9, TheCameraParameters);

              for(int i=0;i<9;i++)
                  cout << "Board " << i << endl <<"Rvec "<<TheBoardDetected[i].Rvec << endl << "Tvec" << TheBoardDetected[i].Tvec << endl;
                  //cout << "TheBoard"<<i<<TheBoardDetected[i].Tvec << endl;


              for(int i=0;i<9;i++)
              {
                  RM[i]=TheBoardDetected[i].Rvec;
                  TM[i]=TheBoardDetected[i].Tvec;
              }


              float pos_x,pos_y,pos_z;
              float ang_x,ang_y,ang_z;
              float prob=0;
              for(int i=0;i<9;i++)
              {
                  prob=prob+probDetect[i];
              }

//              pos_x = (0.25 * ( TM[0].at<float>(0,0) + TM[8].at<float>(0,0) ) * (probDetect[0]+probDetect[8]) +
//                      0.25 * ( TM[1].at<float>(0,0) + TM[7].at<float>(0,0) ) * (probDetect[1]+probDetect[7]) +
//                      0.25 * ( TM[2].at<float>(0,0) + TM[6].at<float>(0,0) ) * (probDetect[2]+probDetect[6]) +
//                      0.25 * ( TM[3].at<float>(0,0) + TM[5].at<float>(0,0) ) * (probDetect[3]+probDetect[5]) +
//                              TM[4].at<float>(0,0) * probDetect[4])/prob;
//                pos_x = (TM[0].at<float>(0,0) + TM[8].at<float>(0,0))*0.5;
              pos_x = TM[6].at<float>(0,0);
              pos_y = TM[6].at<float>(1,0);
              pos_z = TM[6].at<float>(2,0);

              ang_x = RM[6].at<float>(0,0);
              ang_y = RM[6].at<float>(1,0);
              ang_z = RM[6].at<float>(2,0);



              cout<<"POS_X  "<<pos_x<<endl;
              cout<<"POS_Y  "<<pos_y<<endl;
              cout<<"POS_Z  "<<pos_z<<endl;

              pos_msg.x = pos_x;
              pos_msg.y = pos_y;
              pos_msg.z = pos_z;

              ang_msg.x = ang_x;
              ang_msg.y = ang_y;
              ang_msg.z = ang_z;


//              pos_msg.x = TM[0].at<float>(0,0);
//              pos_msg.y = TM[0].at<float>(1,0);
//              pos_msg.z = TM[0].at<float>(2,0);

//              pose_msg.pose.position.x=pos_x;
//              pose_msg.pose.position.y=pos_y;
//              pose_msg.pose.position.z=pos_z;
                if(is_calc_reliable)
                  {
                    double t = (double)cvGetTickCount();
                    M << TM[0].at<float>(0,0), TM[0].at<float>(1,0), 1,      //board 0
                            TM[1].at<float>(0,0), TM[1].at<float>(1,0), 1,   //board 1
                            TM[2].at<float>(0,0), TM[2].at<float>(1,0), 1,    //board 2
                            TM[3].at<float>(0,0), TM[3].at<float>(1,0), 1,  //board 3
                            TM[4].at<float>(0,0), TM[4].at<float>(1,0), 1, //board 4
                            TM[5].at<float>(0,0), TM[5].at<float>(1,0), 1,   //board 5
                            TM[6].at<float>(0,0), TM[6].at<float>(1,0), 1,      //board 6
                            TM[7].at<float>(0,0), TM[7].at<float>(1,0), 1,     //board 7
                            TM[8].at<float>(0,0), TM[8].at<float>(1,0), 1,     //board 8


                    cout << "M =" << endl << M << endl;


                    D << TM[0].at<float>(2,0), TM[1].at<float>(2,0), TM[2].at<float>(2,0),
                         TM[3].at<float>(2,0), TM[4].at<float>(2,0), TM[5].at<float>(2,0),
                         TM[6].at<float>(2,0), TM[7].at<float>(2,0), TM[8].at<float>(2,0);
                    cout << "D =" << endl << D << endl;

                    M_T=M.transpose();
                    M_T_M = M_T * M;
                    M_T_M_I = M_T_M.reverse();
                    M_T_M_I_MT = M_T_M_I * M_T;
                    X = M_T_M_I_MT * D;
                    cout<<"X is reliable"<<endl<< X <<endl;
                    pose_msg.pose.position.x = X(0,0);
                    pose_msg.pose.position.y = X(1,0);
                    pose_msg.pose.position.z = X(2,0);
                    t = (double)cvGetTickCount() - t;
                        cout << "eigen "<<" time: " << t / ((double)cvGetTickFrequency()*1000.) <<"ms"<< endl;

                  } else cout << "X is Unreliable"<< endl;


                pose_msg.header.frame_id="board";
                pos_pub.publish(pos_msg);
                rot_pub.publish(ang_msg);
                pose_pub.publish(pose_msg);

                transform.setOrigin( tf::Vector3(pos_x, pos_y, pos_z) );
                  tf::Quaternion q;
                  q.setRPY(ang_x, ang_y, ang_z);
                  transform.setRotation(q);
                  bc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "board"));

              pos_x = 0;
              pos_y = 0;
              pos_z = 0;
              ang_x = 0;
              ang_y = 0;
              ang_z = 0;




        }
/**************************************************************************************/

        imshow("image",img);
        imshow("gray",gray_img);
        //imshow("corner_img",corner_img);
        ros::spinOnce();
    }


    return 0;
}
