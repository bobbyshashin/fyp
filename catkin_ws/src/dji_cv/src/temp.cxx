// Detection of the marker
MDetector.detect(gray_img, Markers, TheCameraParameters, TheMarkerSize);

// Detection of the board

float probDetect[4]={0,0,0,0};
bool is_calc_reliable;
is_calc_reliable = true;
for(int i=0;i<4;i++)
{
    probDetect[i] = TheBoardDetector.detect(Markers, TheBoardConfig[i], TheBoardDetected[i], TheCameraParameters, TheMarkerSize);
    if((probDetect[i]) == 0) { is_calc_reliable = false;}
    cout<<"prob ["<<i<<"] = "<<probDetect[i]<<endl;
}

board_status_msg.data = board_status;
board_status_pub.publish(board_status_msg);
for (unsigned int i = 0; i < Markers.size(); i++) {
    //cout << Markers[i] << endl;
    //Markers[i].draw(image_callback, Scalar(0, 0, 255), 2);
}

// draw a 3d cube in each marker if there is 3d info
if (TheCameraParameters.isValid() && TheMarkerSize != -1)
{
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        //CvDrawingUtils::draw3dCube(image_callback, Markers[i], TheCameraParameters);
        //CvDrawingUtils::draw3dAxis(image_callback, Markers[i], TheCameraParameters);
    }
    for (int i=0;i<4;i++)
    {
        if(probDetect[i]>0)
            CvDrawingUtils::draw3dAxis(image_callback, TheBoardDetected[i], TheCameraParameters);

    }

    for(int i=0;i<4;i++)
        cout << "Board " << i << endl <<"Rvec "<<TheBoardDetected[i].Rvec << endl << "Tvec" << TheBoardDetected[i].Tvec << endl;
    //scout << "TheBoard"<<i<<TheBoardDetected[i].Tvec << endl;


    for(int i=0;i<4;i++)
    {
        RM[i]=TheBoardDetected[i].Rvec;
        TM[i]=TheBoardDetected[i].Tvec;
    }


    float prob=0;
    for(int i=0;i<4;i++)
    {
        prob=prob+probDetect[i];
    }
    for(int i=0;i<4;i++)
    {
        pos_msg.x = TM[i].at<float>(0,0);
        pos_msg.y = TM[i].at<float>(1,0);
        pos_msg.z = TM[i].at<float>(2,0);

        ang_msg.x = RM[i].at<float>(0,0);
        ang_msg.y = RM[i].at<float>(1,0);
        ang_msg.z = RM[i].at<float>(2,0);

        pose_msg.header.frame_id = "Aruco_Pose";
        pose_msg.header.stamp = ros::Time::now();

        pose_msg.pose.position.x = pos_msg.x;
        pose_msg.pose.position.y = pos_msg.y;
        pose_msg.pose.position.z = pos_msg.z;

        pose_msg.pose.orientation.x = ang_msg.x;
        pose_msg.pose.orientation.y = ang_msg.y;
        pose_msg.pose.orientation.z = ang_msg.z;
        pose_msg.pose.orientation.w = probDetect[i];


        if(i==0) pose_pub_1.publish(pose_msg);
        if(i==1) pose_pub_2.publish(pose_msg);
        if(i==2) pose_pub_3.publish(pose_msg);
        if(i==3) pose_pub_4.publish(pose_msg);

        position_pub.publish(pos_msg);
        rotation_pub.publish(ang_msg);


    }
    frame_count++;

}

vector <Point2f> imagePoint_0,imagePoint_1,imagePoint_2,imagePoint_3;



Mat objectPoint_1(1, 3, CV_32FC1);
Mat objectPoint_2(1, 3, CV_32FC1);
Mat objectPoint_3(1, 3, CV_32FC1);
Mat objectPoint_4(1, 3, CV_32FC1);

objectPoint_1.at< float >(0, 0) = 0;
objectPoint_1.at< float >(0, 1) = 0;
objectPoint_1.at< float >(0, 2) = 0;

objectPoint_2.at< float >(0, 0) = 0;
objectPoint_2.at< float >(0, 1) = 0;
objectPoint_2.at< float >(0, 2) = 0;

objectPoint_3.at< float >(0, 0) = 0;
objectPoint_3.at< float >(0, 1) = 0;
objectPoint_3.at< float >(0, 2) = 0;

objectPoint_4.at< float >(0, 0) = 0;
objectPoint_4.at< float >(0, 1) = 0;
objectPoint_4.at< float >(0, 2) = 0;


projectPoints(objectPoint_1, TheBoardDetected[0].Rvec, TheBoardDetected[0].Tvec,
        TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_0);
projectPoints(objectPoint_2, TheBoardDetected[1].Rvec, TheBoardDetected[1].Tvec,
        TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_1);
projectPoints(objectPoint_3, TheBoardDetected[2].Rvec, TheBoardDetected[2].Tvec,
        TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_2);
projectPoints(objectPoint_4, TheBoardDetected[3].Rvec, TheBoardDetected[3].Tvec,
        TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_3);

cout<<"Points: X: "<<endl<<imagePoint_0[0].x<<" , Y: "<<imagePoint_0[0].y<<endl;

point_msg.x = imagePoint_0[0].x;
point_msg.y = imagePoint_0[0].y;
point_msg.z = probDetect[0];
imgpoint_pub_1.publish(point_msg);

point_msg.x = imagePoint_1[0].x;
point_msg.y = imagePoint_1[0].y;
point_msg.z = probDetect[1];
imgpoint_pub_2.publish(point_msg);

point_msg.x = imagePoint_2[0].x;
point_msg.y = imagePoint_2[0].y;
point_msg.z = probDetect[2];
imgpoint_pub_3.publish(point_msg);

point_msg.x = imagePoint_3[0].x;
point_msg.y = imagePoint_3[0].y;
point_msg.z = probDetect[3];
imgpoint_pub_4.publish(point_msg);

// draw lines of different colours
cv::line(image_in, imagePoint_0[0], imagePoint_1[0], Scalar(0, 255, 0, 160), 2, CV_AA);
cv::line(image_in, imagePoint_1[0], imagePoint_2[0], Scalar(0, 255, 0, 160), 2, CV_AA);
cv::line(image_in, imagePoint_2[0], imagePoint_3[0], Scalar(0, 255, 0, 160), 2, CV_AA);
cv::line(image_in, imagePoint_3[0], imagePoint_0[0], Scalar(0, 255, 0, 160), 2, CV_AA);

putText(image_in, "P_1", imagePoint_0[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);
putText(image_in, "P_2", imagePoint_1[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);
putText(image_in, "P_3", imagePoint_2[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);
putText(image_in, "P_4", imagePoint_3[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);


imshow("Retange_draw",image_in);
imshow("Aruco_detect",image_callback);
