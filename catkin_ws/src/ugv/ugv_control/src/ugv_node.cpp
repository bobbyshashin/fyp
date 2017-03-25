#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "pid.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace ros;

VCI_BOARD_INFO pInfo;

/* CAN publisher */
Publisher can_transmit_pub;
Publisher can_receive_pub;

/* PID Parameter Publisher */
Publisher pid_param_pub;

/* Speed Publisher */
Publisher speed_pub;

/* Error Publisher */
Publisher error_pub;

/* Control Publisher */
Publisher control_pub;

/* PID Parameter Tuning */
Subscriber pid_parameter_sub;       // For tuning PID parameters (Kp, Ki and Kd)

/* Get User Input Speed */
Subscriber target_speed_sub;
  
/* Subscribe target speed from keyboard */
Subscriber keyboard_speed_sub;


/* Subscribe the rotation angles */
Subscriber global_to_body_angle_sub;
Subscriber initial_angle_sub;

geometry_msgs::Vector3 pid_param;
geometry_msgs::Quaternion speed;
geometry_msgs::Quaternion control;
geometry_msgs::Quaternion error;

PID *pid1;
PID *pid2;
PID *pid3;
PID *pid4;

float Kp1 = 0.6;
float Ki1 = 0.15;
float Kd1 = 0.1;

float Kp2 = 0.45;
float Ki2 = 0.1;
float Kd2 = 0.2;

int16_t targetSpeed[4] = {2000, 2000, 2000, 2000};
int16_t currentSpeed[4] = {0,0,0,0}; 

float ctrl[4] = {0,0,0,0}; // data sent to CAN
int threshold = 10;

float dt = 0.1;

float pid_ctrl_limit = 4000;
float integralLimit = 40000;

void get_speed(int32_t id, int16_t high, int16_t low) {

    if(id == 0x201) 
        currentSpeed[0] = (high<<8) | (low);
    else if(id == 0x202) 
	currentSpeed[1] = (high<<8) | (low);		
    else if(id == 0x203) 
        currentSpeed[2] = 0 - ((high<<8) | (low));		
    else if(id == 0x204) 
        currentSpeed[3] = 0 - ((high<<8) | (low));		
    
} 

void keyboard_speed_callback(const geometry_msgs::Twist& msg) {

    pid1 -> set_point(msg.linear.x * 400);
    pid2 -> set_point(msg.linear.x * 400);
    pid3 -> set_point(msg.linear.x * 400);
    pid4 -> set_point(msg.linear.x * 400);

}

void pid_parameter_tuning_callback(const geometry_msgs::Vector3& msg) {

   
    pid1 -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd
    pid2 -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd
    pid3 -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd
    pid4 -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd

    cout << "PID parameters for horizontal position control have been updated!" << endl;
    cout << "Kp: " << msg.x << endl;
    cout << "Ki: " << msg.y << endl;
    cout << "Kd: " << msg.z << endl;

}

void target_speed_callback(const geometry_msgs::Quaternion& msg) {

    targetSpeed[0] = msg.x;
    targetSpeed[1] = msg.y;
    targetSpeed[2] = msg.z;
    targetSpeed[3] = msg.w;

    pid1 -> set_point(msg.x);
    pid2 -> set_point(msg.y);
    pid3 -> set_point(msg.z);
    pid4 -> set_point(msg.w);

    cout << "Speed updated!" << endl;
}


void get_control(int16_t *current, int16_t *target) {
	
    int16_t error1 = abs(current[0] - target[0]);
    int16_t error2 = abs(current[1] - target[1]);
    int16_t error3 = abs(current[2] - target[2]);
    int16_t error4 = abs(current[3] - target[3]);

    error.x = target[0] - current[0]; 
    error.y = target[1] - current[1];
    error.z = target[2] - current[2];
    error.w = target[3] - current[3];


    cout << "Errors:" << endl 
	 <<   error1  << endl 
	 <<   error2  << endl
	 <<   error3  << endl
	 <<   error4  << endl;
	
    if(error1 > threshold)
        ctrl[0] = pid1 -> update(current[0], dt);	
    //else if(error1 <= threshold)
        //ctrl[0] = 0;
    if(error2 > threshold)
        ctrl[1] = pid2 -> update(current[1], dt);	
    //else if(error2 <= threshold)
        //ctrl[1] = 0;
    if(error3 > threshold)
	ctrl[2] = pid3 -> update(current[2], dt);	
    //else if(error3 <= threshold)
	//ctrl[2] = 0;
    if(error4 > threshold)
	ctrl[3] = pid4 -> update(current[3], dt);	
    //else if(error4 <= threshold)
	//ctrl[3] = 0;

    /* Limit control signals */
    if(ctrl[0] > pid_ctrl_limit){ ctrl[0] = pid_ctrl_limit;}
    if(ctrl[1] > pid_ctrl_limit){ ctrl[1] = pid_ctrl_limit;}
    if(ctrl[2] > pid_ctrl_limit){ ctrl[2] = pid_ctrl_limit;}
    if(ctrl[3] > pid_ctrl_limit){ ctrl[3] = pid_ctrl_limit;}
}

void send_speed(int first, int second, int third, int fourth, VCI_CAN_OBJ &send) 
{ 
    // front_left   bottom_left   bottom_right   front_right 
    //     1             2             3             4

    third = 0 - third;
    fourth = 0 - fourth;

    /* First wheel */
    if(first >= 0) {
	send.Data[0] = (int)(first/256);
	send.Data[1] = first & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else {
	int tmp = 0 - (int)(first/256);
	send.Data[0] = 0 - (tmp+1);
	send.Data[1] = first & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

    /* Second wheel */
    if(second >= 0) {
	send.Data[2] = (int)(second/256);
	send.Data[3] = second & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else {
	int tmp = 0 - (int)(second/256);
	send.Data[2] = 0 - (tmp+1);
	send.Data[3] = second & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

    /* Third wheel */
    if(third >= 0) {
	send.Data[4] = (int)(third/256);
	send.Data[5] = third & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else {
	int tmp = 0 - (int)(third/256);
	send.Data[4] = 0 - (tmp+1);
	send.Data[5] = third & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

    /* Fourth wheel */
    if(fourth >= 0) {
	send.Data[6] = (int)(fourth/256);
	send.Data[7] = fourth & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else {
	int tmp = 0 - (int)(fourth/256);
	send.Data[6] = 0 - (tmp+1);
	send.Data[7] = fourth & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }
}


int main(int argc, char *argv[]) {


   
    ros::init(argc, argv, "ugv_node");
    ros::NodeHandle n;

    /* Publish PID, speed, control */
    pid_param_pub = n.advertise<geometry_msgs::Vector3>("/pid_param", 1);
    speed_pub = n.advertise<geometry_msgs::Quaternion>("/speed", 1);
    control_pub = n.advertise<geometry_msgs::Quaternion>("/control", 1);
    error_pub = n.advertise<geometry_msgs::Quaternion>("/error", 1);

    //ros::Subscriber orientation_sub = n.subscribe("/dji_sdk/odometry/pose/pose/orientation", 1000, chatterCallback);
    pid_parameter_sub    = n.subscribe("/ugv_pid_parameter",    1, pid_parameter_tuning_callback);

    target_speed_sub = n.subscribe("/ugv_target_speed", 1, target_speed_callback);
    keyboard_speed_sub = n.subscribe("/cmd_vel", 1, keyboard_speed_callback);

    pid1 = new PID( Kp1, Ki1, Kd1, -integralLimit, integralLimit, -pid_ctrl_limit, pid_ctrl_limit, false);
    pid2 = new PID( Kp1, Ki1, Kd1, -integralLimit, integralLimit, -pid_ctrl_limit, pid_ctrl_limit, false);
    pid3 = new PID( Kp2, Ki2, Kd2, -integralLimit, integralLimit, -pid_ctrl_limit, pid_ctrl_limit, false);
    pid4 = new PID( Kp2, Ki2, Kd2, -integralLimit, integralLimit, -pid_ctrl_limit, pid_ctrl_limit, false);

    pid1 -> set_point(targetSpeed[0]);
    pid2 -> set_point(targetSpeed[1]);
    pid3 -> set_point(targetSpeed[2]);
    pid4 -> set_point(targetSpeed[3]);
	
    can_receive_pub = n.advertise<std_msgs::String>("can_transmit", 1);

    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1) { 
	printf(">>open deivce success!\n");
    }
    else {
	printf(">>open deivce error!\n");
	exit(1);
    }

    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1) {
        printf(">>Get VCI_ReadBoardInfo success!\n");
    }
    else {
	printf(">>Get VCI_ReadBoardInfo error!\n");
	exit(1);
    }

    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xffffffff;
    config.Filter=1;
    config.Mode=0;

    /*1Mbps  0x14  0x00*/	
    config.Timing0=0x00;
    config.Timing1=0x14;
	
    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1) {
	printf("init CAN error\n");
	VCI_CloseDevice(VCI_USBCAN2,0);
    }
    
    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1) {
        printf("Start CAN error\n");
	VCI_CloseDevice(VCI_USBCAN2,0);
    }
	


    /* Config */
    VCI_CAN_OBJ sendData;
    sendData.ID = 0x200;
    sendData.SendType = 2;
    sendData.RemoteFlag = 0;
    sendData.ExternFlag = 0;
    sendData.DataLen = 8;

    
    ros::Rate loop_rate(100);					

    int first = 0;
    int second = 0;
    int third = 0;
    int fourth = 0;
    
    int averSpeed1 = 0;
    int averSpeed2 = 0;
    int averSpeed3 = 0;
    int averSpeed4 = 0;
    
    //send_speed(800,800,800,800, sendData);

    int loopNumber = 0;

    /* Major loop */
    while (ros::ok()) {

	DWORD dwRel;
	printf("I'm cleaning \n");
	dwRel = VCI_ClearBuffer(VCI_USBCAN2, 0 , 0);

    	//pid_param_pub.publish(pid_param);
    	//speed_pub.publish(speed);
    	//control_pub.publish(control);
    	//error_pub.publish(error);

    	loopNumber++;
    	cout<<"This is "<<loopNumber<<" loop"<<endl;
    	int reclen=0;
    	VCI_CAN_OBJ rec[160];
    	int i;
	
    	printf("running....\n");
		    
	first       = 0;
	second      = 0;
	third       = 0;
	fourth 	    = 0;

	averSpeed1  = 0;
	averSpeed2  = 0;
	averSpeed3  = 0;
	averSpeed4  = 0;

    	/* Receive */
    	if((reclen=VCI_Receive(VCI_USBCAN2,0,0,rec,160,0))>0) {

	    /* Print info for each frame */
	    for (int j=0; j<reclen; j++){
	    /* Print CAN ID */
	    printf("Receive: %08X \n", rec[j].ID);
	    
	    /* Print frame numbers */	
	    printf("\n Received %d frames \n", reclen);
	    for(int p = 0; p < rec[reclen-1].DataLen; p++) {
	        /* Print the data under this CAN ID  */
            	printf(" %08X", rec[j].Data[p]);
	    }

	    get_speed(rec[j].ID, rec[j].Data[2], rec[j].Data[3]);
	  

 	    if (rec[j].ID ==0x201) {
	    	averSpeed1 = averSpeed1 + currentSpeed[0];
		first++;
	    }
	    else if (rec[j].ID ==0x202) {
		averSpeed2 = averSpeed2 + currentSpeed[1];
		second++;
	    }
	    else if (rec[j].ID ==0x203) {
		averSpeed3 = averSpeed3 + currentSpeed[2];
		third++;
	    }
	    else if (rec[j].ID ==0x204) {
		averSpeed4 = averSpeed4 + currentSpeed[3];
		fourth++;
	    }

	    printf("\n first: %d \n", first);
	    printf("second: %d \n", second);
	    printf("third: %d \n", third);
	    printf("fourth: %d \n", fourth);

	    printf("Real speed is %d\n %d\n %d\n %d\n", currentSpeed[0], currentSpeed[1], currentSpeed[2], currentSpeed[3]);
	}

	    /* Calculate average speed */
	    if (first != 0)
		averSpeed1 = averSpeed1 / first;
	    else
	    	averSpeed1 = currentSpeed[0];
   	    if (second != 0)
		averSpeed2 = averSpeed2 / second;
	    else
	    	averSpeed2 = currentSpeed[1];
   	    if (third != 0)
		averSpeed3 = averSpeed3 / third;
	    else
	    	averSpeed3 = currentSpeed[2];
   	    if (fourth != 0) 
		averSpeed4 = averSpeed4 / fourth;
	    else
	    	averSpeed4 = currentSpeed[3];
	    
	    currentSpeed[0] = averSpeed1;
	    currentSpeed[1] = averSpeed2;
	    currentSpeed[2] = averSpeed3;
	    currentSpeed[3] = averSpeed4;
    }	

    get_control(currentSpeed, targetSpeed);

    cout << "Control signals: " 
	 << ctrl[0] << endl
	 << ctrl[1] << endl
	 << ctrl[2] << endl 
	 << ctrl[3] << endl;

    send_speed(ctrl[0],ctrl[1],ctrl[2],ctrl[3], sendData);
    //send_speed(0, ctrl[1], 0, 0, sendData);

    usleep(2000);

    /* Transmit */		
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, &sendData, 1) > 0) {

	printf("CAN 1 ID: %08X\r\n",sendData.ID);
	printf("CAN 1 data :\r\n");

	for(i=0;i<sendData.DataLen;i++) {
            printf(" %08X",sendData.Data[i]);
	    //printf(" Decimal: %d",sendData.Data[i]);
	}

	printf("\n");
	
    }

    else
        break;

    pid_param.x = Kp1;
    pid_param.y = Ki1;
    pid_param.z = Kd1;

    speed.x = currentSpeed[0];
    speed.y = currentSpeed[1];
    speed.z = currentSpeed[2];
    speed.w = currentSpeed[3];

    control.x = ctrl[0];
    control.y = ctrl[1];
    control.z = ctrl[2];
    control.w = ctrl[3];

    pid_param_pub.publish(pid_param);
    speed_pub.publish(speed);
    control_pub.publish(control);
    error_pub.publish(error);

		
    ros::spinOnce();
    loop_rate.sleep();

    } // End of while loop


    printf("\n Terminating... \n");
    DWORD dwRel2;

    printf("I'm cleaning \n");
    dwRel2 = VCI_ClearBuffer(VCI_USBCAN2, 0 , 0);
    VCI_CloseDevice(VCI_USBCAN2,0);

    //ext:	
    //	VCI_CloseDevice(VCI_USBCAN2,0);
}