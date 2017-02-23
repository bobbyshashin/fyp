#include <ros/ros.h>
#include "std_msgs/String.h"

#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "pid.h"


using namespace std;
using namespace ros;

VCI_BOARD_INFO pInfo;


ros::Publisher can_transmit_pub;
ros::Publisher can_receive_pub;

PID *pid;


void send_speed(int first, int second, int third, int fourth, VCI_CAN_OBJ &send) { // front_left, bottom_left, bottom_right, front_right
							  // number 1 to 4, respectively
    third = 0 - third;
    fourth = 0 - fourth;
    /* First wheel */
    if(first > 0)
    {
	send.Data[0] = (int)(first/256);
	send.Data[1] = first & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else
    {
	int tmp = 0 - (int)(first/256);
	send.Data[0] = 0 - (tmp+1);
	send.Data[1] = first & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

    /* Second wheel */
    if(second > 0)
    {
	send.Data[2] = (int)(second/256);
	send.Data[3] = second & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else
    {
	int tmp = 0 - (int)(second/256);
	send.Data[2] = 0 - (tmp+1);
	send.Data[3] = second & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

    /* Third wheel */
    if(third > 0)
    {
	send.Data[4] = (int)(third/256);
	send.Data[5] = third & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else
    {
	int tmp = 0 - (int)(third/256);
	send.Data[4] = 0 - (tmp+1);
	send.Data[5] = third & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

    /* Fourth wheel */
    if(fourth > 0)
    {
	send.Data[6] = (int)(fourth/256);
	send.Data[7] = fourth & 0x00ff;
	//send.Data[1] = first - 256*send.Data[0];
    }
    else
    {
	int tmp = 0 - (int)(fourth/256);
	send.Data[6] = 0 - (tmp+1);
	send.Data[7] = fourth & 0x00ff;
	//send.Data[1] = first - (send.Data[0]*256);
    }

}

int* get_speed(VCI_CAN_OBJ *speedData)
{
	
	
}

int main(int argc, char *argv[]) {


   
    ros::init(argc, argv, "ugv_node");
    ros::NodeHandle n;
    //ros::Subscriber orientation_sub = n.subscribe("/dji_sdk/odometry/pose/pose/orientation", 1000, chatterCallback);
	
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

	


    /* Data to be sent */
    //send.Data[0] = 0;
    //send.Data[1] = 0;
	
    //send.Data[2] = 5;
    //send.Data[3] = 100;

    //send.Data[4] = 0;
    //send.Data[5] = 0;
	  
    //send.Data[6] = 0;	
    //send.Data[7] = 0;

    
    ros::Rate loop_rate(100);

    int first = 0;
    int second = 0;
    int third = 0;
    int fourth = 0;

    int realSpeed = 0;
    
    send_speed(800,800,800,800, sendData);
	
    while (ros::ok()) {


	//DWORD dwRel;
	//printf("I'm cleaning your shit \n");
	//dwRel = VCI_ClearBuffer(VCI_USBCAN2, 0 , 0);
	//printf("This is return value %d \n", dwRel);


    int reclen=0;
    VCI_CAN_OBJ rec[40];
    int i;
	
    printf("running....\n");
	
    if((reclen=VCI_Receive(VCI_USBCAN2,0,0,rec,40,0))>0) {

	/* Print frame numbers */	
	printf("Received %d frames \n", reclen);
		
	/* Print info for each frame */
	for (int j=0; j<reclen; j++){
	    /* Print CAN ID */
	    printf("Receive: %08X \n", rec[j].ID);

		for(int p = 0; p < rec[reclen-1].DataLen; p++) {
		        /* Print the data under this CAN ID  */
            		printf(" %08X", rec[j].Data[p]);
		}

	    if (rec[j].ID == 0x201) { first++; }
	    else if (rec[j].ID == 0x202) { second++; }
	    else if (rec[j].ID == 0x203) { third++; }
	    else if (rec[j].ID == 0x204) { fourth++; }

	    printf("\n first: %d \n", first);
	    printf("second: %d \n", second);
	    printf("third: %d \n", third);
	    printf("fourth: %d \n", fourth);

	realSpeed = rec[j].Data[2]*256 + rec[j].Data[3];
	printf("Real speed is %d \n", realSpeed);
	printf("\n");


	}

	     
	//printf("IND:%d Receive: %08X", ind, rec[reclen-1].ID);
	

	

	int speed1 = 0;
	int speed2 = 0;
	int speed3 = 0;
	int speed4 = 0;
	
	
    }	

    /* Transmit */
		
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, &sendData, 1) > 0) {

	printf("CAN 1 ID: %08X\r\n",sendData.ID);
	printf("CAN 1 data :\r\n");

	for(i=0;i<sendData.DataLen;i++) {
            printf(" %08X",sendData.Data[i]);
	    //printf(" Decimal: %d",sendData.Data[i]);
	}

	printf("\n");
	//send[0].ID=sendind++;
	//send[1].ID=sendind++;
	//send[2].ID=sendind++;
    }

    else
        break;
	
	

    //can_receive_pub.()
		
    //int input = 0;
    //std::cin>>input;
    //if (input==1)
    //{break;}		
    ros::spinOnce();
    loop_rate.sleep();

    }

printf("\n Exit loop already \n");

DWORD dwRel2;

printf("I'm cleaning \n");
dwRel2 = VCI_ClearBuffer(VCI_USBCAN2, 0 , 0);
VCI_CloseDevice(VCI_USBCAN2,0);

//ext:	
//	VCI_CloseDevice(VCI_USBCAN2,0);
}
