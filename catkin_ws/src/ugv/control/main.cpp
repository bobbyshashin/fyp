#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include <iostream>
#include "unistd.h"

VCI_BOARD_INFO pInfo;

using namespace std;

void *receive_func(void* param) {

	int reclen = 0;
	VCI_CAN_OBJ rec[100];
	int i;
	
	int *run=(int*)param;
	int ind=((*run)>>4);
	
	while((*run)&0x0f)
	{
//		printf("running....%d\n",ind);
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,2500,100))>0)
		{
			printf("IND:%d Receive: %08X", ind, rec[reclen-1].ID);
			for(i = 0; i < rec[reclen-1].DataLen; i++)
			{
				printf(" %08X", rec[reclen-1].Data[i]);
			}
			printf("\n");
		}	
	}
	printf("run thread exit\n");
	
	pthread_exit(0);
}


int main() {

	if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)
	
		printf(">>open deivce success!\n");
	
	else {

		printf(">>open deivce error!\n");
		exit(1);
	}


	if(VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1)
	
        printf(">>Get VCI_ReadBoardInfo success!\n");

	else {

		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}
	

	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xffffffff;
	config.Filter=1;
	config.Mode=0;

	/*Baud rate = 1 Mbps*/	
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

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1) {

		printf("init can 1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1) {

		printf("start can 1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	/*
	VCI_CAN_OBJ send[3];

	send[0].ID 			= 0;
	send[0].SendType	= 2;
	send[0].RemoteFlag	= 0;
	send[0].ExternFlag	= 1;
	send[0].DataLen 	= 8;

	send[1] = send[0];
	send[2] = send[0];

	send[1].ID = 1;
	send[2].ID = 2;
	*/

	VCI_CAN_OBJ send;

	send.ID 	    = 0x200;
	send.SendType 	= 2;
	send.RemoteFlag = 0;
	send.ExternFlag = 0;
	send.DataLen 	= 8;
	
	
	int i = 0;
	
	/*for(i = 0; i < send.DataLen-2; i++)
	{
		send.Data[i] = 20;
		//send[1].Data[i] = i + send[0].Data[i];
		//send[2].Data[i] = i + send[1].Data[i];
	}*/
	
	send.Data[0] = 5;
	send.Data[1] = 255;
	
	send.Data[2] = 5;
	send.Data[3] = 255;

	send.Data[4] = 5;
	send.Data[5] = 255;
	
	send.Data[6] = 0xFB;	
	send.Data[7] = 255;
	
	int m_run_0 = 1;
	int m_run_1 = 0x11;

	pthread_t threadid_0;
	pthread_t threadid_1;
	
	int ret;
	ret = pthread_create(&threadid_0, NULL, receive_func, &m_run_0);
	ret = pthread_create(&threadid_1, NULL, receive_func, &m_run_1);

	int times = 25;
	int sendind = 3;
	time_t tm1, tm2;
	time(&tm1);

	while(times--)
	{
		if(VCI_Transmit(VCI_USBCAN2, 0, 0, &send, 1) > 0) {

			printf("CAN 1 ID: %04X\r\n",send.ID);
			printf("CAN 1 data :\r\n");
			for(i = 0; i < send.DataLen; i++) {

				printf("%04X",send.Data[i]);
			}
			
			cout << endl;
			//send[0].ID=sendind++;
			//send[1].ID=sendind++;
			//send[2].ID=sendind++;
		}

		else {

			break;
		}

		usleep(1);

	}

	//time(&tm2);
	
	usleep(100);
	
	//printf("minute:%d   second:%d\n",(tm2-tm1)/60,(tm2-tm1)%60);

	m_run_0 = 0;
	m_run_1 = 0x10;
	pthread_join(threadid_0,NULL);
	pthread_join(threadid_1,NULL);

//ext:	
//	VCI_CloseDevice(VCI_USBCAN2,0);
}
