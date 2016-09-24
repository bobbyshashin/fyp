#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

#define FALSE  -1
#define TRUE   0


int  UART_Open(int fd,char* port);
void UART_Close(int fd);
int  UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int  UART_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
int  UART_Recv(int fd, unsigned char *rcv_buf,int data_len);
int  UART_Send(int fd, unsigned char *send_buf,int data_len);
