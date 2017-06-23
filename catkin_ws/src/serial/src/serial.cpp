//#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <semaphore.h>

static int st_baud[] =
{
	B4800,
	B9600,
	B19200,
	B38400,
	B57600,
	B115200,
	B1000000,
	B1152000,
	B3000000,
};
static int baudrate[] =
{
	4800,
	9600,
	19200,
	38400,
	57600,
	115200,
	1000000,
	1152000,
	3000000,
};

int g_sdk_uart_fd = -1;

int uart_config(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	int i, j;
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	/*步骤一，设置字符大小*/
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	/*设置停止位*/
	switch (nBits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	/*设置奇偶校验位*/
	switch (nEvent)
	{
	case 'O': //奇数
	case 'o': //奇数
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		//		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': //偶数
	case 'e': //偶数
		//		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  //无奇偶校验位
	case 'n':  //无奇偶校验位
		newtio.c_cflag &= ~PARENB;
		break;
	}
	j = sizeof(baudrate) / 4;
	for (i = 0; i < j; i++)
	{
		if (baudrate[i] == nSpeed)
		{
			cfsetispeed(&newtio, st_baud[i]);
			cfsetospeed(&newtio, st_baud[i]);
			break;
		}
	}
	if ((i == j) && (baudrate[i - 1] != nSpeed))
	{
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
	}
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 1;
	newtio.c_cc[VMIN] = 1;
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	newtio.c_oflag &= ~OPOST;   /*Output*/

	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}

	printf("set UART port parameter done!\n");
	return 0;
}

int connect_serial(const char* port, int baudrate)
{// currently only support baud rate 115200

    g_sdk_uart_fd = open(port, O_RDWR | O_NOCTTY);
	if (g_sdk_uart_fd < 0)
    {
		printf("open UART ERROR!\n");
		return -1;
	}
	printf("open uart ok!\n");
	if (uart_config(g_sdk_uart_fd, baudrate, 8, 'N', 1) < 0)
	{
		printf("UART config ERROR!\n");
		return -1;
	}

	return 0;
}

int read_serial( unsigned char *buf, int len, int timeout )
{
    int read_len = 0;
    for( ; read_len < len; )
    {
        //ros::spinOnce();
	int n = read(g_sdk_uart_fd, buf + read_len, len);
        if( n < 0 )
        {
            break;
        }
        if( 0 == n )
        {
            continue;
        }
        read_len += n;
    }

    return read_len;
}


int UART_Recv(int fd, unsigned char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec = 10;
    time.tv_usec = 0;
    //使用select实现串口的多路通信
    fs_sel = select(g_sdk_uart_fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel)
    {
        len = read(g_sdk_uart_fd,rcv_buf,data_len);
        //printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);
        return len;
    }
    else
    {
        //printf("error send");
        return 0;
    }
}


int write_serial( unsigned char *buf, int len, int timeout )
{
    int write_len = 0;
    for( ; write_len < len; )
    {
        int n = write(g_sdk_uart_fd, buf + write_len, len);
        if( n < 0 )
        {
            break;
        }
        if( 0 == n )
        {
            continue;
        }
        write_len += n;
    }

    return write_len;
}




int disconnect_serial(void)
{
	close(g_sdk_uart_fd);
	g_sdk_uart_fd = -1;
	return 0;
}
