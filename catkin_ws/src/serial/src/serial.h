#ifndef SERIAL_H
#define SERIAL_H

int connect_serial(const char* port, int baudrate = 115200);

int read_serial( unsigned char* data, int max_size, int timeout );
int write_serial( unsigned char* data, int max_size, int timeout );
int UART_Recv(int fd, unsigned char *rcv_buf,int data_len);
void disconnect_serial();

#endif
