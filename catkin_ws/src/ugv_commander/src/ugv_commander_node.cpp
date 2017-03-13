#include <ros/ros.h>
#include "std_msgs/String.h"

#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>


#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <stdint.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <termios.h>


using namespace std;
using namespace ros;

ros::Publisher speed_value_pub;
geometry_msgs::Quaternion speed_value;

int speed_counter = 0;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "ugv_commander_node");
    ros::NodeHandle n;
    
    speed_value_pub = n.advertise<geometry_msgs::Quaternion>("/speed_input", 1);
    
    
    ros::Rate loop_rate(10);

    
    int first = 0;
    int second = 0;
    int third = 0;
    int fourth = 0;

    speed_value.x = 0;
    speed_value.y = 0;
    speed_value.z = 0;
    speed_value.w = 0;


    while (ros::ok()) {
    /*
    cin>>first;
    cin>>second;
    cin>>third;
    cin>>fourth;
    
    speed_value.x = first;
    speed_value.y = second;
    speed_value.z = third;
    speed_value.w = fourth;
    */
    
    

    int c = getch();   // call your non-blocking input function
    if (c == 'w') {

	speed_counter += 100;
	
	if (speed_counter > 1600)
	    speed_counter = 1600;
	
	speed_value.x = speed_counter;
    	speed_value.y = speed_counter;
    	speed_value.z = speed_counter;
    	speed_value.w = speed_counter;
	
    	
    }
    else if (c == 's') {

        speed_counter -= 100;
        
        if (speed_counter < -1600) 
	    speed_counter = -1600;

	speed_value.x = speed_counter;
    	speed_value.y = speed_counter;
    	speed_value.z = speed_counter;
    	speed_value.w = speed_counter;
    }
    else if (c == 'a') {
        
	speed_value.x = - speed_counter;
    	speed_value.y = - speed_counter;
    	speed_value.z = speed_counter;
    	speed_value.w = speed_counter;
        
    }
    else if (c == 'd') {

        speed_value.x = speed_counter;
    	speed_value.y = speed_counter;
    	speed_value.z = - speed_counter;
    	speed_value.w = - speed_counter;

    }
    else {

	speed_counter = 0;
	speed_value.x = 0;
    	speed_value.y = 0;
    	speed_value.z = 0;
    	speed_value.w = 0;

    }

    speed_value_pub.publish(speed_value);

    ros::spinOnce();
    loop_rate.sleep();

    }

    


//ext:	
//	VCI_CloseDevice(VCI_USBCAN2,0);
}
