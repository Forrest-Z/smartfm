#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <deque>
#include <vector>
#include "DigitalCompassHoneywell/DigitalCompass.h"

      #define BAUDRATE B19200
      #define COMPASSDEVICE "/dev/ttyS0"
      #define _POSIX_SOURCE 1 /* POSIX compliant source */
      #define FALSE 0
      #define TRUE 1
        
      volatile int STOP=FALSE; 


int main(int argc, char **argv)
{
// ROS initiation
  ros::init(argc, argv, "Compass");
  ros::NodeHandle n;

  ros::Publisher compass_pub = n.advertise<std_msgs::String>("CompassRaw", 1000);
  ros::Publisher compass_pub1 = n.advertise<DigitalCompassHoneywell::DigitalCompass>("DigitalCompass", 1000);
  ros::Rate loop_rate(10);

// COM port connection code
        int fd,c, res;
        struct termios oldtio,newtio;
        char buf[255];
        
        fd = open(COMPASSDEVICE, O_RDWR | O_NOCTTY ); 
        if (fd <0) {perror(COMPASSDEVICE); exit(-1); }
        
        tcgetattr(fd,&oldtio); /* save current port settings */
        
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        
        /* set input mode (non-canonical, no echo,...) */
        newtio.c_lflag = 0;
         
        newtio.c_cc[VTIME]    = 1;   /* inter-character timer set as one tenth of a second */
        newtio.c_cc[VMIN]     = 20;   /* blocking read until 20 chars received */
        
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);
        
        
        while (ros::ok()) {       /* loop for input */
   


          res = read(fd,buf,255);   /* returns after 5 chars have been input */
          buf[res]=0;               /* so we can printf... */
  //        printf(":%s\n", buf);
          if (buf[0]=='z') STOP=TRUE;

/**********************The ROS PART Finally..****************************/
    std_msgs::String msg;
    DigitalCompassHoneywell::DigitalCompass Dcompass;
    float heading1, pitch1 , roll1;


    std::stringstream ss;
    ss << buf;
    msg.data = ss.str();
    ROS_INFO("The raw output from the compass : %s", msg.data.c_str());

//try spliting the string into floats 
std::string str1 = ss.str();
int size = str1.size();
    //ROS_INFO("%i", size);
int pos = str1.find(",");
std::string strh = str1.substr(0,pos);
//std::cout<<strh<<" \n";
str1.replace(pos,1," ");
int pos1 = str1.find(",");
std::string strp = str1.substr(pos +1, pos1 -pos -1);
//std::cout<<strp<<" \n";
std::string strr = str1.substr(pos1 +1);
//std::cout<<strr<<" \n";


//conversion into floating values and publishing the messages

    
    heading1 = (double)atof(strh.c_str());
    pitch1   = (double)atof(strp.c_str());
    roll1    = (double)atof(strr.c_str());
    
    Dcompass.heading = ((int)(heading1 *100 +.5)/100.0);
    Dcompass.pitch  = ((int)(pitch1 *100 +.5)/100.0);
    Dcompass.roll = ((int)(roll1 *100 +.5)/100.0);
    compass_pub1.publish(Dcompass);

    compass_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();


        }
        tcsetattr(fd,TCSANOW,&oldtio);

  return 0;
}
