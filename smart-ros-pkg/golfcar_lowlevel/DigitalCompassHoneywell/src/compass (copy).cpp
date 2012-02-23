#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>

      #define BAUDRATE B19200
      #define MODEMDEVICE "/dev/ttyS0"
      #define _POSIX_SOURCE 1 /* POSIX compliant source */
      #define FALSE 0
      #define TRUE 1
        
      volatile int STOP=FALSE; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Compass");
  ros::NodeHandle n;

  ros::Publisher compass_pub = n.advertise<std_msgs::String>("CompassRaw", 1000);

  ros::Rate loop_rate(10);
/*

  //
    // Open the serial port.
    //

    printf("the code started\n");    

    using namespace LibSerial ;
    SerialStream serial_port ;
    serial_port.Open( "/dev/ttyS0" ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port." 
                  << std::endl ;
        exit(1) ;
    }

  //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_19200 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Turn off hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Do not skip whitespace characters while reading from the
    // serial port.
    //
    // serial_port.unsetf( std::ios_base::skipws ) ;
    //
    // Wait for some data to be available at the serial port.
    //
    while( serial_port.rdbuf()->in_avail() == 0 ) 
    {
        usleep(100) ;
    }

*/

        int fd,c, res;
        struct termios oldtio,newtio;
        char buf[255];
        
        fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
        if (fd <0) {perror(MODEMDEVICE); exit(-1); }
        
        tcgetattr(fd,&oldtio); /* save current port settings */
        
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        
        /* set input mode (non-canonical, no echo,...) */
        newtio.c_lflag = 0;
         
        newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
        newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */
        
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);
        
        
        while (ros::ok()) {       /* loop for input */
          res = read(fd,buf,255);   /* returns after 5 chars have been input */
          buf[res]=0;               /* so we can printf... */
          printf(":%s\n", buf);
          if (buf[0]=='z') STOP=TRUE;
        }
        tcsetattr(fd,TCSANOW,&oldtio);

/**********************The ROS PART Finally..****************************/

  
  while (ros::ok())
  {

      /*  char next_byte;
        serial_port.get(next_byte);
        std::cerr << (next_byte) << ""; 
        usleep(100) ;*/

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    compass_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
