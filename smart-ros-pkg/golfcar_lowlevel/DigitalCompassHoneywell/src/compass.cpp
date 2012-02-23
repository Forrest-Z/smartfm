/*
Copyright (c) 2012, Zac Arackakudyil (National University of Singapore)
All rights reserved.

- Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


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

#include <SerialStream.h>



int main(int argc, char **argv)
{
// ROS initiation - The node is called Compass
  ros::init(argc, argv, "Compass");
  ros::NodeHandle n;

//Two topics are published. One is the raw output in ASCII from the Digital Compass and the other is topic that includes floating values of each heading, pitch and roll
  ros::Publisher compass_pub = n.advertise<std_msgs::String>("CompassRaw", 1000);
  ros::Publisher compass_pub1 = n.advertise<DigitalCompassHoneywell::DigitalCompass>("DigitalCompass", 1000);
  ros::Rate loop_rate(1000);

// COM port connection code
    using namespace LibSerial;
    SerialStream serial_port;
//Please not that you will have to change this value as per as the address of the Digital Compass in your robot
    serial_port.Open( "/dev/ttyS0" ) ;

    //Error management for serial port
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
    //
    // Keep reading data from serial port and print it to the screen.
    //
   // while( serial_port.rdbuf()->in_avail() > 0  ) 
   
 while (ros::ok()) { 
  
     
 
        std::stringstream ss;
        
        do{
          char next_byte;
          serial_port.get(next_byte);
          ss<<(next_byte);
        //std::cerr << std::dec << static_cast<int>(next_byte) << " \n";
                //std::cout << (next_byte) << "";
        //printf(" one byte received... \n");
          usleep(1) ;
       // std::cerr<<"yay I read something"; 
          if ((next_byte)=='\n') break;
           
          }while (1);

         std::cout<<ss.str() <<"";
         

/**********************The ROS PART Finally..****************************/
    std_msgs::String msg;
    DigitalCompassHoneywell::DigitalCompass Dcompass;
    float heading1, pitch1 , roll1;

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

 
    ros::spinOnce();
    loop_rate.sleep();


        }
     
    std::cerr << std::endl ;
  return 0;
}
