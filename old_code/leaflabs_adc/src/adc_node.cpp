/*
 * adc_node.cpp
 *
 *  Created on: May 19, 2011
 *      Author: golfcar
 */

#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <SerialStream.h>
using namespace LibSerial ;


SerialStream serial_port ;

void openSerialPort()
{
    serial_port.Open( "/dev/maple" ) ;
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
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_57600 ) ;
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
}

int main( int argc, char** argv  )
{
    ros::init(argc, argv, "adc_node");
    ros::NodeHandle n;
    ros::Publisher adc_pub = n.advertise<std_msgs::Float64>("adc", 2);

    openSerialPort();

    //
    // Wait for some data to be available at the serial port.
    //

    //
    // Keep reading data from serial port and print it to the screen.
    //
    int count=0, raw_data=0;
    const int average=50;
    while(ros::ok())
    {
        while( serial_port.rdbuf()->in_avail() == 0  ){usleep(100);}
        char next_byte;
        char resp_1, resp_2;
        serial_port.get(next_byte);
        //std::cout<<1<<" ";
        if(next_byte==0x7D)
        {
            while( serial_port.rdbuf()->in_avail() == 0  ){usleep(100);}
            serial_port.get(next_byte);
            //std::cout<<2<<" ";

            //0 degree 3532
            //-90 degree 2075
            //
            float resolution = 90.0/(3532-2075);
            if(next_byte==0x5D)
            {
                //std::cout<<3<<" ";
                while( serial_port.rdbuf()->in_avail() ==0 ) {usleep(100);}
                serial_port.get(resp_1);
                while( serial_port.rdbuf()->in_avail() ==0 ) {usleep(100);}
                serial_port.get(resp_2);
                //std::cerr << int(resp_1&0xff)*256+int(resp_2&0xff) << "\n";
                int raw_message =  int(resp_1&0xff)*256+int(resp_2&0xff) ;
                if(count<average)
                {
                    raw_data+=raw_message;
                    count++;
                }
                else
                {
                    std_msgs::Float64 adc_value;
                    adc_value.data=((raw_data/average-3532)*(float)resolution);///4095.0*270)-227;// = (float)raw_message;
                    adc_pub.publish(adc_value);
                    //std::cout<<4<<" ";
                    //std::cout<<int(next_byte)<<"\n";

                    btMatrix3x3 btm;// = new btMatrix3x3();
                    btm.setRPY(0, (-adc_value.data)/180.0*M_PI, 0);
                    tf::Quaternion qt_temp;
                    btm.getRotation(qt_temp);
                    static tf::TransformBroadcaster broadcaster_b;

                    broadcaster_b.sendTransform(
                        tf::StampedTransform(
                            tf::Transform(qt_temp, tf::Vector3(0, 0, 0)),
                                             ros::Time::now(),"/tilt_base", "/tilt_hokuyo"));
                    count=0; raw_data=0;
                }
            }
        }
        //std::cout<<std::endl;
    }

    return EXIT_SUCCESS ;
}
