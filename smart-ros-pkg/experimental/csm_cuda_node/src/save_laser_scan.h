#include <fstream>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std;

void readWriteScan(sensor_msgs::LaserScanConstPtr msg){
  sensor_msgs::LaserScan ls;
  rosbag::Bag bag("laser_data.bag", rosbag::bagmode::Write);
  bag.write("laser", ros::Time::now(), *msg);
  bag.close();
  rosbag::Bag read_bag("laser_data.bag");
  
  rosbag::View view(read_bag, rosbag::TopicQuery("laser"));
  
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::LaserScan::ConstPtr i = m.instantiate<sensor_msgs::LaserScan>();
    if (i != NULL){
	ls = *i;
	break;
    }
  }
  bag.close();
}

template<class T>
void writeROSMsg(T &my_value, string file_name){
  
  ros::SerializedMessage serial_msg = ros::serialization::serializeMessage<T>(my_value);
  ofstream output_file(file_name.c_str(), ios::binary);
  output_file.write((const char*)serial_msg.message_start, serial_msg.num_bytes - (serial_msg.message_start-serial_msg.buf.get()));
  output_file.close();
}

template<class T>
T readROSFile(string file_name){
  ifstream input_file(file_name.c_str(), ios::binary);
  input_file.seekg (0, input_file.end);
  int length = input_file.tellg();
  input_file.seekg (0, input_file.beg);
  char buffer[length];
  input_file.read(&buffer[0], length);
  uint8_t *buffer_unsigned = reinterpret_cast<uint8_t*>(buffer);
  ros::serialization::IStream serial_msg(buffer_unsigned, length);
  T msg;
  ros::serialization::deserialize(serial_msg, msg);
  return msg;
}