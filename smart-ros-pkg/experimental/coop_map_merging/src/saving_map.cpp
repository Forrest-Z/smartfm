//
// Created 31 July 2013, 
// by Sean Kim, Xiaotong Shen
//
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>

using namespace std;

#define stringArray std::vector<std::string>

stringArray listString;
stringArray data;

void readInput(char* fileName) {
    printf("****************ReadInput***************\n");
 
    // read file
    ifstream fin(fileName);
    string tmp = ""; // a line in the input file
    while(getline(fin, tmp)) {
        // add string into vector
        listString.push_back(tmp);
//          cout << tmp << endl;
        cout << listString[listString.size() - 1] << endl;
    }
    printf("\n\n");
    fin.close();
}
 
/*
 * parseValues is to extract data in the input file
 */
void parseValues(sensor_msgs::PointCloud &pcl_scan) {
  printf("****************ParseValue***************\n");
  for (int i = 0; i < listString.size(); ++i) {
    char tmp[100000];
		
    strcpy(tmp, listString[i].c_str()); // copy string to char array
    stringArray tmpArray;
    // utilize string token to extract data
    char * pch;
    pch = strtok (tmp,",");

//    while (pch != NULL) {
    geometry_msgs::Point32 pt;

    pt.x = atof(pch);
    pch = strtok (NULL, ",");
    pt.y = atof(pch);
    pch = strtok (NULL, ",");
    pt.z = atof(pch);
    pch = strtok (NULL, ",");
//    }
    pcl_scan.points.push_back(pt);
  }
}
 
/*
 * parseValues is to write data at the beginning of file
 */
void writeToCSV(char* fileName, sensor_msgs::PointCloud &pcl_scan1) {
  ofstream fout(fileName);

  // for each row
  for (int i = 0; i < pcl_scan1.points.size(); ++i) {
    // for each column
    fout << pcl_scan1.points[i].x << ',' << pcl_scan1.points[i].y << ',' << pcl_scan1.points[i].z << "\r\n";
  }
  fout.close();
}

void scanToPointclouds(const sensor_msgs::LaserScan::ConstPtr& msg, sensor_msgs::PointCloud &cloud_in)
{
  cloud_in.header = msg->header;

  cloud_in.header.frame_id = "/ldmrs0";

  for(unsigned int i = 0; i < msg->ranges.size(); i++)
  {
     geometry_msgs::Point32 pt;

     pt.x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
     pt.y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
     pt.z = 0;
     cloud_in.points.push_back(pt);
  }
}

void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::PointCloud pcl_scan1;

  scanToPointclouds(msg, pcl_scan1);

  writeToCSV("./pointcloud2.csv", pcl_scan1);
}

void scanCallback1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::PointCloud pcl_scan1;

  scanToPointclouds(msg, pcl_scan1);

  writeToCSV("./pointcloud1.csv", pcl_scan1);
}

void scanCallback0(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::PointCloud pcl_scan1;

  scanToPointclouds(msg, pcl_scan1);

  writeToCSV("./pointcloud0.csv", pcl_scan1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_merging_test");

  ros::NodeHandle n;

  // subscribe to a laser scan msg
  ros::Subscriber scan_sub0 = n.subscribe("sickldmrs/scan0", 10, scanCallback0);
  ros::Subscriber scan_sub1 = n.subscribe("robot_1/scan", 10, scanCallback1);
  ros::Subscriber scan_sub2 = n.subscribe("robot_2/scan", 10, scanCallback2);

  ros::spin();

  return 0;
}
