//NUS GolfCart, 2011-12-06

#include <scan_assemble.h>
#include <cmath>

using namespace std;
using namespace ros;
using namespace tf;

namespace road_detection{
	
	scan_assemble::scan_assemble()
	{
        ros::NodeHandle private_nh_;
        laser_income_   =   0;
        private_nh_.param("assemble_time_thresh",  assemble_time_thresh_,  0.05);
        
		ros::NodeHandle nh;
        scan_sub_       =   nh.subscribe("sick_scan2", 10, &scan_assemble::assemble_scans, this);        
        scan_pub_       =   nh.advertise<sensor_msgs::LaserScan>("assembled_sick_scan2", 2);    
	}
	
	scan_assemble::~scan_assemble(){}

	void scan_assemble::assemble_scans (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
        if(laser_income_==0)
        {
            laser_scan1_ = * scan_in;
            laser_income_ =1;
        }
        else if(laser_income_ ==1)
        {
            laser_scan2_ = * scan_in;
            double time_dis = laser_scan2_.header.stamp.toSec() - laser_scan1_.header.stamp.toSec();
            //ROS_INFO("time_dis %5f", time_dis);
            
            if( time_dis > assemble_time_thresh_)
            {
                ROS_WARN("time between two scans is too long");
                laser_scan1_  = * scan_in;
                laser_income_ = 1;
                return;
            }
            //make sure that "laser_scan2_" is always with bigger angle_max;
            if(laser_scan2_.angle_max < laser_scan1_.angle_max)
            {
                sensor_msgs::LaserScan scan_temp = laser_scan2_;
                laser_scan2_ = laser_scan1_;
                laser_scan1_ = scan_temp;
            }
            
            assembled_scan_.header = laser_scan2_.header; 
            assembled_scan_.angle_min = laser_scan2_.angle_min;
            assembled_scan_.angle_max = laser_scan2_.angle_max;
            assembled_scan_.time_increment  = laser_scan2_.time_increment*2.0;
            assembled_scan_.angle_increment = laser_scan2_.angle_increment/2.0;
            assembled_scan_.scan_time = laser_scan2_.scan_time*2.0;
            assembled_scan_.range_min = laser_scan2_.range_min;
            assembled_scan_.range_max = laser_scan2_.range_max;
            assembled_scan_.ranges.clear();   
            
            for(unsigned int i =0; i < laser_scan1_.ranges.size(); i++)
            {
                assembled_scan_.ranges.push_back(laser_scan2_.ranges[i]);
                assembled_scan_.ranges.push_back(laser_scan1_.ranges[i]);
            }
            assembled_scan_.ranges.push_back(laser_scan2_.ranges.back()); 
            scan_pub_.publish(assembled_scan_);
            laser_income_ = 0;
        } 
	}
    
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "scan_assemble_node");
	 road_detection::scan_assemble* scan_assemble_node;
	 scan_assemble_node = new road_detection::scan_assemble;
	 ros::spin();
}

