//NUS GolfCart, 2011-12-15

#include <scan_ldmrs_assemble.h>
#include <cmath>

using namespace std;
using namespace ros;

namespace road_detection{
	
	scan_ldmrs::scan_ldmrs():private_nh_("~")
	{
        scan1_income_   =   false;
        scan2_income_   =   false;
        
        private_nh_.param("ldmrs_single_id",            ldmrs_single_id_,            std::string("ldmrsAssemb"));
        private_nh_.param("assemble_time_thresh",  assemble_time_thresh_,  0.05);
        scan_sub1_      =   nh_.subscribe("sickldmrs/scan1", 10, &scan_ldmrs::ldmrs_callBack1, this);
        scan_sub2_      =   nh_.subscribe("sickldmrs/scan2", 10, &scan_ldmrs::ldmrs_callBack2, this);        
        scan_pub_       =   nh_.advertise<sensor_msgs::LaserScan>("sickldmrs/assembled", 2);    
            
	}
	scan_ldmrs::~scan_ldmrs(){}
    
    void scan_ldmrs::ldmrs_callBack1 (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
        //ROS_INFO("ldmrs/scan1");
        scan1_ = *scan_in;
        scan1_income_ = true;
        scan_ldmrs::ldmrs_assemble();
	}
    
    void scan_ldmrs::ldmrs_callBack2 (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
        //ROS_INFO("ldmrs/scan2");
        scan2_ = *scan_in;
        scan2_income_ = true;
        scan_ldmrs::ldmrs_assemble();
	}
    
    void scan_ldmrs::ldmrs_assemble()
    {
        if(scan1_income_&&scan2_income_)
        {
            ROS_INFO("ldmrs/assemble");
            scan1_income_ =  false;
            scan2_income_ =  false;
            
            double time_dis = scan1_.header.stamp.toSec() - scan2_.header.stamp.toSec();            
            if( time_dis > assemble_time_thresh_)
            {
                ROS_WARN("time between two ldmrs scans too long");
                return;
            }
            
            assembled_scan_.header          =   scan1_.header;
            assembled_scan_.header.frame_id =   ldmrs_single_id_;
            assembled_scan_.angle_min       =   scan1_.angle_min;
            assembled_scan_.angle_max       =   scan1_.angle_max;
            assembled_scan_.time_increment  =   scan1_.time_increment*2.0;
            assembled_scan_.angle_increment =   scan1_.angle_increment;
            assembled_scan_.scan_time       =   scan1_.scan_time*2.0;
            assembled_scan_.range_min       =   scan1_.range_min;
            assembled_scan_.range_max       =   scan1_.range_max;
            assembled_scan_.ranges.clear();
            
            for(unsigned int ib=0; ib< scan1_.ranges.size(); ib++)
            {
                float range_tmp;
                if((int)ib%2==1)
                {
                    range_tmp = scan1_.ranges[ib];
                    assembled_scan_.ranges.push_back(range_tmp);
                }
                else
                {
                    range_tmp = scan2_.ranges[ib];
                    assembled_scan_.ranges.push_back(range_tmp);
                }
            }
            scan_pub_.publish(assembled_scan_);
        }
        else return;
    }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "scan_ldmrs_node");
	 road_detection::scan_ldmrs scan_ldmrs_node;
	 ros::spin();
}

