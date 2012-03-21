//NUS GolfCart, 2011-12-15
//last modified 2011-12-19, add MAX_RANGE;

#include <scan_ldmrs_single.h>
#include <cmath>

using namespace std;
using namespace ros;

#define MAX_RANGE 15.0

namespace road_detection{
	
	scan_ldmrs::scan_ldmrs():private_nh_("~")
	{
        private_nh_.param("ldmrs_single_id",            ldmrs_single_id_,            std::string("ldmrsSingle"));
        scan_sub1_      =   nh_.subscribe("sickldmrs/scan1", 10, &scan_ldmrs::ldmrs_callBack1, this);   
        scan_pub_       =   nh_.advertise<sensor_msgs::LaserScan>("sickldmrs/single", 2);    
            
	}
	scan_ldmrs::~scan_ldmrs(){}
    
    void scan_ldmrs::ldmrs_callBack1 (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
        scan1_ = *scan_in;
        scan_ldmrs::ldmrs_single();
	}
    
    void scan_ldmrs::ldmrs_single()
    {      
        single_scan_.header          =   scan1_.header;
        single_scan_.header.frame_id =   ldmrs_single_id_;
        single_scan_.angle_min       =   scan1_.angle_min;
        single_scan_.angle_max       =   scan1_.angle_max;
        single_scan_.time_increment  =   scan1_.time_increment*2.0;
        single_scan_.angle_increment =   scan1_.angle_increment*2.0;
        single_scan_.scan_time       =   scan1_.scan_time*2.0;
        single_scan_.range_min       =   scan1_.range_min;

        single_scan_.range_max    =   MAX_RANGE;
        single_scan_.ranges.clear();
        
        for(unsigned int ib=0; ib< scan1_.ranges.size(); ib++)
        {
            float range_tmp;
            if((int)ib%2==1)
            {
                range_tmp = scan1_.ranges[ib];
                single_scan_.ranges.push_back(range_tmp);
            }
        }
        scan_pub_.publish(single_scan_);
    }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "single_ldmrs_node");
	 road_detection::scan_ldmrs single_ldmrs_node;
	 ros::spin();
}

