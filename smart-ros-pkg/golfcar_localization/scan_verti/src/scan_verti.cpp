//NUS GolfCart, 2011-12-11

#include <scan_verti.h>
#include <cmath>

using namespace std;
using namespace ros;
using namespace tf;

namespace road_detection{
	
	scan_verti::scan_verti():private_nh_("~")
	{
        private_nh_.param("verti_laser_frame_id",       verti_laser_frame_id_,      std::string("verti_sick_laser"));
        private_nh_.param("base_frame_id",              base_frame_id_,             std::string("base_link"));
        private_nh_.param("verti_laser_frame_id",       verti_base_frame_id_,       std::string("verti_base_link"));
        private_nh_.param("LowHeight_Thresh",           LowHeight_Thresh_,          0.5);
        private_nh_.param("LowHeight_Thresh",           HighHeight_Thresh_,         4.0);
        private_nh_.param("UseNum_Thresh",              UseNum_Thresh_,             30);
        
               
        laser_sub_.subscribe(nh_, "scan", 10);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_frame_id_, 10);
        tf_filter_->registerCallback(boost::bind(&scan_verti::scanCallback, this, _1));
        tf_filter_->setTolerance(ros::Duration(0.05));
        verti_scan_pub_  =  nh_.advertise<sensor_msgs::LaserScan>("verti_scan", 2);
        imu_subscriber_  =  nh_.subscribe("/ms/imu/data", 10, &scan_verti::imuCallback, this);
            
	}
	scan_verti::~scan_verti(){}
    
    void scan_verti::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
        ROS_DEBUG("Scan CallBack");
        laser_frame_id_ = scan_in->header.frame_id;
        laser_time_     = scan_in->header.stamp;
        raw_scan_ = *scan_in;
        try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *scan_in, raw_laser_pcl_, tf_);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
        
        scan_verti::tfGeneration();
        scan_verti::consVertiScan();
	}
    
    //function: set related tf relationship into "transformer_", which will be used later;
    void scan_verti::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        ROS_DEBUG("imuCallback");
        latest_imu_msg_ = * imu_msg;
    }
    
    void scan_verti::tfGeneration()
    {
        ROS_DEBUG("tfGeneration");
        
        //naming way: from "source" to "target";
        
        //step 1: store transform between vertiBase and base;
        ROS_DEBUG("step 1");
        tf::Transform vertiBase_to_base;
        tf::Vector3 origin;
        origin.setValue (0.0, 0.0, 0.0);
        vertiBase_to_base.setOrigin(origin);
        tf::Quaternion q;
        
        double roll, pitch, yaw;
	    geometry_msgs::Quaternion orientation = latest_imu_msg_.orientation;
	    tf::Quaternion btq(orientation.x, orientation.y, orientation.z, orientation.w);
	    tf::Matrix3x3(btq).getEulerYPR(yaw, pitch, roll);
        
        tf::Quaternion rotation;
        //slightly different from the method in "laser_ortho_projector", which solves the problem through an non-existing "world" coordinate;
        rotation.setEuler (0.0, pitch, roll);
        vertiBase_to_base.setRotation(rotation);
        transformer_.setTransform(StampedTransform(vertiBase_to_base, laser_time_, verti_base_frame_id_, base_frame_id_));
        //step 2: store transform between vertiBase and vertiSick;
        ROS_DEBUG("step 2");
        StampedTransform BasetoSick;
        try{tf_.lookupTransform(base_frame_id_, laser_frame_id_, laser_time_, BasetoSick);}
        catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());}
        transformer_.setTransform(BasetoSick);
        
        //get the translation between vertiBase and sick, while setting the rotation to zero;
        tf::Transform vertiBasetoSick = vertiBase_to_base * BasetoSick;
        tf::Transform vertiBasetovertiSick(vertiBasetoSick);
        tf::Quaternion rotation_tmp;
        rotation_tmp.setEuler (0.0, 0.0, 0.0);
        vertiBasetovertiSick.setRotation (rotation_tmp);
        
        transformer_.setTransform(StampedTransform(vertiBasetovertiSick, laser_time_, verti_base_frame_id_, verti_laser_frame_id_));
        
        //parallelly publish useful tf as by-product;
        //in 2D-plane application, we usually take base as vertiBase, nominally;
        tf::StampedTransform nominalBasetovertiSick(vertiBasetovertiSick, laser_time_,  base_frame_id_, verti_laser_frame_id_);
        tf_broadcaster_.sendTransform (nominalBasetovertiSick);
    }
    
    void scan_verti::consVertiScan()
    {
        int lowPtNum                 =   0;
        verti_scan_.header           =   raw_scan_.header;
        verti_scan_.header.frame_id  =   verti_laser_frame_id_;
        verti_scan_.angle_min        =   raw_scan_.angle_min;
        verti_scan_.angle_max        =   raw_scan_.angle_max;
        verti_scan_.time_increment   =   raw_scan_.time_increment;
        verti_scan_.angle_increment  =   raw_scan_.angle_increment;
        verti_scan_.scan_time        =   raw_scan_.scan_time;
        verti_scan_.range_min        =   raw_scan_.range_min;
        verti_scan_.range_max        =   raw_scan_.range_max;
        verti_scan_.ranges.clear();
        
        geometry_msgs::PointStamped      laser_pt;
        geometry_msgs::PointStamped      verti_laser_pt;
        geometry_msgs::PointStamped      vertiBase_laser_pt;
        
        laser_pt.header.stamp        =   raw_scan_.header.stamp;
        laser_pt.header.frame_id     =   laser_frame_id_;
        
        for(unsigned int ip=0; ip<raw_laser_pcl_.points.size(); ip++)
        {
            tf::Stamped <tf::Vector3> LaserPt, vertiLaserPt, vertiBaseLaserPt;
            laser_pt.point.x = (float)raw_laser_pcl_.points[ip].x;
            laser_pt.point.y = (float)raw_laser_pcl_.points[ip].y;
            laser_pt.point.z = (float)raw_laser_pcl_.points[ip].z;            
            pointStampedMsgToTF(laser_pt, LaserPt);
            
            //measurement verification, eliminate those points too low or too hight;
            if (!transformer_.canTransform(laser_frame_id_, verti_base_frame_id_, laser_pt.header.stamp))
            {
                ROS_ERROR("tf not found between laser_frame_id_ and verti_base_frame_id_");
                return ;
            }
            transformer_.transformPoint(verti_base_frame_id_, LaserPt, vertiBaseLaserPt);
            pointStampedTFToMsg (vertiBaseLaserPt, vertiBase_laser_pt);
            if(vertiBase_laser_pt.point.z < LowHeight_Thresh_ ||vertiBase_laser_pt.point.z > HighHeight_Thresh_){lowPtNum++;continue;}
            
            if (!transformer_.canTransform(laser_frame_id_, verti_laser_frame_id_, laser_pt.header.stamp))
            {
                ROS_ERROR("tf not found between laser_frame_id_ and verti_laser_frame_id_");
                return ;
            }
            transformer_.transformPoint(verti_laser_frame_id_, LaserPt, vertiLaserPt);
            pointStampedTFToMsg (vertiLaserPt, verti_laser_pt);
            addPttoScan(verti_laser_pt, verti_scan_);
        }
        ROS_DEBUG("lowPtNum %d", lowPtNum);
        
        if(lowPtNum > UseNum_Thresh_)
        {
            ROS_DEBUG("Scan Hit Ground");
            verti_scan_.ranges.clear();
        }
        
        verti_scan_pub_.publish(verti_scan_);
    }
    
    void scan_verti::addPttoScan(geometry_msgs::PointStamped &verti_pt, sensor_msgs::LaserScan &verti_scan)
    {
        float bigger_than_max_range = verti_scan.range_max+1.0;
        float angle, range;
        range = (float)sqrt(verti_pt.point.x*verti_pt.point.x+verti_pt.point.y*verti_pt.point.y);
        if(range == 0.0) return;
        
        if(verti_pt.point.x == 0.0)
        {
            if(verti_pt.point.y > 0.0) angle = (float)M_PI_2; 
            else angle = -(float)M_PI_2;
        }
        else
        {
            angle = atan2f((float)verti_pt.point.y, (float)verti_pt.point.x);
        }
        
        ROS_DEBUG("x, y, angle: %5f, %5f, %5f", verti_pt.point.x, verti_pt.point.y, angle);
        if(angle < verti_scan.angle_min ||angle > verti_scan.angle_max) return;
        float delt_angle = angle - verti_scan.angle_min;
        unsigned int serial_tmp = (unsigned int)floor(delt_angle/verti_scan.angle_increment);
        
        ROS_DEBUG("delt_angle %5f, verti_scan.angle_increment %5f, serial_tmp: %d", delt_angle, verti_scan.angle_increment, serial_tmp);
        
        unsigned int back_serial, delt;
        if(verti_scan.ranges.size()==0)
        {
            delt = serial_tmp;
            for(unsigned int i=0; i<delt; i++) verti_scan.ranges.push_back(bigger_than_max_range);
        }
        else 
        {
            back_serial = verti_scan.ranges.size()-1;
            if(back_serial>serial_tmp) {ROS_ERROR("Unexpected ERROR!!!!!!");return;}
            else if(back_serial == serial_tmp){ROS_DEBUG("two points fall into one beam");return;}
            
            delt = serial_tmp - back_serial;
            for(unsigned int i=1; i<delt; i++)
            {verti_scan.ranges.push_back(bigger_than_max_range);}
        }
        
        verti_scan.ranges.push_back(range);
    }
    
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "scan_verti_node");
	 road_detection::scan_verti* scan_verti_node;
	 scan_verti_node = new road_detection::scan_verti;
	 ros::spin();
}

