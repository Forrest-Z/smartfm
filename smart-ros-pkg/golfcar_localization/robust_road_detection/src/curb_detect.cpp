//NUS GolfCart
//firt version  2011-11-30
//last update   2012-01-04
//add DP1_Find function;

#include <curb_detect.h>
#include <cmath>

using namespace std;
using namespace ros;
using namespace tf;

namespace road_detection{
	
	curb_detect::curb_detect():private_nh_("~")
	{
        pub_init_ = false;
        private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
        private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
        
        //scan resolution is critically important when deciding the following parameters;
        private_nh_.param("disFirst_multi",     disFirst_multi_,        4);
		private_nh_.param("disConti_thresh",    disConti_thresh_,       0.4);
		private_nh_.param("rdPtNum_thresh",     rdPtNum_thresh_,        50);
		private_nh_.param("rdWidth_thresh",     rdWidth_thresh_,        2.0);
		private_nh_.param("slope_thresh",       slope_thresh_,          30.0/180.0*3.14159265);
        private_nh_.param("mergeAngle_thresh1", mergeAngle_thresh1_,    10.0/180.0*3.14159265);
        private_nh_.param("mergeAngle_thresh2", mergeAngle_thresh2_,    10.0/180.0*3.14159265);
        private_nh_.param("mergeNum_thresh2_",  mergeNum_thresh2_,      3);
		private_nh_.param("curbLow_thresh",     curbLow_thresh_,        0.05);
		private_nh_.param("curbHigh_thresh",    curbHigh_thresh_,       0.30);
        private_nh_.param("yLength_thresh",     yLength_thresh_,        0.50);
		private_nh_.param("curbAngle_thresh",   curbAngle_thresh_,      20.0/180.0*3.14159265);
        
        private_nh_.param("crossing_thresh",    crossing_thresh_,       10.0);
        
        private_nh_.param("curbFilter_reinit_thresh", curbFilter_assoc_thresh_,     0.3);
        private_nh_.param("curbFilter_reinit_thresh", curbFilter_reinit_thresh_,    0.5);
        
        private_nh_.param("publish_dis_thresh",     publish_dis_thresh_,    0.05);
        private_nh_.param("publish_angle_thresh",   publish_angle_thresh_,  5.0/180.0*3.14159265);
        private_nh_.param("srcUse_thresh",          srcUse_thresh_,         0.1);
        private_nh_.param("srcFilter_thresh",       srcFilter_thresh_,      0.5);
        private_nh_.param("srcFilter_thresh",       filterHeight_thresh_,   1.0);
        
		
        
        looking_forward_distance_ = 5.0;
        
		DP1_pub_            =   nh_.advertise<sensor_msgs::PointCloud>("DP1_pcl", 2);
		DP2_pub_            =   nh_.advertise<sensor_msgs::PointCloud>("DP2_pcl", 2);
		raw_RA_pub_         =   nh_.advertise<sensor_msgs::PointCloud>("raw_RA_pcl", 2);
		FR_RA_pub_          =   nh_.advertise<sensor_msgs::PointCloud>("FR_RA_pcl", 2);
		DP_RA_pub_          =   nh_.advertise<sensor_msgs::PointCloud>("DP_RA_pcl", 2);
        
		road_boundary_pub_  =   nh_.advertise<sensor_msgs::PointCloud>("road_boundary_pcl", 2);				
		
        left_curbline_pub_  =   nh_.advertise<sensor_msgs::PointCloud>("left_curbline_pcl", 2);
		right_curbline_pub_ =   nh_.advertise<sensor_msgs::PointCloud>("right_curbline_pcl", 2);
        left_crossing_pub_  =   nh_.advertise<sensor_msgs::PointCloud>("left_crossing_pcl", 2);
		right_crossing_pub_ =   nh_.advertise<sensor_msgs::PointCloud>("right_crossing_pcl", 2);
        
        hybrid_pub_         =   nh_.advertise<sensor_msgs::PointCloud>("hybrid_pt", 2);
        
	    laser_sub_.subscribe(nh_, "assembled_sick_scan2", 2);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, "odom", 10);
		tf_filter_->registerCallback(boost::bind(&curb_detect::scanCallback, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.05));
        
        odom_sub_ = nh_.subscribe("encoder_odom", 100, &curb_detect::odomCallback, this);
        //this is subscribed to filter possible noise;
        scan_sub_ = nh_.subscribe("scan", 10, &curb_detect::scanSource, this); 
	}
	
	curb_detect::~curb_detect(){}
        
	void curb_detect::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
        laser_scan_ = *scan_in;
        std::string laser_frame_id = scan_in->header.frame_id;
        try{projector_.transformLaserScanToPointCloud(laser_frame_id, *scan_in, laser_cloud_laser_, tf_);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}       
		try{projector_.transformLaserScanToPointCloud(base_frame_id_, *scan_in, laser_cloud_base_, tf_);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
        try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *scan_in, laser_cloud_odom_, tf_);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
        
        for(unsigned int ip=0; ip < laser_cloud_base_.points.size(); ip++ )
        {
            if(laser_cloud_base_.points[ip].y>0)
            {
                looking_forward_distance_ = laser_cloud_base_.points[ip].x;
                ROS_DEBUG("%5f", looking_forward_distance_);
                break;
            }
        }
        
	    curb_detect::scanProcessing();
	}
    
	void curb_detect::scanProcessing()
	{
		ROS_DEBUG("Begin scanProcessing");
		curb_detect::DP_Extraction();
		curb_detect::Road_Selection();
		curb_detect::Curb_Searching();
        curb_detect::Curb_Verification();
		ROS_DEBUG("End scanProcessing");
	}
	
    //------------------------------------------------------------------------------------
	//subfun1: to find all discontinuous points in the filter response; record their ID2s;
    //------------------------------------------------------------------------------------
	void curb_detect::DP_Extraction()
	{
		DP1_IDs_.clear();
		DP1_pcl_.points.clear();
	    DP1_pcl_.header=laser_cloud_laser_.header;
        for(unsigned int ip=0; ip<laser_cloud_laser_.points.size(); ip++)
		{
            if(ip==0) continue;
            unsigned int serial_tempn1 = (unsigned int)(laser_cloud_laser_.channels.front().values[ip-1]);
            unsigned int serial_tempp0 = (unsigned int)(laser_cloud_laser_.channels.front().values[ip]);
            float  range_tempn1 = laser_scan_.ranges[serial_tempn1];
            float  range_tempp0 = laser_scan_.ranges[serial_tempp0];
            if(fabsf(range_tempp0-range_tempn1)>laser_scan_.angle_increment * range_tempp0 * disFirst_multi_ && (serial_tempp0-serial_tempn1)==1)
            {
                //DP1_IDs_ is optional;
                DP1_IDs_.push_back(ip);
                DP1_pcl_.points.push_back(laser_cloud_laser_.points[ip]);
            }
        }
        DP1_pub_.publish(DP1_pcl_);
        
        DP2_IDs_.clear();
		DP2_pcl_.points.clear();
	    DP2_pcl_.header=laser_cloud_laser_.header;
        raw_RA_pcl_.points.clear();
        FR_RA_pcl_.points.clear();
        DP_RA_pcl_.points.clear();
	    raw_RA_pcl_.header = laser_cloud_laser_.header;	    
	    FR_RA_pcl_.header  = laser_cloud_laser_.header;	 
	    DP_RA_pcl_.header  = laser_cloud_laser_.header;	    
	    geometry_msgs::Point32 raw_RA_pt;
	    geometry_msgs::Point32 FR_RA_pt;
	    geometry_msgs::Point32 DP_RA_pt;
        
        //"raw_RA_pcl_";
		for(unsigned int ip=0; ip<laser_cloud_laser_.points.size(); ip++)
		{
            unsigned int    serial_temp = (unsigned int)(laser_cloud_laser_.channels.front().values[ip]);
            float           range_temp  = laser_scan_.ranges[serial_temp];
            //multiplied by 0.1 when shown, just for showing purpose here;
			raw_RA_pt.x = range_temp;
			raw_RA_pt.y = ip * 0.1;   
			raw_RA_pt.z = 0;			
			raw_RA_pcl_.points.push_back(raw_RA_pt);
		}
		
        
		//"FR_RA_pcl_";
		for(unsigned int ip=0; ip<raw_RA_pcl_.points.size(); ip++)            // pay attention to i;
		{
			float response_temp =0;
            
			//*******Higher order (5+1+5) to overcome noise**************
			if(ip<=4||ip>=raw_RA_pcl_.points.size()-5){response_temp = raw_RA_pcl_.points[ip].x;}
			else{
                response_temp = raw_RA_pcl_.points[ip+5].x+raw_RA_pcl_.points[ip+4].x+raw_RA_pcl_.points[ip+3].x+raw_RA_pcl_.points[ip-3].x+raw_RA_pcl_.points[ip-4].x+raw_RA_pcl_.points[ip-5].x;
                response_temp = response_temp-(raw_RA_pcl_.points[ip+2].x+raw_RA_pcl_.points[ip+1].x+raw_RA_pcl_.points[ip].x+raw_RA_pcl_.points[ip].x+raw_RA_pcl_.points[ip-1].x+raw_RA_pcl_.points[ip-2].x);
                }
			FR_RA_pt.x = response_temp;
			FR_RA_pt.y = raw_RA_pcl_.points[ip].y;
			FR_RA_pt.z = 0;
            FR_RA_pcl_.points.push_back(FR_RA_pt);            
		}
		
		
		//find local minima and maxima to determine Discontinous Points(DP);
		for (unsigned int ip=0; ip<FR_RA_pcl_.points.size(); ip++)
		{
			if(4<ip && ip<FR_RA_pcl_.points.size()-5)
			{
				float temp_responsein2 = FR_RA_pcl_.points[ip-2].x;
				float temp_responsein1 = FR_RA_pcl_.points[ip-1].x;
				float temp_responsei   = FR_RA_pcl_.points[ip].x;
				float temp_responseip1 = FR_RA_pcl_.points[ip+1].x;
				float temp_responseip2 = FR_RA_pcl_.points[ip+2].x;
				//try to find local "maxima" and "minima";
				bool  temp_maxima=(temp_responsei>=temp_responsein1)&&(temp_responsei>temp_responsein2)&&(temp_responsei>=temp_responseip1)&&(temp_responsei>temp_responseip2);
				bool  temp_minima=(temp_responsei<=temp_responsein1)&&(temp_responsei<temp_responsein2)&&(temp_responsei<=temp_responseip1)&&(temp_responsei<temp_responseip2);
				bool  maxima2=false;
				bool  minima2=false;

				if(temp_maxima||temp_minima)
				{
                    if(temp_maxima &&(temp_responsei>disConti_thresh_)) {maxima2=true;}
                    else if(temp_minima&&(temp_responsei<-disConti_thresh_)) {minima2=true;}
                    else{}                                     
				}
                
				if(maxima2||minima2)
				{
					DP2_IDs_.push_back(ip);
                    DP_RA_pcl_.points.push_back(FR_RA_pcl_.points[ip]);
					DP2_pcl_.points.push_back(laser_cloud_laser_.points[ip]);
				}
			}
		}

        /*
         * 
        //erase those DPs caused by missing maximum points;
        for(unsigned int is=0; i<DP_IDs_.size(); is++)
        {
            unsigned temp_ip = DP_IDs_[is];
            if(temp_ip<=4||temp_ip>=FR_RA_pcl_.points.size()-5){ROS_ERROR("Bug");return;}
            
            unsigned int serial_ipp0 = (unsigned int)(laser_cloud_.channels.front().values[temp_ip]);
            unsigned int serial_ipp5 = (unsigned int)(laser_cloud_.channels.front().values[temp_ip+5]);
            unsigned int serial_ipn5 = (unsigned int)(laser_cloud_.channels.front().values[temp_ip-5]);
            if((serial_ipp5-serial_ipp0==5)&&(serial_ipn5-serial_ipn5==-5))
            {
                ROS_DEBUG("DP pass");
                DP_RA_pcl_.points.push_back(FR_RA_pcl_.points[temp_ip]);
				DP_pcl_.points.push_back(laser_cloud_.points[temp_ip]);
            }
            else
            {
                DP_IDs_.erase(DP_IDs_.begin()+is);
            }
        }
        */ 
        
		DP2_pub_.publish(DP2_pcl_);
        raw_RA_pub_.publish(raw_RA_pcl_);
        FR_RA_pub_.publish(FR_RA_pcl_);
		DP_RA_pub_.publish(DP_RA_pcl_);
	}

    //-----------------------------------------------------------------
	//subfun2: Extract road_line of the scan, with DP2s from subfun1;
    //-----------------------------------------------------------------
	void curb_detect::Road_Selection()
	{
        road_extracted_ = true;
		road_boundary_pcl_.points.clear();
	    road_boundary_pcl_.header = laser_cloud_base_.header;
        
        //a. extract all possible pairs of road candidates; mainly rely on call of funcion "Road_Cand_Verify";
        std::vector <pair_ids> road_cands;
        pair_ids road_cand_tmp;

        if(DP2_IDs_.size()==0)
        {
            road_cand_tmp.front_id = 0;road_cand_tmp.back_id  = laser_cloud_laser_.points.size()-1;
            if(Road_Cand_Verify(road_cand_tmp)){road_cands.push_back(road_cand_tmp);}
        }
        else
        {
            for(unsigned int is=0; is<DP2_IDs_.size(); is++)
            {
                if(is==0){road_cand_tmp.front_id = 0; road_cand_tmp.back_id = DP2_IDs_[is];}
                else{road_cand_tmp.front_id = DP2_IDs_[is-1]; road_cand_tmp.back_id  = DP2_IDs_[is];}
                if(Road_Cand_Verify(road_cand_tmp)){road_cands.push_back(road_cand_tmp);}
                if(is==DP2_IDs_.size()-1)
                {
                    road_cand_tmp.front_id = DP2_IDs_[is]; road_cand_tmp.back_id  = laser_cloud_laser_.points.size()-1;
                    if(Road_Cand_Verify(road_cand_tmp)){road_cands.push_back(road_cand_tmp);}
                }
            }
        }
        ROS_DEBUG("Road_Cand size %d", road_cands.size());
        
        //b. choose the best road candidate relying on several criteria;
        if(road_cands.size()==0){road_extracted_ = false;}
        else if(road_cands.size()==1){road_surface_ = road_cands[0];}
        else
        {
            //1. choos the pair whose front and back points are placed at two sides of origin in y direction;
            bool two_side = false;
            for(unsigned int ip=0; ip<road_cands.size(); ip++)
            {
                pair_ids road_pair_temp = road_cands[ip];
                geometry_msgs::Point32 front_pt = laser_cloud_base_.points[road_pair_temp.front_id];
                geometry_msgs::Point32 back_pt  = laser_cloud_base_.points[road_pair_temp.back_id];
                if(front_pt.y>0 && back_pt.y<0){road_surface_ = road_pair_temp; two_side = true; break;}
            }
            //2. the line with more points are given high priority;
            if(two_side == false)
            {
                int ptNum = 0;
                for(unsigned int ip=0; ip<road_cands.size(); ip++)
                {
                    pair_ids road_pair_temp = road_cands[ip];
                    int ptNum_tmp = road_pair_temp.back_id-road_pair_temp.front_id;
                    if(ptNum_tmp > ptNum){ptNum = ptNum_tmp;road_surface_ = road_pair_temp;}
                }
            }
        }
        
        /*
		if(road_extracted_)
		{
			road_boundary_pcl_.points.push_back(laser_cloud_base_.points[road_surface_.front_id]);
			road_boundary_pcl_.points.push_back(laser_cloud_base_.points[road_surface_.back_id]);
		}
		road_boundary_pub_.publish(road_boundary_pcl_);
        */ 
	}
    
    //function to decide whether one pair of road candidate is valid;
    bool curb_detect::Road_Cand_Verify(pair_ids &road_cand)
    {
        geometry_msgs::Point32 front_pt = laser_cloud_base_.points[road_cand.front_id];
        geometry_msgs::Point32 back_pt  = laser_cloud_base_.points[road_cand.back_id];
        
        //All following criteria are applied in the base_link coordinate;
        //Criterion 1: distance between two points in y direction should be over rdWidth_thresh;
        //Criterion 2: number of points from front to back should be over rdPtNum_thresh;
        //Criterion 3: line x-y slope < rdSlope_thresh_;
        
        int delt_serial= road_cand.back_id - road_cand.front_id;
        float delt_x= fabsf(back_pt.x-front_pt.x);
		float delt_y=fabsf (back_pt.y-front_pt.y);
        
        bool C1_flag = (delt_y       > rdWidth_thresh_);
		bool C2_flag = (delt_serial  > rdPtNum_thresh_);
		bool C3_flag = (delt_x       < delt_y * tan(slope_thresh_));
        //ROS_DEBUG("dis_x %5f, dis_y %5f, delt_serial %d, delty*slope %5f", delt_x, delt_y, delt_serial, delt_y * tanf(slope_thresh_));
        
        if(C1_flag&&C2_flag&&C3_flag) return true;
        else return false;
    }
    
    
	//----------------------------------------------------------------------------
	//Sub Fun3:from end points of the road, to search curbs nearby;
	//according to the filter response, 2 pairs of points nearby are examined;
    //----------------------------------------------------------------------------
    void curb_detect::Curb_Searching()
    {
        right_break_    = false;
        left_break_     = false;
        right_crossing_ = false;
        left_crossing_  = false;
        right_curb_     = false;
        left_curb_      = false;   
        
        pair_ids right_curb_tmp;
        pair_ids left_curb_tmp;

        if(road_extracted_ == false) return;
        else 
        {
            bool right_go_on_merging = true;
            
            //right curb line;
            while(right_go_on_merging)
            {
                right_curb_tmp.back_id = road_surface_.front_id;
                right_curb_tmp.front_id = 0;
                int right_DP2_serial = -1;
                
                for(unsigned int ip=0; ip <DP2_IDs_.size(); ip++)
                {if(right_curb_tmp.back_id==DP2_IDs_[ip]){right_DP2_serial = ip; break;}}
                //there may be no DP2s at all, and road is extracted from the first point to the last point;
                if(right_DP2_serial == -1){right_crossing_ = true; break;}
                else
                {
                    if(right_DP2_serial != 0)right_curb_tmp.front_id = DP2_IDs_[right_DP2_serial-1];
                    else{ right_curb_tmp.front_id = 0; right_go_on_merging = false; ROS_DEBUG("to the very front");}
                }
                
                if(Merge_Road_Curb(road_surface_, right_curb_tmp)){road_surface_.front_id = right_curb_tmp.front_id;}
                else{right_go_on_merging = false;}
            }
            
            //left curb line;
            bool left_go_on_merging = true;
            while(left_go_on_merging)
            {
                left_curb_tmp.front_id = road_surface_.back_id;
                left_curb_tmp.back_id = 0;
                
                int left_DP2_serial = -1;
                
                for(unsigned int ip=0; ip <DP2_IDs_.size(); ip++)
                {if(left_curb_tmp.front_id == DP2_IDs_[ip]){left_DP2_serial = ip; break;}}
                
                if(left_DP2_serial == -1) {left_crossing_ = true; break;}
                else{
                    if(left_DP2_serial != (int)DP2_IDs_.size()-1) left_curb_tmp.back_id = DP2_IDs_[left_DP2_serial+1];
                    else{ left_curb_tmp.back_id = laser_cloud_laser_.points.size()-1 ; left_go_on_merging = false; ROS_DEBUG("to the very back");}
                }
                
                if(Merge_Road_Curb(road_surface_, left_curb_tmp)){road_surface_.back_id = left_curb_tmp.back_id;}
                else {left_go_on_merging = false;}
            }
            
            if(road_extracted_)
            {
                if(right_crossing_==false && right_break_== false) rightCurb_cand_ = right_curb_tmp;
                if(left_crossing_==false  && left_break_ == false)  leftCurb_cand_ = left_curb_tmp;
            }
        }
        
        //show merged road surface;
        if(road_extracted_)
		{
			road_boundary_pcl_.points.push_back(laser_cloud_base_.points[road_surface_.front_id]);
			road_boundary_pcl_.points.push_back(laser_cloud_base_.points[road_surface_.back_id]);
		}
		road_boundary_pub_.publish(road_boundary_pcl_);
    }
    
    bool curb_detect::Merge_Road_Curb(pair_ids &road_line,  pair_ids &curb_line)
    {
        geometry_msgs::Point32 rdFront_pt = laser_cloud_laser_.points[road_line.front_id];
        geometry_msgs::Point32 rdBack_pt  = laser_cloud_laser_.points[road_line.back_id];
        geometry_msgs::Point32 cbFront_pt = laser_cloud_laser_.points[curb_line.front_id];
        geometry_msgs::Point32 cbBack_pt  = laser_cloud_laser_.points[curb_line.back_id];
        geometry_msgs::Point32 cbFront_pt_base = laser_cloud_base_.points[curb_line.front_id];
        geometry_msgs::Point32 cbBack_pt_base  = laser_cloud_base_.points[curb_line.back_id];
        
        float rd_angle, cb_angle;
        if(rdBack_pt.x-rdFront_pt.x==0.0) rd_angle = (float)M_PI_2;
        else rd_angle = atan2f(rdBack_pt.y-rdFront_pt.y, rdBack_pt.x-rdFront_pt.x);
        if(rd_angle<0) rd_angle = rd_angle + (float)M_PI;
        if(cbBack_pt.x-cbFront_pt.x==0.0) cb_angle = (float)M_PI_2;
        else cb_angle = atan2f(cbBack_pt.y-cbFront_pt.y, cbBack_pt.x-cbFront_pt.x);
        if(rd_angle<0) cb_angle = cb_angle + (float)M_PI;
        
        float delt_angle     = fabsf(cb_angle-rd_angle);
        bool angle_flag1     = delt_angle < mergeAngle_thresh1_;
        bool height_flag     = fabsf(cbFront_pt_base.z-cbBack_pt_base.z) < curbLow_thresh_;
        bool condition1_flag = angle_flag1 && height_flag;
        
        bool angle_flag2     = delt_angle < mergeAngle_thresh2_;
        bool num_flag        = ((int)curb_line.back_id - (int)curb_line.front_id) > mergeNum_thresh2_ ;
        bool condition2_flag = angle_flag2 && num_flag;
        
        if(condition1_flag || condition2_flag) return true;
        else return false;
    }
    
    //--------------------------------------------------------------------------------
	//Sub Fun4:verify curb candidate; generate "curb" or "crossing" as measurements;
    //--------------------------------------------------------------------------------
    void curb_detect::Curb_Verification()
    {
        left_curbline_pcl_.points.clear();
	    left_curbline_pcl_.header=laser_cloud_base_.header;
	    right_curbline_pcl_.points.clear();
	    right_curbline_pcl_.header=laser_cloud_base_.header;
        
        left_crossing_pcl_.points.clear();
	    left_crossing_pcl_.header=laser_cloud_base_.header;
	    right_crossing_pcl_.points.clear();
	    right_crossing_pcl_.header=laser_cloud_base_.header;
        
        hybrid_pcl_.points.clear();
	    hybrid_pcl_.header = laser_cloud_base_.header;
        
        curb_detect::publish_control();
        if(!publish_flag_) return;
        
        //remember to use 3 flags: road_extracted_; right_break_; right_crossing;
        if(road_extracted_)
        {
            if(right_crossing_){}
            else
            {
                if(right_break_)
                { 
                    //only one point and not very frequently, choose not to use considering noise effect;
                    //if first order DP happen, no measurement will be publish for this moment;
                }
                else
                {
                    unsigned int  right_serial_temp = rightCurb_cand_.back_id;
                    geometry_msgs::Point32 right_pt = laser_cloud_base_.points[right_serial_temp];
                    if(right_pt.y<-crossing_thresh_){right_crossing_=true;}
                    else
                    {
                        ROS_DEBUG("verify right curb");
                        if(Curb_Cand_Verify(road_surface_, rightCurb_cand_))
                        {   
                            //for curb, its filter response is local minimum; utilize this knowledge;
                            if(FR_RA_pcl_.points[right_serial_temp].x < 0)
                            {
                                ROS_DEBUG("FR_RA: %5f", FR_RA_pcl_.points[right_serial_temp].x);
                                right_curb_ = true;
                            }
                        }
                        else{}
                    }
                }
            }
            
            if(left_crossing_){}
            else
            {
                if(left_break_)
                {
                }
                else
                {
                    unsigned int  left_serial_temp = leftCurb_cand_.front_id;
                    geometry_msgs::Point32 left_pt = laser_cloud_base_.points[left_serial_temp];
                    if(left_pt.y > crossing_thresh_){left_crossing_ = true;}
                    else
                    {
                        ROS_DEBUG("verify left curb");
                        if(Curb_Cand_Verify(road_surface_, leftCurb_cand_)) 
                        {
                            if(FR_RA_pcl_.points[left_serial_temp].x < 0) 
                            {
                                ROS_DEBUG("FR_RA: %5f", FR_RA_pcl_.points[left_serial_temp].x);
                                left_curb_ = true;
                            }
                        }
                        else{}
                    }
                }
            }
        }
        
        if(right_crossing_)
        {
            geometry_msgs::Point32 rightCrossing_pt; 
            //pay attention to the searching sequence, from left to right;
            for(int ip=(int)laser_cloud_base_.points.size()-1; ip >=0; ip-- )
            {
                if(laser_cloud_base_.points[ip].y < -crossing_thresh_)
                {break;}
                rightCrossing_pt.x = laser_cloud_base_.points[ip].x;
            }
            rightCrossing_pt.y = - crossing_thresh_;
            rightCrossing_pt.z = 0.0;
            right_crossing_pcl_.points.push_back(rightCrossing_pt);
            
            /////////////////////////////////////////////////////////////////////////
            //20120104 update:
            //changed to counter non-verticle fact of crossing beam to base_link x;
            //corresponding to changes in "amcl" part;
            //store beam angle in "rightCrossing_pt.z";
            /////////////////////////////////////////////////////////////////////////
            float delt_x = rightCrossing_pt.x - looking_forward_distance_;
            float delt_y = rightCrossing_pt.y - 0.0;
            float angle_tmp;
            if(delt_x ==0) delt_x =0.01;
            angle_tmp = atan2f(delt_y, delt_x);
            rightCrossing_pt.x = looking_forward_distance_;
            rightCrossing_pt.y = rightCrossing_pt.y;
            rightCrossing_pt.z = angle_tmp;
            hybrid_pcl_.points.push_back(rightCrossing_pt);
            
            //help reset right curb things;
            if(pub_init_) 
            {
                odom_meas_RCurb_old_ = odom_meas_;
                right_curb_line_.clear();
            }
        }
        
        if(left_crossing_)
        {
             geometry_msgs::Point32 leftCrossing_pt; 
             //searching from right to left;
             for(unsigned int ip=0; ip < laser_cloud_base_.points.size(); ip++ )
             {
                 if(laser_cloud_base_.points[ip].y > crossing_thresh_){break;}
                leftCrossing_pt.x = laser_cloud_base_.points[ip].x;
             }
             leftCrossing_pt.y  = crossing_thresh_;
             leftCrossing_pt.z  = 0.0;
             left_crossing_pcl_.points.push_back(leftCrossing_pt);
             
             /////////////////////////////////////////////////////////////////////////
             //20120104 update:
             //changed to counter non-verticle fact of crossing beam to base_link x;
             //corresponding to changes in "amcl" part;
             /////////////////////////////////////////////////////////////////////////
             float delt_x = leftCrossing_pt.x - looking_forward_distance_;
             float delt_y = leftCrossing_pt.y - 0.0;
             if(delt_x ==0) delt_x =0.01;
             float angle_tmp = atan2f(delt_y, delt_x);
             leftCrossing_pt.x = looking_forward_distance_;
             leftCrossing_pt.y = leftCrossing_pt.y;
             leftCrossing_pt.z = angle_tmp;
             hybrid_pcl_.points.push_back(leftCrossing_pt);
             
             //help reset left curb things;
             if(pub_init_) 
             {
                 odom_meas_LCurb_old_ = odom_meas_;
                 left_curb_line_.clear();
             }
        }
        
        if(right_curb_)
        {
             unsigned int right_curb_serial;
             
             int right_DP1 = DP1_Find(rightCurb_cand_);
             if(right_DP1 != -1)
             {
                 ROS_DEBUG("right curb DP1");
                 //use the upper point as curb point when DP1;
                 right_curb_serial = (unsigned int)(right_DP1-1);
             }
             else {right_curb_serial = road_surface_.front_id;}
            
             geometry_msgs::Point32 rightCurb_pt; 
             rightCurb_pt.x = laser_cloud_base_.points[right_curb_serial].x;
             rightCurb_pt.y = laser_cloud_base_.points[right_curb_serial].y;
             rightCurb_pt.z = laser_cloud_base_.points[right_curb_serial].z;
             
             geometry_msgs::Point32 rightCurb_odom_pt = laser_cloud_odom_.points[right_curb_serial];
             unsigned int serial_number = right_curb_serial;
             if(RCurb_Noise_Filtering(rightCurb_odom_pt, serial_number)==false)
             {
                rightCurb_pt.z = 0.0;
                right_curbline_pcl_.points.push_back(rightCurb_pt);
                hybrid_pcl_.points.push_back(rightCurb_pt);
             }
        }
        if(left_curb_)
        {
             unsigned int left_curb_serial;
             
             int left_DP1 = DP1_Find(leftCurb_cand_);
             if(left_DP1 != -1)
             {
                 ROS_DEBUG("left curb DP1");
                 left_curb_serial = (unsigned int)left_DP1;
             }
             else {left_curb_serial = road_surface_.back_id;}
             
             geometry_msgs::Point32 leftCurb_pt; 
             leftCurb_pt.x = laser_cloud_base_.points[left_curb_serial].x;
             leftCurb_pt.y = laser_cloud_base_.points[left_curb_serial].y;
             leftCurb_pt.z = laser_cloud_base_.points[left_curb_serial].z;
             
             geometry_msgs::Point32 leftCurb_odom_pt = laser_cloud_odom_.points[left_curb_serial] ;
             unsigned int serial_number = left_curb_serial;
             if(LCurb_Noise_Filtering(leftCurb_odom_pt, serial_number)==false)
             {
                leftCurb_pt.z = 0.0; 
                left_curbline_pcl_.points.push_back(leftCurb_pt);
                hybrid_pcl_.points.push_back(leftCurb_pt);
             }
        }
        
        if(right_crossing_||left_crossing_||right_curb_||left_curb_) odom_meas_old_ = odom_meas_;
        
        left_crossing_pub_.publish(left_crossing_pcl_); 
		right_crossing_pub_.publish(right_crossing_pcl_);
        left_curbline_pub_.publish(left_curbline_pcl_);
		right_curbline_pub_.publish(right_curbline_pcl_);
        hybrid_pub_.publish(hybrid_pcl_);
    }
    
    int curb_detect::DP1_Find(pair_ids &curb_cand)
    {
        int DP1_serial = -1;
        bool breaker = false;
        
        for(unsigned int ic = curb_cand.front_id; (ic <= curb_cand.back_id) && (!breaker); ic++)
        {
            for(unsigned int is=0; is<DP1_IDs_.size(); is++)
            {
                if(ic == DP1_IDs_[is])
                {
                    DP1_serial = ic;
                    breaker = true;
                    break;
                }
            }
        }
        
        if(DP1_serial == -1) return -1;
        else
        {
            geometry_msgs::Point32 cbBack_pt_base   = laser_cloud_base_.points[DP1_serial];
            geometry_msgs::Point32 cbFront_pt_base  = laser_cloud_base_.points[DP1_serial-1];
            bool height_flag = fabsf(cbFront_pt_base.z-cbBack_pt_base.z) > curbLow_thresh_ && fabsf(cbFront_pt_base.z-cbBack_pt_base.z) < curbHigh_thresh_;
            if(height_flag){return DP1_serial;}
            else return -1;
        }
    }

    
    
    bool curb_detect::Curb_Cand_Verify(pair_ids &road_line, pair_ids &curb_line)
    {
        geometry_msgs::Point32 rdFront_pt = laser_cloud_laser_.points[road_line.front_id];
        geometry_msgs::Point32 rdBack_pt  = laser_cloud_laser_.points[road_line.back_id];
        geometry_msgs::Point32 cbFront_pt = laser_cloud_laser_.points[curb_line.front_id];
        geometry_msgs::Point32 cbBack_pt  = laser_cloud_laser_.points[curb_line.back_id];
        
        geometry_msgs::Point32 cbFront_pt_base = laser_cloud_base_.points[curb_line.front_id];
        geometry_msgs::Point32 cbBack_pt_base  = laser_cloud_base_.points[curb_line.back_id];
        
        float rd_angle, cb_angle;
        if(rdBack_pt.x-rdFront_pt.x==0.0) {rd_angle = (float)M_PI_2;}
        else rd_angle = atan2f(rdBack_pt.y-rdFront_pt.y, rdBack_pt.x-rdFront_pt.x);
        if(rd_angle<0) rd_angle = rd_angle + (float)M_PI;
        if(cbBack_pt.x-cbFront_pt.x == 0.0) {cb_angle = (float)M_PI_2;}
        else cb_angle = atan2f(cbBack_pt.y-cbFront_pt.y, cbBack_pt.x-cbFront_pt.x);
        if(rd_angle<0) cb_angle = cb_angle + (float)M_PI;
        
        ROS_DEBUG("cbBack_pt.x %5f, cbFront_pt.x %5f", cbBack_pt.x, cbFront_pt.x);
        ROS_DEBUG("rd_angle %5f, cb_angle %5f", rd_angle, cb_angle);
        
        float delt_angle = fabsf(cb_angle-rd_angle);
        bool angle_flag  = delt_angle > curbAngle_thresh_;
        bool height_flag = fabsf(cbFront_pt_base.z-cbBack_pt_base.z) > curbLow_thresh_ && fabsf(cbFront_pt_base.z-cbBack_pt_base.z) < curbHigh_thresh_;
        //to counter situation of slowing changing slope;
        bool yLength_flag = fabsf(cbFront_pt_base.y-cbBack_pt_base.y) < yLength_thresh_;
        
        bool conti_flag  = true;
        int serial_front = (int)(laser_cloud_laser_.channels.front().values[curb_line.front_id]);
        int serial_back  = (int)(laser_cloud_laser_.channels.front().values[curb_line.back_id]);
        if(serial_back - serial_front > (int)curb_line.back_id - (int)curb_line.front_id) conti_flag = false;
        
        ROS_DEBUG("road front back: %d, %d; curb front back: %d, %d", road_line.front_id, road_line.back_id, curb_line.front_id, curb_line.back_id );
        ROS_DEBUG("angle diff: %5f, height diff: %5f", delt_angle, fabsf(cbFront_pt_base.z-cbBack_pt_base.z));
        
        if(angle_flag && height_flag && conti_flag &&yLength_flag) return true;
        else return false;
    }
    
    bool curb_detect::LCurb_Noise_Filtering(geometry_msgs::Point32 &meas_pt, unsigned int serial_num)
    {
        //"false" means not filtered, and "true" means filtered.
        if(Curb_Noise_Filtering(odom_meas_LCurb_old_, left_curb_line_, meas_pt, serial_num) ==false) return false;
        else return true;        
    }
    
    bool curb_detect::RCurb_Noise_Filtering(geometry_msgs::Point32 &meas_pt, unsigned int serial_num)
    {
        //"false" means not filtered, and "true" means filtered.
        if(Curb_Noise_Filtering(odom_meas_RCurb_old_, right_curb_line_, meas_pt, serial_num)==false) return false;
        else return true; 
    }
    
    bool curb_detect::Curb_Noise_Filtering(tf::StampedTransform &odom_meas_curb_old, std::vector <geometry_msgs::Point32> &curb_line, geometry_msgs::Point32 &meas_pt, unsigned int serial_num)
    {   
        //--------------1st step: check whether curb point within predicted line; a method similar to EKF ---------------
        //to disable/de-activate this function, just give a small value to "curbFilter_reinit_thresh_";
        
        //a. check if reach reinit_thresh;
        tf::Transform odom_curb_old_new  = odom_meas_curb_old.inverse() * odom_meas_;
        float tx, ty;
        tx = -odom_curb_old_new.getOrigin().y();
        ty =  odom_curb_old_new.getOrigin().x();
        float mov_dis = sqrtf(tx*tx + ty*ty);
        if(mov_dis > curbFilter_reinit_thresh_)
        {
            odom_meas_curb_old = odom_meas_;
            curb_line.clear();
            return false;
        }
        
        //b. check curb point within assoc_thresh;
        if(curb_line.size() < 10)
        {
            curb_line.push_back(meas_pt);
            odom_meas_curb_old = odom_meas_;
            return false;
        }
        else if(curb_line.size() == 10)
        {
            float nearest_dis = Pt_Line_Dis(curb_line, meas_pt);
            ROS_DEBUG("Nearest Distance %5f", nearest_dis);
            ROS_DEBUG("---------curb baselink:%5f, %5f; odom: %5f, %5f", laser_cloud_laser_.points[serial_num].x, laser_cloud_laser_.points[serial_num].y, laser_cloud_odom_.points[serial_num].x, laser_cloud_odom_.points[serial_num].y);
            if(nearest_dis < curbFilter_assoc_thresh_ ) 
            {
                curb_line.erase(curb_line.begin());
                curb_line.push_back(meas_pt);
                odom_meas_curb_old = odom_meas_;
                return false;
            }
            else
            {
                //--------------2nd step: check whether curb point can be eliminated by other sensors;
                ROS_DEBUG("--------------------to be further checked---------------------------------");
                if(Eliminate_noise_other_sensors(serial_num) == false) return false;
                else return true; 
            }
        }
        else
        {ROS_ERROR("Unexpected Error"); return false;}
    }
    
    float curb_detect::Pt_Line_Dis(std::vector <geometry_msgs::Point32> &line, geometry_msgs::Point32 &pt)
    {
        //a. calculate the parameters of the line;
        float x1_temp = line.front().x;
        float y1_temp = line.front().y;
        float x2_temp = line.back().x;
        float y2_temp = line.back().y;
        ROS_DEBUG("x1_temp, y1_temp:  (%5f, %5f); x2_temp, y2_temp: (%5f, %5f); pt.x, pt.y: (%5f, %5f)", x1_temp, y1_temp, x2_temp, y2_temp, pt.x, pt.y);
        float para_A1 = y2_temp-y1_temp;
        float para_B1 = x1_temp-x2_temp;
        float para_C1 = para_A1*x1_temp+para_B1*y1_temp;
        
        //b. calculate the parameters of the perpendicular line;
        float para_A2 = - para_B1;
        float para_B2 =   para_A1;
        float para_C2 =   para_A2*pt.x+ para_B2*pt.y;
        
        //c. calculate the intersection;
        float det =  para_A1*para_B2 - para_A2*para_B1;
        if(det==0) {ROS_DEBUG("not likely happen"); return 0.0;}
        else
        {
            float x_intersection = (para_B2 * para_C1 - para_B1 * para_C2)/det;
            float y_intersection = (para_A1 * para_C2 - para_A2 * para_C1)/det;
            float x_dis = x_intersection - pt.x;
            float y_dis = y_intersection - pt.y; 
            float distance = sqrtf(x_dis*x_dis+y_dis*y_dis);
            return distance;
        }
    }
    
    void curb_detect::scanSource (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        std::string laser_frame_id = scan_in->header.frame_id;
        
        try{projector_.transformLaserScanToPointCloud(laser_frame_id, *scan_in, otherSource_pcl_, tf_);}
		catch (tf::TransformException& e){ROS_DEBUG("Other Souce Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}    
    }
    
    //The function here is open to change in the future; plays no effect here;
    //should transform into baselink... here in laser_frame_id, z (height) is always 0 below threshold...
    bool curb_detect::Eliminate_noise_other_sensors(unsigned int serial)
    {
        double time_dis = otherSource_pcl_.header.stamp.toSec() - laser_cloud_laser_.header.stamp.toSec();
        if(time_dis > srcUse_thresh_) {ROS_ERROR("other source too old"); return false;}
        
        geometry_msgs::Point32  curb_pt_base = laser_cloud_base_.points[serial];
        
        for(unsigned int ip = 0; ip < otherSource_pcl_.points.size(); ip++)
        {
            float disx = curb_pt_base.x - otherSource_pcl_.points[ip].x;
            float disy = curb_pt_base.y - otherSource_pcl_.points[ip].y;
            float dist = sqrtf(disx*disx+disy*disy);
            if(dist < srcFilter_thresh_)
            {
                if(otherSource_pcl_.points[ip].z > filterHeight_thresh_) return true;
            }
        }
        return false;
    }
    
    void curb_detect::publish_control()
    {
        publish_flag_ = false;
        ros::Time meas_time = hybrid_pcl_.header.stamp;
		if (!transformer_.canTransform("base_link","odom", meas_time))
		{ROS_DEBUG("curb_time older than odom message buffer");return;}
		else {ROS_DEBUG("odom OK");}
		transformer_.lookupTransform("odom", "base_link", meas_time, odom_meas_);
		
        if(pub_init_)
		{
            tf::Transform odom_old_new  = odom_meas_old_.inverse() * odom_meas_;
            float tx, ty;
            tx = -odom_old_new.getOrigin().y();
            ty =  odom_old_new.getOrigin().x();
            float mov_dis = sqrtf(tx*tx + ty*ty);
            
            double yaw_dis, ttemp;
            odom_old_new.getBasis().getEulerYPR(yaw_dis, ttemp, ttemp);
            
            
            if(mov_dis > publish_dis_thresh_ || yaw_dis > publish_angle_thresh_)
            {
                publish_flag_ = true;
                odom_meas_old_ = odom_meas_;
            }
        }
        else
        {
            pub_init_ = true;
            //--move to "Curb_Verification", to guarantee odom_meas_old only reset when measurement available--
            //odom_meas_old_ = odom_meas_;
            publish_flag_ = true;
        }
        //if(publish_flag_) hybrid_pub_.publish(hybrid_pcl_);
    }
    
    void curb_detect::odomCallback(const OdomConstPtr& odom)
	{
		//ROS_DEBUG("odom call_back");
		ros::Time odom_time = odom->header.stamp;
		Quaternion q;
		tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
		tf::Transform odom_meas_add = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
		StampedTransform meas_add = StampedTransform(odom_meas_add.inverse(), odom_time, "base_link", "odom");
		transformer_.setTransform( meas_add );
	}
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "curb_detect_node");
	 road_detection::curb_detect curb_detect_node;
     ros::NodeHandle nh;
	 ros::spin();
     return (0);
}
