//NUS Golf Cart, 2011-06-01;
//////////////////////////////////Brief Description////////////////////////////////////////////////////
//"road_detect" to detect two curb lines from sick laser scan;
//input: 	topic "sick_scan", data type "sensor_msgs::LaserScan";
//output: 	topic  "left_curbline_pub_" and "right_curbline_pub_"; data type "sensor_msgs::PointCloud";
//other topics are mainly for debugging and visualization purposes;
///////////////////////////////////////////////////////////////////////////////////////////////////////

#include <road_detect.h>
#include <cmath>

namespace road_detection{
	
	road_detect::road_detect()
	{
		//------------parameters of function "CurbEtraction"----------------
		//"max_min_tresh_" should be carefully learned from filter response, which may change according to different laser sensors;
		private_nh_.param("max_min_tresh_", max_min_tresh_, 0.6);
		
		//------------parameters of function "RoadSelection"----------------
		private_nh_.param("ptNum_tresh_", ptNum_tresh_, 40);
		private_nh_.param("rdLength_tresh_", rdLength_tresh_, 2.0);
		private_nh_.param("slopeSin_tresh_", slopeSin_tresh_, 0.5);
		
		//------------parameters of function "CurbSideLine"----------------
		private_nh_.param("curbLow_tresh_", curbLow_tresh_, 0.05);
		private_nh_.param("curbheigh_tresh_", curbheigh_tresh_, 4.0);
		
		// "curbTan_tresh_" need to be adjusted to be a bigger value;
		//a small value may ignore some points, but will help to reduce noise as for pure "curb_track"; 
		//a big value get more points when relying on raw points for "curb_amcl"; but yield lots of noise in some cases;
		private_nh_.param("curbTan_tresh_", curbTan_tresh_, 5.0);
		
		ros::NodeHandle nh;
		
		laser_pub_ = nh.advertise<sensor_msgs::PointCloud>("laser_cloud_", 2);
		//For debugging purposes (1), show in rviz "show_laser_cloud_", "show_filter_response_", "show_boundary_candidates_";
		curb_pub_ = nh.advertise<sensor_msgs::PointCloud>("curb_candidates_", 2);
		boundary_pub_ = nh.advertise<sensor_msgs::PointCloud>("road_boundary_", 2);
		show_laser_pub_ = nh.advertise<sensor_msgs::PointCloud>("show_laser_cloud_", 2);
		show_response_pub_ = nh.advertise<sensor_msgs::PointCloud>("show_filter_response_", 2);		
		show_curb_pub_ = nh.advertise<sensor_msgs::PointCloud>("show_curb_candidates_", 2);		
		
		left_curbline_pub_ = nh.advertise<sensor_msgs::PointCloud>("left_curbline_pub_", 2);
		right_curbline_pub_ = nh.advertise<sensor_msgs::PointCloud>("right_curbline_pub_", 2);

	    laser_sub_.subscribe(nh, "sick_scan", 2);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, "odom", 10);
		tf_filter_->registerCallback(boost::bind(&road_detect::scanCallback, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.05));

		begin_end_PID_[0]=0;
		begin_end_PID_[1]=0;


		curbPCIDLeft_pub_ = nh.advertise<road_detection::curbPointCloudID>("svm_leftCurbPoint_id",2);
		curbPCIDRight_pub_ = nh.advertise<road_detection::curbPointCloudID>("svm_rightCurbPoint_id",2);

	}
	
	road_detect::~road_detect()
	{

	}
	void road_detect::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
		//LaserScan to PointCloud, stored in laser_cloud_;
		ROS_DEBUG("laser callback begin");
		std::string target_frame = scan_in->header.frame_id;
		try
		{
			projector_.transformLaserScanToPointCloud(target_frame, *scan_in, laser_cloud_, tf_);
		}
		catch (tf::TransformException& e)
		{
			ROS_DEBUG("Wrong!!!!!!!!!!!!!");
			std::cout << e.what();
			return;
		}
		//For the debugging purpose;
		laser_pub_.publish(laser_cloud_);             
	    road_detect::ProcessSingleScan();
	}
	
	void road_detect::ProcessSingleScan()
	{
		ROS_DEBUG("Begin ProcessSingleScan");
		road_detect::CurbEtraction();
		road_detect::RoadSelection();
		road_detect::CurbSideLine();
		ROS_DEBUG("End ProcessSingleScan");
	}
	
	//Sub Fun1: to find all discontinuous points in the filter response; 
	//they keep their own point IDs;
	void road_detect::CurbEtraction()
	{
		//curb_candidate_PIDs_ stored all discontinuous points, which are possible candidates;
		curb_candidate_PIDs_.clear();
		curb_candidates_.points.clear();
	    curb_candidates_.header=laser_cloud_.header;
	    
	    //For debugging purposes(1)
		show_laser_cloud_.points.clear();
	    show_laser_cloud_.header=laser_cloud_.header;	    
		show_filter_response_.points.clear();
	    show_filter_response_.header=laser_cloud_.header;	 
	    show_curb_candidates_.points.clear();
	    show_curb_candidates_.header=laser_cloud_.header;	    
   
	    geometry_msgs::Point32 temp_show_laser;
	    geometry_msgs::Point32 temp_show_filter;
	    geometry_msgs::Point32 temp_show_boundary;
	    
		for(unsigned int i=0; i<laser_cloud_.points.size(); i++)
		{
			float temp_x=laser_cloud_.points[i].x;
			float temp_y=laser_cloud_.points[i].y;
			float temp_distance=sqrt(float(temp_x*temp_x+temp_y*temp_y));
			
			temp_show_laser.x=temp_distance;
			temp_show_laser.y=0.5*i*0.1;   //0.5 is the resolution of angle; multiple 0.1 when shown; just for showing purpose here;
			temp_show_laser.z=0;			
			show_laser_cloud_.points.push_back(temp_show_laser);
		}
		show_laser_pub_.publish(show_laser_cloud_);
		
		//get "filter_response_";
		for(unsigned int i=0; i<laser_cloud_.points.size();i++)            // pay attention to i;
		{
			float temp_response=0;
			
			//*******Higher order (5+1+5) to overcome noise**************
			if(i<=4||i>=laser_cloud_.points.size()-5){temp_response=show_laser_cloud_.points[i].y;}
			else
			{
			 ////the method proposed by CMU not used;
			 //temp_response=GuassianDifferentialFilter[0]*show_laser_cloud_.points[i-3].x+GuassianDifferentialFilter[1]*show_laser_cloud_.points[i-2].x+GuassianDifferentialFilter[2]*show_laser_cloud_.points[i-1].x;
			 //temp_response=temp_response+GuassianDifferentialFilter[4]*show_laser_cloud_.points[i+1].x+GuassianDifferentialFilter[5]*show_laser_cloud_.points[i+2].x+GuassianDifferentialFilter[6]*show_laser_cloud_.points[i+3].x;
			 
			 temp_response=show_laser_cloud_.points[i+5].x+show_laser_cloud_.points[i+4].x+show_laser_cloud_.points[i+3].x+show_laser_cloud_.points[i-3].x+show_laser_cloud_.points[i-4].x+show_laser_cloud_.points[i-5].x;
			 temp_response=temp_response-(show_laser_cloud_.points[i+2].x+show_laser_cloud_.points[i+1].x+show_laser_cloud_.points[i].x+show_laser_cloud_.points[i].x+show_laser_cloud_.points[i-1].x+show_laser_cloud_.points[i-2].x);
			}
			
			temp_show_laser.x=temp_response;
			temp_show_laser.y=show_laser_cloud_.points[i].y;
			temp_show_laser.z=0;
            show_filter_response_.points.push_back(temp_show_laser);            
		}
		show_response_pub_.publish(show_filter_response_);
		
		//find local minima and maxima; determine curb candidates;
		for (unsigned int i=0; i<show_filter_response_.points.size();i++)
		{
			if(4<i&&i<show_filter_response_.points.size()-5)
			{
				float temp_responsein2=show_filter_response_.points[i-2].x;
				float temp_responsein1=show_filter_response_.points[i-1].x;
				float temp_responsei=show_filter_response_.points[i].x;
				float temp_responseip1=show_filter_response_.points[i+1].x;
				float temp_responseip2=show_filter_response_.points[i+2].x;
				
				//try to find local "maxima" and "minima";
				bool  temp_maxima=(temp_responsei>temp_responsein1)&&(temp_responsei>temp_responsein2)&&(temp_responsei>temp_responseip1)&&(temp_responsei>temp_responseip2);
				bool  temp_minima=(temp_responsei<temp_responsein1)&&(temp_responsei<temp_responsein2)&&(temp_responsei<temp_responseip1)&&(temp_responsei<temp_responseip2);
				bool  maxima2=false;
				bool  minima2=false;
				
				if(temp_maxima||temp_minima)
				{
					float temp_substraction=0;

					for(int j=-5; j<6;j++)
					{
						temp_substraction=show_filter_response_.points[i].x-show_filter_response_.points[i+j].x;
						if(temp_maxima&&(temp_substraction>max_min_tresh_)) {maxima2=true;}
						else if(temp_minima&&(temp_substraction< -max_min_tresh_)) {minima2=true;}
						else{}
					}	   						   						
				}
				else{}
				
				if(maxima2||minima2)
				{
					curb_candidate_PIDs_.push_back(i);
					show_curb_candidates_.points.push_back(show_filter_response_.points[i]);    //show "curb_candidates" on "filter_response" 
					curb_candidates_.points.push_back(laser_cloud_.points[i]);
				}
				else{}
			}
			else {}
		}
		//ROS_DEBUG("1------Curb candidate numbers %d", curb_candidate_PIDs_.size());
		curb_pub_.publish(curb_candidates_);
		show_curb_pub_.publish(show_curb_candidates_);
	}
	
	//Sub Fun2:Extract two end points of the road plane, with the discontinous points from last function;
	void road_detect::RoadSelection()
	{
		road_boundary_.points.clear();
	    road_boundary_.header=laser_cloud_.header;
	    
	    //store pairs of points, with each pair of points corresponding to one road segment candidate.
	    std::vector<unsigned int> temp_C1C2;     	    
		//ROS_DEBUG("2------Curb candidate numbers %d",curb_candidate_PIDs_.size());
		
		//pay attention to "i<(curb_candidate_PIDs_.size()-1)";
		for(unsigned int i=0; i<(curb_candidate_PIDs_.size()-1);i++)     
		{
			//ROS_DEBUG("2-------curb candidate PIDs %d---------",i);
			unsigned int line_begin_PID=curb_candidate_PIDs_[i];
			unsigned int line_end_PID=curb_candidate_PIDs_[i+1];
						
			float temp_x0=laser_cloud_.points[line_begin_PID].x;
			float temp_y0=laser_cloud_.points[line_begin_PID].y;
			float temp_x1=laser_cloud_.points[line_end_PID].x;
			float temp_y1=laser_cloud_.points[line_end_PID].y;
			
			//Criterion 1: numbers threshold 30; distances threshold 2m;
			float temp_line_distance=sqrt(float((temp_x0-temp_x1)*(temp_x0-temp_x1)+(temp_y0-temp_y1)*(temp_y0-temp_y1)));
			
			//ROS_DEBUG("begin pid %d; end pid %d", line_begin_PID,line_end_PID);
			//ROS_DEBUG("distance %5f", temp_line_distance);
			int tempint= line_end_PID-line_begin_PID;
			bool C1_flag=(tempint > ptNum_tresh_)&&(temp_line_distance>rdLength_tresh_);
			
			//Criterion 2: slope <30 degree;
			float delt_line_x=temp_x1-temp_x0;
			float delt_line_y=temp_y1-temp_y0;
			//ROS_DEBUG("delt x %3f; delt y %3f", delt_line_x,delt_line_y);
			bool C2_flag=(-delt_line_y*slopeSin_tresh_<delt_line_x && delt_line_x<delt_line_y*slopeSin_tresh_);
						
			//if(C1_flag==false){ROS_DEBUG("C1 fail");}
			//if(C2_flag==false){ROS_DEBUG("C2 fail");}
			
			if(C1_flag&&C2_flag)
			{
				ROS_DEBUG("C1&&C2 Pass!");
				temp_C1C2.push_back(line_begin_PID);
				temp_C1C2.push_back(line_end_PID);
			}	
		}
		
		//determine  "begin_end_PID_[2]";
		bool temp_curb_exist=true;
		if(temp_C1C2.size()==0){ROS_DEBUG("NO Curbs Extracted");temp_curb_exist=false;}  //impossible in general cases;
		else if(temp_C1C2.size()==2)
		{
		 begin_end_PID_[0]=temp_C1C2[0];
		 begin_end_PID_[1]=temp_C1C2[1];
		}
		else if(temp_C1C2.size()>2)
		{
			//Criterion 3: the origin is located near begin and end points in y axis; 
			//Only used when multi choices exist; 2 steps;
						
			temp_curb_exist=false;
			for(unsigned int i=0; i<temp_C1C2.size();i=i+2)
			{
				unsigned int temp_begin=temp_C1C2[i];
				unsigned int temp_end=temp_C1C2[i+1];
				float temp_y0=laser_cloud_.points[temp_begin].y;
				float temp_y1=laser_cloud_.points[temp_end].y;
				if(temp_y0<0&&temp_y1>0)
				{
					temp_curb_exist=true;
					begin_end_PID_[0]=temp_C1C2[i];
		            begin_end_PID_[1]=temp_C1C2[i+1];
				}
			}
			
			if(temp_curb_exist==false)
			{
				temp_curb_exist=true;
				float orgin_nearest=100.0;
				for(unsigned int i=0; i<temp_C1C2.size();i=i+2)
				{
					unsigned int temp_begin=temp_C1C2[i];
					unsigned int temp_end=temp_C1C2[i+1];
				
					float temp_y0=laser_cloud_.points[temp_begin].y;
					float temp_y1=laser_cloud_.points[temp_end].y;
					float temp_origin_dis=temp_y0+temp_y1;
					if(temp_origin_dis<0){temp_origin_dis=-temp_origin_dis;}
				
					if(temp_origin_dis<orgin_nearest)
					{
					orgin_nearest=temp_origin_dis;
					begin_end_PID_[0]=temp_C1C2[i];
					begin_end_PID_[1]=temp_C1C2[i+1];
					}
				}
			}	
		}
		else {}	
		
		//publish the "pair" of curbs;
		if(temp_curb_exist)
		{
			ROS_DEBUG("Curbs Extracted");
			unsigned int temp_curb_begin=begin_end_PID_[0];
			unsigned int temp_curb_end=begin_end_PID_[1];
			road_boundary_.points.push_back(laser_cloud_.points[temp_curb_begin]);
			road_boundary_.points.push_back(laser_cloud_.points[temp_curb_end]);
		}
						
		boundary_pub_.publish(road_boundary_);
	}
	
	//Sub Fun3:from end points of the road, to find curbs nearby;
	//according to the filter response, 2 pairs of points nearby are examined;
	void road_detect::CurbSideLine()
	{
		left_curb_line_.points.clear();
	    left_curb_line_.header=laser_cloud_.header;
	    right_curb_line_.points.clear();
	    right_curb_line_.header=laser_cloud_.header;
        
        //find PID for each points.	    
        //pay attention here,left_line_begin_PID is bigger than left_line_end_PID;
        unsigned int right_line_begin_PID = begin_end_PID_[0]; 
		unsigned int right_line_end_PID=0;
		unsigned int right_backup_end=0;
		unsigned int left_line_begin_PID  = begin_end_PID_[1];   
		unsigned int left_line_end_PID=0;
		unsigned int left_backup_end=0;    
		
		for(unsigned int i=0; i<curb_candidate_PIDs_.size();i++)
		{
			if(curb_candidate_PIDs_[i] == right_line_begin_PID)
			{
				if(i>0) //make sure that 
				{unsigned int temp_serial1_in_candidates=i-1;
				 right_line_end_PID=curb_candidate_PIDs_[temp_serial1_in_candidates];}
				if(i>1)
				{unsigned int temp_serial1_in_candidates=i-2;
				 right_backup_end=curb_candidate_PIDs_[temp_serial1_in_candidates];}
			}
			
			if(curb_candidate_PIDs_[i] == left_line_begin_PID)
			{
				if(i<(curb_candidate_PIDs_.size()-1))
				{unsigned int temp_serial2_in_candidates=i+1;
				 left_line_end_PID=curb_candidate_PIDs_[temp_serial2_in_candidates];}
				if(i<(curb_candidate_PIDs_.size()-2))
				{unsigned int temp_serial2_in_candidates=i+2;
				 left_backup_end=curb_candidate_PIDs_[temp_serial2_in_candidates];}
			}
			
		}
		
		bool temp_left_flag=false;
		bool temp_left_backup_flag=false;
		bool temp_right_flag=false;
		bool temp_right_backup_flag=false;
		
		//for left_line;
		if((left_line_begin_PID!=0)&&(left_line_end_PID!=0))
		{
			geometry_msgs::Point32 temp_left_begin_Point=laser_cloud_.points[left_line_begin_PID];
			geometry_msgs::Point32 temp_left_end_Point  =laser_cloud_.points[left_line_end_PID];
			
			geometry_msgs::PointStamped left_begin_Point_base;
			geometry_msgs::PointStamped left_end_Point_base;
			tfSicktoBaselink(temp_left_begin_Point,left_begin_Point_base);
			tfSicktoBaselink(temp_left_end_Point,left_end_Point_base);
			
			//criterion 4: curb is higher than 0.05, lower than 0.5 meter;
			float temp_delt_z=left_begin_Point_base.point.z-left_end_Point_base.point.z;
			ROS_DEBUG("left1: %3f",temp_delt_z);
			bool temp_left_height=(temp_delt_z>curbLow_tresh_ && temp_delt_z< curbheigh_tresh_)||(temp_delt_z<-curbLow_tresh_ && temp_delt_z>-curbheigh_tresh_);
			
			//criterion 5: the curb should be somewhat orthorgnal to the road, say, 45 degree;
			float temp_delt_x=temp_left_begin_Point.x-temp_left_end_Point.x;
			float temp_delt_y=temp_left_begin_Point.y-temp_left_end_Point.y;
			bool temp_left_slope =(temp_delt_x>=0)&&((temp_delt_y>0&&curbTan_tresh_*temp_delt_x>temp_delt_y)||(temp_delt_y<0&&curbTan_tresh_*temp_delt_x>-temp_delt_y));
			temp_left_slope=(temp_left_slope)||((temp_delt_x<0)&&((temp_delt_y>0&&-temp_delt_x*curbTan_tresh_>temp_delt_y)||(temp_delt_y<0&&-temp_delt_x*curbTan_tresh_>-temp_delt_y)));
			
			bool distY = temp_delt_y > -10 && temp_delt_y < 10 ;
			
			if(temp_left_height&&temp_left_slope&& distY)
			{temp_left_flag=true;ROS_DEBUG("left_curb true");}
			else{ROS_DEBUG("left1: not satisfied");}
	    }
	    //backup case;
	    if((temp_left_flag==false)&&(left_line_end_PID!=0)&&(left_backup_end!=0))
	    {
			geometry_msgs::Point32 temp_left_begin_Point=laser_cloud_.points[left_line_end_PID];
			geometry_msgs::Point32 temp_left_end_Point=laser_cloud_.points[left_backup_end];
			
			geometry_msgs::PointStamped left_begin_Point_base;
			geometry_msgs::PointStamped left_end_Point_base;
			tfSicktoBaselink(temp_left_begin_Point,left_begin_Point_base);
			tfSicktoBaselink(temp_left_end_Point,left_end_Point_base);
			
			//criterion 4: curb is higher than 0.05, lower than 100 meter;
			float temp_delt_z=left_begin_Point_base.point.z-left_end_Point_base.point.z;
			bool temp_left_height=(temp_delt_z>curbLow_tresh_ && temp_delt_z<curbheigh_tresh_)||(temp_delt_z<-curbLow_tresh_&& temp_delt_z>-curbheigh_tresh_);
			
			//criterion 5
			float temp_delt_x=temp_left_begin_Point.x-temp_left_end_Point.x;
			float temp_delt_y=temp_left_begin_Point.y-temp_left_end_Point.y;
			bool temp_left_slope =(temp_delt_x>=0)&&((temp_delt_y>0&&curbTan_tresh_*temp_delt_x>temp_delt_y)||(temp_delt_y<0&&curbTan_tresh_*temp_delt_x>-temp_delt_y));
			temp_left_slope=(temp_left_slope)||((temp_delt_x<0)&&((temp_delt_y>0&&-temp_delt_x*curbTan_tresh_>temp_delt_y)||(temp_delt_y<0&&-temp_delt_x*curbTan_tresh_>-temp_delt_y)));
			
			bool distY = temp_delt_y > -10 && temp_delt_y < 10 ;
			
			if(temp_left_height&&temp_left_slope&&distY)
			{temp_left_backup_flag=true;ROS_DEBUG("left_curb backup true");}
			else{ROS_DEBUG("left2: not satisfied");}
		}
	    
	    
	    //for right_line
	    if((right_line_begin_PID!=0)&&(right_line_end_PID!=0))
		{
			geometry_msgs::Point32 temp_right_begin_Point=laser_cloud_.points[right_line_begin_PID];
			geometry_msgs::Point32 temp_right_end_Point=laser_cloud_.points[right_line_end_PID];
		
			geometry_msgs::PointStamped right_begin_Point_base;
			geometry_msgs::PointStamped right_end_Point_base;
			tfSicktoBaselink(temp_right_begin_Point,right_begin_Point_base);
			tfSicktoBaselink(temp_right_end_Point,right_end_Point_base);

			float temp_delt_z=right_begin_Point_base.point.z-right_end_Point_base.point.z;
			bool temp_right_height=(temp_delt_z>curbLow_tresh_ && temp_delt_z<curbheigh_tresh_)||(temp_delt_z<-curbLow_tresh_ &&temp_delt_z> -curbheigh_tresh_);
			
			float temp_delt_x=temp_right_begin_Point.x-temp_right_end_Point.x;
			float temp_delt_y=temp_right_begin_Point.y-temp_right_end_Point.y;
			bool temp_right_slope =(temp_delt_x>=0)&&((temp_delt_y>0&&curbTan_tresh_*temp_delt_x>temp_delt_y)||(temp_delt_y<0&&curbTan_tresh_*temp_delt_x>-temp_delt_y));
			temp_right_slope=(temp_right_slope)||((temp_delt_x<0)&&((temp_delt_y>0&&-temp_delt_x*curbTan_tresh_>temp_delt_y)||(temp_delt_y<0&&-temp_delt_x*curbTan_tresh_>-temp_delt_y)));
			
			bool distY = temp_delt_y > -10 && temp_delt_y < 10 ;
			
			if(temp_right_height&&temp_right_slope&&distY)
			{temp_right_flag=true;ROS_DEBUG("right curb true");}

	    }
	    //backup case;
	    if((temp_right_flag==false)&&(right_line_end_PID!=0)&&(right_backup_end!=0))
	    {
			geometry_msgs::Point32 temp_right_begin_Point=laser_cloud_.points[right_line_end_PID];
			geometry_msgs::Point32 temp_right_end_Point=laser_cloud_.points[right_backup_end];
		
			geometry_msgs::PointStamped right_begin_Point_base;
			geometry_msgs::PointStamped right_end_Point_base;
			tfSicktoBaselink(temp_right_begin_Point,right_begin_Point_base);
			tfSicktoBaselink(temp_right_end_Point,right_end_Point_base);

			float temp_delt_z=right_begin_Point_base.point.z-right_end_Point_base.point.z;
			bool temp_right_height=(temp_delt_z>curbLow_tresh_&&temp_delt_z<curbheigh_tresh_)||(temp_delt_z<-curbLow_tresh_&&temp_delt_z>-curbheigh_tresh_);
			
			float temp_delt_x=temp_right_begin_Point.x-temp_right_end_Point.x;
			float temp_delt_y=temp_right_begin_Point.y-temp_right_end_Point.y;
			bool temp_right_slope =(temp_delt_x>=0)&&((temp_delt_y>0&&curbTan_tresh_*temp_delt_x>temp_delt_y)||(temp_delt_y<0&&curbTan_tresh_*temp_delt_x>-temp_delt_y));
			temp_right_slope=(temp_right_slope)||((temp_delt_x<0)&&((temp_delt_y>0&&-temp_delt_x*curbTan_tresh_>temp_delt_y)||(temp_delt_y<0&&-temp_delt_x*curbTan_tresh_>-temp_delt_y)));
			bool distY = temp_delt_y > -10 && temp_delt_y < 10 ;
			
			if(temp_right_height&&temp_right_slope&&distY)
			{temp_right_backup_flag=true;ROS_DEBUG("right curb backup true");}
		}


		sensor_msgs::PointCloud svm_curb_data_set;
		svm_curb_data_set.header = left_curb_line_.header;

		//push back left curb line

		road_detection::curbPointCloudID cpcID;

	    if(temp_left_flag==true)
	    {
			//the serial or sequence of "begin" and "end" will be changed here because of "push_back";
			for(unsigned int i=left_line_begin_PID; i<=left_line_end_PID;i++)
			{left_curb_line_.points.push_back(laser_cloud_.points[i]);}
			cpcID.pc = laser_cloud_;
			cpcID.id_start = left_line_begin_PID;
			cpcID.id_end = left_line_end_PID;
			curbPCIDLeft_pub_.publish(cpcID);
			//road_detect::svmLeftCurbFeatures(laser_cloud_,left_line_begin_PID, left_line_end_PID);
		}
		if(temp_left_backup_flag==true)
	    {
			for(unsigned int i=left_line_end_PID; i<=left_backup_end;i++)
			{left_curb_line_.points.push_back(laser_cloud_.points[i]);}
			cpcID.pc = laser_cloud_;
			cpcID.id_start = left_line_end_PID;
			cpcID.id_end = left_backup_end;
			curbPCIDLeft_pub_.publish(cpcID);
			//road_detect::svmLeftCurbFeatures(laser_cloud_,left_line_end_PID, left_backup_end);
		}
		
		//push back right curb line
	    if(temp_right_flag==true)
	    {
			for(unsigned int i=right_line_end_PID; i<=right_line_begin_PID;i++)
			{right_curb_line_.points.push_back(laser_cloud_.points[i]);}
			cpcID.pc = laser_cloud_;
			cpcID.id_start = right_line_end_PID;
			cpcID.id_end = right_line_begin_PID;
			curbPCIDRight_pub_.publish(cpcID);
			//road_detect::svmRightCurbFeatures(laser_cloud_,right_line_end_PID, right_line_begin_PID);
		}
		if(temp_right_backup_flag==true)
	    {
			for(unsigned int i=right_backup_end; i<=right_line_end_PID;i++)
			{right_curb_line_.points.push_back(laser_cloud_.points[i]);}
			cpcID.pc = laser_cloud_;
			cpcID.id_start = right_backup_end;
			cpcID.id_end = right_line_end_PID;
			curbPCIDRight_pub_.publish(cpcID);
			//road_detect::svmRightCurbFeatures(laser_cloud_,right_backup_end, right_line_end_PID);
		}
		
		sensor_msgs::PointCloud baselink_leftline_;
		sensor_msgs::PointCloud baselink_rightline_;

		sick_to_baselink_.transformPointCloud("base_link", left_curb_line_, baselink_leftline_);
		sick_to_baselink_.transformPointCloud("base_link", right_curb_line_, baselink_rightline_);
		
		//publish two lines;
		left_curbline_pub_.publish(baselink_leftline_);		
		right_curbline_pub_.publish(baselink_rightline_);
	}
	
	
	void road_detect::tfSicktoBaselink(geometry_msgs::Point32& point_para,geometry_msgs::PointStamped& stampedpoint_para)
	{
		geometry_msgs::PointStamped sick_local;
		sick_local.header=laser_cloud_.header;
		sick_local.point.x=point_para.x;
		sick_local.point.y=point_para.y;
		sick_local.point.z=0;
		sick_to_baselink_.transformPoint("base_link", sick_local, stampedpoint_para);
	}
	



	
};



int main(int argc, char** argv)
{
	 ros::init(argc, argv, "roaddetect_node");
	 
	 road_detection::road_detect* road_detect_node;
	 road_detect_node = new road_detection::road_detect();
	 
	 ros::spin();
}
