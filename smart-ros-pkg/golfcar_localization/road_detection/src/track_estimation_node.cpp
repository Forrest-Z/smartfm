#include <track_estimation_node.h>
#include <cmath>

using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;

#define MAX_DISTANCE 15.0
#define CURB_HEIGHT 0.30

namespace estimation{

	//***************************the most elementary class of "TrackEstimationNode"******************************
	TrackEstimationNode::TrackEstimationNode():
	Rmeas_update_(true),
	Lmeas_update_(true),
	Rmeas_exist_(true),
	Lmeas_exist_(true),
    Rreinitial_(false),
    Lreinitial_(false),
    Rtrack_output_cred_(true),
    Ltrack_output_cred_(true),
    RcrossHandle_(true),
    LcrossHandle_(true)    	
	{
		ros::NodeHandle nh;
		
		cloud_baselink_pub_ = nh.advertise<sensor_msgs::PointCloud>("cloud_baselink_", 2);
		//right_point_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("right_point_", 2);
		//right_Ptcred_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("right_Ptcred_", 2);
		//left_point_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("left_point_", 2);
		//left_Ptcred_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("left_Ptcred_", 2); 
		
		//////////////// to publish both points in one topic/////////////////////
		left_point_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("curb_points", 10);
		right_point_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("curb_points", 10);
		//right_Ptcred_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("Ptcred_", 2);
		//left_Ptcred_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("Ptcred_", 2);
		
		Raw_Lpt_pub_ 		= nh.advertise<sensor_msgs::PointCloud>("raw_curb_points", 10);
		Raw_Rpt_pub_ 		= nh.advertise<sensor_msgs::PointCloud>("raw_curb_points", 10);
				 
		//cred_full_right_pub_= nh.advertise<sensor_msgs::PointCloud>("full_curbs", 2);
		//cred_full_left_pub_ = nh.advertise<sensor_msgs::PointCloud>("full_curbs", 2); 
		
		hybrid_Lpt_pub_		= nh.advertise<sensor_msgs::PointCloud>("hybrid_pt", 10);
		hybrid_Rpt_pub_ 	= nh.advertise<sensor_msgs::PointCloud>("hybrid_pt", 10);
		
		cloud_sub_.subscribe(nh, "laser_cloud_", 2);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(cloud_sub_, tf_, "odom", 10);
		tf_filter_->registerCallback(boost::bind(&TrackEstimationNode::cloudCallback, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.1));		
		
		Rcurb_sub_ = nh.subscribe("right_curbline_pub_", 10, &TrackEstimationNode::RcurbCallback, this);
		Lcurb_sub_ = nh.subscribe("left_curbline_pub_",  10, &TrackEstimationNode::LcurbCallback, this);
		
		odom_sub_ = nh.subscribe("odom", 10, &TrackEstimationNode::odomCallback, this);
		
		max_dis_point_.z = 0.0;
	}
	
	void TrackEstimationNode::cloudCallback (const sensor_msgs::PointCloud::ConstPtr& cloud_in)
	{
		try
		{
			tf_.transformPointCloud("base_link", *cloud_in, cloud_baselink_);
		}
		catch (tf::TransformException& e)
		{
			ROS_INFO("Wrong!!!!!!!!!!!!!");
			std::cout << e.what();
			return;
		}
		cloud_baselink_pub_.publish(cloud_baselink_);
	}
	
	void TrackEstimationNode::odomCallback(const OdomConstPtr& odom)
	{
		//ROS_INFO("odom call_back");
		ros::Time odom_time = odom->header.stamp;
		Quaternion q;
		tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
		tf::Transform odom_meas_add = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
		StampedTransform meas_add = StampedTransform(odom_meas_add.inverse(), odom_time, "base_link", "odom");
		transformer_.setTransform( meas_add );
	}
	
	
	void TrackEstimationNode::RcurbCallback(const sensor_msgs::PointCloud::ConstPtr&  Rcurb)
	{
		infoPt temp_infoPt;
		
		Rmeas_update_ = true;
		Rmeas_exist_  = true;		
		Rcurb_line_= * Rcurb;
		R_filter_.vehicles_flag_=false;
		right_point_.header = Rcurb_line_.header;
		right_point_.points.clear();
		
		right_Ptcred_.header= Rcurb_line_.header;
		right_Ptcred_.points.clear();
		
		cred_full_right_curb_.header= Rcurb_line_.header;
		cred_full_right_curb_.points.clear();
		
		Raw_Rpt_.header= Rcurb_line_.header;
		Raw_Rpt_.points.clear();
		
		//"hybrid_Rpt_" consists of information of distance for "curb" && "no_curb";
		// false curb of vehicles and pedestrians will not be included;
		hybrid_Rpt_.header= Rcurb_line_.header;
		hybrid_Rpt_.points.clear();
		
		geometry_msgs::Point32 right_point;
		
		TrackEstimationNode::RcurbProcess();
		
		ros::Time Rcurb_time = Rcurb_line_.header.stamp;
		if (!transformer_.canTransform("base_link","odom",Rcurb_time))
		{
			//here use "ROS_INFO" rather than "ROS_ERROR"; this situation is allowed here;
			ROS_INFO("curb_time older than odom message buffer");
			return;
		}
		else 
		{
			ROS_INFO("odom OK");
		}
		
		transformer_.lookupTransform("odom", "base_link", Rcurb_time, Rodom_meas_);
		
		tf::Transform odom_old_new  = Rodom_meas_old_.inverse() * Rodom_meas_;
		Rtx_ = -odom_old_new.getOrigin().y();
		Rty_ =  odom_old_new.getOrigin().x();
		float mov_dis = sqrtf(Rtx_*Rtx_ + Rty_*Rty_);
		float mov_dis_tresh =0.05; 		
		temp_infoPt.distoFormer_=mov_dis;
		
		if(mov_dis>mov_dis_tresh) {RcrossHandle_=true;}
		/*
		double deltyaw, ttemp;
		odom_old_new.getBasis().getEulerYPR(deltyaw, ttemp, ttemp);
		float rotate_dis_tresh = 0.09  //about 5 degree;
		*/
		
		if((!R_filter_.isInitialized())||Rreinitial_)
		{
			ROS_INFO("try to initial or reinitial");
			if(Rmeas_exist_)
			{
				ROS_INFO("can initial or reinitial: meas_exist_");
				
				R_filter_.initialize(Rcurb_observation_);
				Rodom_meas_old_  = Rodom_meas_;
				Rodom_meas_2old_ = Rodom_meas_;
				Rodom_meas_3old_ = Rodom_meas_;
				Rreinitial_ = false;	
				
				temp_infoPt.vecPt_= R_filter_.estimate_value_;
				temp_infoPt.covX_ = R_filter_.estimate_covariance_(1,1);
				temp_infoPt.covY_ = R_filter_.estimate_covariance_(2,2);
				temp_infoPt.firstPt_=true;		
				
				right_point.x = Rcurb_line_.points.back().x;
				right_point.y = Rcurb_line_.points.back().y;	
				right_point.z = Rcurb_line_.points.back().z;
				right_point_.points.push_back(right_point);
				Raw_Rpt_.points.push_back(right_point);
				
				if(right_point.y<MAX_DISTANCE && right_point.y>-MAX_DISTANCE)
				{
					hybrid_Rpt_.points.push_back(right_point);
				}
				else
				{
					max_dis_point_.x = R_filter_.old_forward_dis_;
					max_dis_point_.y = -MAX_DISTANCE;
					hybrid_Rpt_.points.push_back(max_dis_point_);
				}

				R_window_.rollingProcess(temp_infoPt, Rtrack_output_cred_);
				if(Rtrack_output_cred_)
				{
					right_Ptcred_.points.push_back(right_point);
					/*
					for (unsigned int i=0; i<Rcurb_line_.points.size();i++)
					{
						cred_full_right_curb_.points.push_back(Rcurb_line_.points[i]);
					}
					*/ 
				}								
			}
			else
			{
				if(RcrossHandle_) {Rodom_meas_old_  = Rodom_meas_; RcrossHandle_=false;}
				
				max_dis_point_.x = R_filter_.old_forward_dis_;
				max_dis_point_.y = -MAX_DISTANCE;
				hybrid_Rpt_.points.push_back(max_dis_point_);
			}			
		}
		else
		{
			//tf::Transform odom_old_new  = Rodom_meas_old_.inverse() * Rodom_meas_; //(move to above)			
			tf::Transform odom_2old_new = Rodom_meas_2old_.inverse() * Rodom_meas_;
			tf::Transform odom_3old_new = Rodom_meas_3old_.inverse() * Rodom_meas_;
					
			////always Remember to change coordinates. //(move to above)
			//Rtx_ = -odom_old_new.getOrigin().y();
			//Rty_ =  odom_old_new.getOrigin().x();
			//float mov_dis = sqrtf(Rtx_*Rtx_ + Rty_*Rty_);
			
			//update controlled by "mov_dis"; this is optional;
			
			if(mov_dis<mov_dis_tresh){ROS_INFO("R:mov_dis less than mov_dis_tresh");return;}

			double tmp = 0;
			double temp_delt_phi;
			odom_old_new.getBasis().getEulerYPR(temp_delt_phi, tmp, tmp);
			Rdelt_phi_ = (float) temp_delt_phi;
			Rodom_meas_old_ = Rodom_meas_;
			
			float estimation_credible_threshold = 0.3;
			float estimation_reinitial_threshold = 0.5;
			float measurement_reinitial_threshold = 0.5;
			float gate_para = 0.5;
			float gate_coefficient = 0.5;
			if(R_filter_.vehicles_flag_==true)
			{estimation_reinitial_threshold=4;measurement_reinitial_threshold=0.6; gate_coefficient=1;gate_coefficient=0.5;}
			
			
			float tx2, ty2, dis2;
			tx2 = -odom_2old_new.getOrigin().y();
			ty2 =  odom_2old_new.getOrigin().x();
			dis2 = sqrtf(tx2*tx2+ty2*ty2);
			ROS_INFO("R: vehicle move from last update: tx2 %f, ty2 %f, dis2 %f", tx2, ty2, dis2);
			//assume other vehicles will be no longer than 4 meters; when driving in the same direction;
			if(dis2>estimation_credible_threshold){Rtrack_output_cred_ = false;}
			if(dis2>estimation_reinitial_threshold)
			{ROS_INFO("dis2>estimation_reinitial_threshold, set reinitail true");Rreinitial_= true; return;}
			Rassociation_gate_ = gate_para+gate_coefficient*dis2;
			
			R_filter_.update(Rcurb_observation_, Rassociation_gate_, Rmeas_update_, Rreinitial_, Rdelt_phi_, Rtx_, Rty_);	
			if(Rmeas_update_){Rodom_meas_2old_ = Rodom_meas_; Rtrack_output_cred_ = true;}
						
			float tx3, ty3, dis3;
			tx3 = -odom_3old_new.getOrigin().y();
			ty3 =  odom_3old_new.getOrigin().x();
			dis3 = sqrtf(tx3*tx3+ty3*ty3);
			//ROS_INFO("vehicle move from last measurement: tx3 %f, ty3 %f, dis3 %f", tx3, ty3, dis3);			
			if(dis3>measurement_reinitial_threshold)
			{ROS_INFO("dis3>measurement_reinitial_threshold, set reinitail true"); Rtrack_output_cred_ = false; Rreinitial_= true; return;}
			if(Rmeas_exist_){Rodom_meas_3old_  = Rodom_meas_;}
			
			temp_infoPt.firstPt_=false;	
			temp_infoPt.vecPt_= R_filter_.estimate_value_;
			temp_infoPt.covX_ = R_filter_.estimate_covariance_(1,1);
			temp_infoPt.covY_ = R_filter_.estimate_covariance_(2,2);
			
			//to visualize the results;
			//always remember to change coordinates;
			right_point.x = R_filter_.estimate_value_.y;
			right_point.y =-R_filter_.estimate_value_.x;
			right_point.z = Rcurb_line_.points.back().z;			
			right_point_.points.push_back(right_point);
			
			R_window_.rollingProcess(temp_infoPt, Rtrack_output_cred_);
			
			if(Rtrack_output_cred_)
			{
					right_Ptcred_.points.push_back(right_point);
					
					/*
					for (unsigned int i=0; i<Rcurb_line_.points.size();i++)
					{
						cred_full_right_curb_.points.push_back(Rcurb_line_.points[i]);
					}
					*/
			}
			
			if(Rmeas_exist_)
			{
				if(!R_filter_.vehicles_flag_)
				{
					geometry_msgs::Point32 raw_right_point;
					raw_right_point.x = Rcurb_line_.points.back().x;
					raw_right_point.y = Rcurb_line_.points.back().y;	
					raw_right_point.z = Rcurb_line_.points.back().z;
					Raw_Rpt_.points.push_back(raw_right_point);
					
					if(raw_right_point.y<MAX_DISTANCE && raw_right_point.y>-MAX_DISTANCE)
					{
						hybrid_Rpt_.points.push_back(raw_right_point);
					}
					else
					{
						max_dis_point_.x = R_filter_.old_forward_dis_;
						max_dis_point_.y = -MAX_DISTANCE;
						hybrid_Rpt_.points.push_back(max_dis_point_);
					}
				}
			}
			else
			{
				max_dis_point_.x = R_filter_.old_forward_dis_;
				max_dis_point_.y = -MAX_DISTANCE;
				hybrid_Rpt_.points.push_back(max_dis_point_);
			}
						
		}
		right_point_pub_.publish(right_point_);
		//right_Ptcred_pub_.publish(right_Ptcred_);		
		//cred_full_right_pub_.publish(cred_full_right_curb_);
		
		
		
		if(mov_dis<mov_dis_tresh){return;}		
		else
		{
			Raw_Rpt_pub_.publish(Raw_Rpt_);
			hybrid_Rpt_pub_.publish(hybrid_Rpt_);
		}
	}
	
	
	void TrackEstimationNode::RcurbProcess()
	{
		if(Rcurb_line_.points.size()==0)
		{
			//ROS_INFO("Rcurb_line_ no points inside, set meas_exist_ ; meas_update_ as false");
			Rmeas_exist_  =false;
			Rmeas_update_ =false;
		 }

		else
		{	
			//aways Remeber to change coordinate;
			//pay attention for "begin" and "end" in right curbline;
			float end_x = -Rcurb_line_.points[0].y;
			float end_y =  Rcurb_line_.points[0].x;
			float begin_x   = -Rcurb_line_.points.back().y;
			float begin_y   =  Rcurb_line_.points.back().x;
			float beginthetha=0;
			float dx=begin_x-end_x;
			float dy=begin_y-end_y;
			
			float end_z = Rcurb_line_.points[0].z;
			float beg_z = Rcurb_line_.points.back().z;
			float dz = end_z-beg_z;
			ROS_INFO("heith of right curbline begin point, end point, %3f, %3f", beg_z, end_z);
			
			if(dz >CURB_HEIGHT||dz <-CURB_HEIGHT)
			{
				ROS_INFO("vehicle: dz %3f", dz);
				R_filter_.vehicles_flag_=true;
				Rmeas_update_ =false;
			}
			
			if((dx<0.01&&dx>0.0)||(dx>-0.01&&dx<0.0)){beginthetha = M_PI_2;}
			else{beginthetha = atan2f(dy,dx);if(beginthetha<0)beginthetha=beginthetha+M_PI;}
			
			Rcurb_observation_.x = begin_x;
			Rcurb_observation_.y = begin_y;
			Rcurb_observation_.thetha = beginthetha;
		}
	}
	
	void TrackEstimationNode::LcurbCallback(const sensor_msgs::PointCloud::ConstPtr&  Lcurb)
	{
		infoPt temp_infoPt;
		
		Lmeas_update_ = true;
		Lmeas_exist_  = true;		
		Lcurb_line_   = * Lcurb;
		L_filter_.vehicles_flag_=false;
		left_point_.header = Lcurb_line_.header;
		left_point_.points.clear();
		
		left_Ptcred_.header= Lcurb_line_.header;
		left_Ptcred_.points.clear();
		
		cred_full_left_curb_.header= Lcurb_line_.header;
		cred_full_left_curb_.points.clear();
		
		Raw_Lpt_.header= Lcurb_line_.header;
		Raw_Lpt_.points.clear();
		
		hybrid_Lpt_.header= Lcurb_line_.header;
		hybrid_Lpt_.points.clear();
		
		
		geometry_msgs::Point32 left_point;
		
		TrackEstimationNode::LcurbProcess();
		
		ros::Time Lcurb_time = Lcurb_line_.header.stamp;
		if (!transformer_.canTransform("base_link","odom",Lcurb_time))
		{
			//here use "ROS_INFO" rather than "ROS_ERROR";
			ROS_INFO("curb_time older than odom message buffer");
			return;
		}
		else {ROS_INFO("odom OK");}
		
		transformer_.lookupTransform("odom", "base_link", Lcurb_time, Lodom_meas_);
		
		tf::Transform odom_old_new  = Lodom_meas_old_.inverse() * Lodom_meas_;
		Ltx_ = -odom_old_new.getOrigin().y();
		Lty_ =  odom_old_new.getOrigin().x();
		float mov_dis = sqrtf(Ltx_*Ltx_ + Lty_*Lty_);
		float mov_dis_tresh =0.05;
		temp_infoPt.distoFormer_=mov_dis;
		
		if(mov_dis>mov_dis_tresh) {LcrossHandle_=true;}
		
		if((!L_filter_.isInitialized())||Lreinitial_)
		{
			ROS_INFO("try to initial or reinitial");
			if(Lmeas_exist_)
			{
				ROS_INFO("can initial or reinitial: meas_exist_");
				
				L_filter_.initialize(Lcurb_observation_);
				Lodom_meas_old_ = Lodom_meas_;
				Lodom_meas_2old_ = Lodom_meas_;
				Lodom_meas_3old_ = Lodom_meas_;
				
				Lreinitial_ = false;	
				
				temp_infoPt.vecPt_= L_filter_.estimate_value_;
				temp_infoPt.covX_ = L_filter_.estimate_covariance_(1,1);
				temp_infoPt.covY_ = L_filter_.estimate_covariance_(2,2);
				temp_infoPt.firstPt_=true;
				
				left_point.x = Lcurb_line_.points[0].x;
				left_point.y = Lcurb_line_.points[0].y;	
				left_point.z = Lcurb_line_.points[0].z;
				left_point_.points.push_back(left_point);
				Raw_Lpt_.points.push_back(left_point);	
				
				if(left_point.y<MAX_DISTANCE && left_point.y>-MAX_DISTANCE)
				{
					hybrid_Lpt_.points.push_back(left_point);
				}
				else
				{
					max_dis_point_.x = L_filter_.old_forward_dis_;
					max_dis_point_.y = MAX_DISTANCE;
					hybrid_Lpt_.points.push_back(max_dis_point_);
				}
				
				L_window_.rollingProcess(temp_infoPt, Ltrack_output_cred_);
				
				if(Ltrack_output_cred_)
				{
					left_Ptcred_.points.push_back(left_point);
					
					/*
					for (unsigned int i=0; i<Lcurb_line_.points.size();i++)
					{
						cred_full_left_curb_.points.push_back(Lcurb_line_.points[i]);
					}
					*/ 
				}				
			}
			else
			{
				if(LcrossHandle_) {Lodom_meas_old_  = Lodom_meas_; LcrossHandle_=false;}
				
				max_dis_point_.x = L_filter_.old_forward_dis_;
				max_dis_point_.y = MAX_DISTANCE;
				hybrid_Lpt_.points.push_back(max_dis_point_);
			}
		}
		else
		{
			//tf::Transform odom_old_new  = Lodom_meas_old_.inverse() * Lodom_meas_;
			tf::Transform odom_2old_new = Lodom_meas_2old_.inverse() * Lodom_meas_;
			tf::Transform odom_3old_new = Lodom_meas_3old_.inverse() * Lodom_meas_;
			
			double tmp = 0;
			double temp_delt_phi;
			
			////always Remember to change coordinates.
			//Ltx_ = -odom_old_new.getOrigin().y();
			//Lty_ =  odom_old_new.getOrigin().x();
			//float mov_dis = sqrtf(Ltx_*Ltx_ + Lty_*Lty_);
			
			
			if(mov_dis<mov_dis_tresh){ROS_INFO("L: mov_dis less than mov_dis_tresh");return;}
			
			odom_old_new.getBasis().getEulerYPR(temp_delt_phi, tmp, tmp);
			Ldelt_phi_ = (float) temp_delt_phi;
			Lodom_meas_old_ = Lodom_meas_;
			
			float estimation_credible_threshold = 0.3;
			float estimation_reinitial_threshold = 0.5;
			float measurement_reinitial_threshold = 0.5;
			float gate_para = 0.4;
			float gate_coefficient = 0.5;
			
			//for left filter, actually this case doesn't exist;
			if(L_filter_.vehicles_flag_==true)
			{estimation_reinitial_threshold=4;measurement_reinitial_threshold=0.6;gate_coefficient=1;gate_coefficient=0.5;}
			
			
			float tx2, ty2, dis2;
			tx2 = -odom_2old_new.getOrigin().y();
			ty2 =  odom_2old_new.getOrigin().x();
			dis2 = sqrtf(tx2*tx2+ty2*ty2);
			ROS_INFO("L: vehicle move from last update: tx2 %f, ty2 %f, dis2 %f", tx2, ty2, dis2);
			//assume other vehicles will be no longer than 4 meters; when driving in the same direction;
			if(dis2>estimation_credible_threshold){Ltrack_output_cred_ = false;}
			if(dis2>estimation_reinitial_threshold)
			{ROS_INFO("dis2>estimation_reinitial_threshold, set reinitail true");Lreinitial_= true; return;}
			Lassociation_gate_ = gate_para+gate_coefficient*dis2;
			
			//pay attention here;
			L_filter_.update(Lcurb_observation_, Lassociation_gate_, Lmeas_update_, Lreinitial_, Ldelt_phi_, Ltx_, Lty_);	
			
			if(Lmeas_update_){Lodom_meas_2old_ = Lodom_meas_; Ltrack_output_cred_ = true;}
			
			float tx3, ty3, dis3;
			tx3 = -odom_3old_new.getOrigin().y();
			ty3 =  odom_3old_new.getOrigin().x();
			dis3 = sqrtf(tx3*tx3+ty3*ty3);
			ROS_INFO("vehicle move from last measurement: tx3 %f, ty3 %f, dis3 %f", tx3, ty3, dis3);			
			if(dis3>measurement_reinitial_threshold)
			{ROS_INFO("dis3>measurement_reinitial_threshold, set reinitail true");Lreinitial_= true; return;}
			if(Lmeas_exist_){Lodom_meas_3old_  = Lodom_meas_;}
			
			
			temp_infoPt.firstPt_=false;	
			temp_infoPt.vecPt_= L_filter_.estimate_value_;
			temp_infoPt.covX_ = L_filter_.estimate_covariance_(1,1);
			temp_infoPt.covY_ = L_filter_.estimate_covariance_(2,2);
			
			//always remember to change coordinates;
			left_point.x = L_filter_.estimate_value_.y;
			left_point.y =-L_filter_.estimate_value_.x;
			left_point.z = Lcurb_line_.points[0].z;	
			
			left_point_.points.push_back(left_point);
			
			L_window_.rollingProcess(temp_infoPt, Ltrack_output_cred_);
			if(Ltrack_output_cred_)
			{
					left_Ptcred_.points.push_back(left_point);
					
					/*
					for (unsigned int i=0; i<Lcurb_line_.points.size();i++)
					{
						cred_full_left_curb_.points.push_back(Lcurb_line_.points[i]);
					}
					*/ 
			}

			if(Lmeas_exist_)
			{
				if(!L_filter_.vehicles_flag_)
				{
					geometry_msgs::Point32 raw_left_point;
					raw_left_point.x = Lcurb_line_.points[0].x;
					raw_left_point.y = Lcurb_line_.points[0].y;	
					raw_left_point.z = Lcurb_line_.points[0].z;
					Raw_Lpt_.points.push_back(raw_left_point);
				
					if(raw_left_point.y<MAX_DISTANCE && raw_left_point.y>-MAX_DISTANCE)
					{
						hybrid_Lpt_.points.push_back(raw_left_point);
					}
					else
					{
						max_dis_point_.x = L_filter_.old_forward_dis_;
						max_dis_point_.y = MAX_DISTANCE;
						hybrid_Lpt_.points.push_back(max_dis_point_);
					}
				}
			}
			else
			{
				max_dis_point_.x = L_filter_.old_forward_dis_;
				max_dis_point_.y = MAX_DISTANCE;
				hybrid_Lpt_.points.push_back(max_dis_point_);
			}						
		}

		left_point_pub_.publish(left_point_);
		//left_Ptcred_pub_.publish(left_Ptcred_);
		//cred_full_left_pub_.publish(cred_full_left_curb_);
		
			
		if(mov_dis<mov_dis_tresh){return;}
		else
		{
			Raw_Lpt_pub_.publish(Raw_Lpt_);
			hybrid_Lpt_pub_.publish(hybrid_Lpt_);
		}
		
	}
	
	
	void TrackEstimationNode::LcurbProcess()
	{
		if(Lcurb_line_.points.size()==0)
		{
			ROS_INFO("Lcurb_line_ no points inside, set meas_exist_ ; meas_update_ as false");
			Lmeas_exist_  =false;
			Lmeas_update_ =false;
		 }

		else
		{	
			//aways Remeber to change coordinate;
			float begin_x = -Lcurb_line_.points[0].y;
			float begin_y =  Lcurb_line_.points[0].x;
			float end_x   = -Lcurb_line_.points.back().y;
			float end_y   =  Lcurb_line_.points.back().x;
			float beginthetha=0;
			float dx=begin_x-end_x;
			float dy=begin_y-end_y;
			
			float beg_z = Lcurb_line_.points[0].z;
			float end_z = Lcurb_line_.points.back().z;
			float dz = end_z - beg_z;
			
			if(dz >CURB_HEIGHT||dz <-CURB_HEIGHT)
			{
				ROS_INFO("vehicle: dz %3f", dz);
				L_filter_.vehicles_flag_=true;
				Lmeas_update_ =false;
			}
			
			if((dx<0.01&&dx>0.0)||(dx>-0.01&&dx<0.0)){beginthetha = M_PI_2;}
			else{beginthetha = atan2f(dy,dx);if(beginthetha<0)beginthetha=beginthetha+M_PI;}
			
			Lcurb_observation_.x = begin_x;
			Lcurb_observation_.y = begin_y;
			Lcurb_observation_.thetha = beginthetha;
		}
	}
}


int main(int argc, char** argv)
{
	 ros::init(argc, argv, "track_estimation_node");
	 
	 estimation::TrackEstimationNode* track_estimation_node;
	 track_estimation_node = new estimation::TrackEstimationNode();
	 
	 ros::spin();
}

