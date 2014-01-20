/*
 * SMSpeedControl.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: liuwlz
 */

#include <Speed_Control/SMSpeedControl.hpp>

namespace MPAV{

	SMSpeedControl::SMSpeedControl(){
		ROS_INFO("SM Speed Control");
		nh.param("base_frame", base_frame, string("base_link"));

		nh.param("offcenter_x", offcenter_x, 1.7);
		nh.param("offcenter_y", offcenter_y, 0.8);
		nh.param("back_boundary", back_boundary, 0.5);

		nh.param("x_buffer", x_buffer, 1.5);
		nh.param("y_buffer", y_buffer, 0.2);
		nh.param("x_coeff", x_coeff, 2.0);
		nh.param("y_coeff", y_coeff, 0.25);
		nh.param("angle_coeff", angle_coeff, 2.0);
		nh.param("vel_inc", vel_inc, 0.3);
		nh.param("replan_vel_inc", replan_vel_inc, 0.5);
		nh.param("replan_vel", replan_vel, 0.8);
		nh.param("slow_move_vel", slow_move_ratio, 0.6);
		nh.param("temp_stop_dec", temp_stop_dec, 0.4);

		base_frame = "base_link";

		vFilter_ = fmutil::LowPassFilter(0.1);
		filter_time = ros::Time::now();

		odom_sub_ = nh.subscribe("odom",1, &SMSpeedControl::OdomCallBack, this);
		advised_vel_sub_ = nh.subscribe("cmd_speed",1,&SMSpeedControl::AdvisedSpeedCallBack, this);

		processed_vel_pub_ = nh.advertise<twist_t>("cmd_sm",1);
		dead_zone_pub_ = nh.advertise<poly_t>("dead_zone_poly",1);
		dynamic_zone_pub_ = nh.advertise<poly_t>("dynamic_zone_poly",1);
		obst_view_pub_ = nh.advertise<pc_t>("speed_zone_obst_view",1);
		song_status_pub = nh.advertise<std_msgs::UInt16>("voice_id", 1);
		vel_process_timer_ = nh.createTimer(ros::Duration(0.05), &SMSpeedControl::SpeedControlTimer, this);

		laser_sub_.subscribe(nh, "front_bottom_scan", 10);
		tf_filter_ = new tf::MessageFilter<laser_t>(laser_sub_, tf_, base_frame, 10);
		tf_filter_->registerCallback(boost::bind(&SMSpeedControl::LaserCallBack, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.05));

		current_vel = 0.0;
		current_angle = 0.0;
		stop_time_threshold = 10.0;

		front_boundary = offcenter_x + x_buffer;
		side_boundary  = offcenter_y + y_buffer;
		geometry_msgs::Point32 corner[4];
		corner[0].x = front_boundary; corner[0].y = -side_boundary;
		corner[1].x = front_boundary; corner[1].y = side_boundary;
		corner[2].x = -back_boundary; corner[2].y = side_boundary;
		corner[3].x = -back_boundary; corner[3].y = -side_boundary;
		for(int i=0; i<4; i++)
			danger_zone.polygon.points.push_back(corner[i]);

		stop_time_init_ = false;
		SpeedState = Normal;
	}

	SMSpeedControl::~SMSpeedControl(){

	}

	void SMSpeedControl::OdomCallBack(const odom_t odomIn){
		ros::Time time_now = ros::Time::now();
		double dt = (time_now - filter_time).toSec();
		current_vel = fabs(vFilter_.filter_dt(dt, odomIn.twist.twist.linear.x));
		filter_time = time_now;
		front_boundary = offcenter_x + x_buffer + x_coeff*current_vel;
		side_boundary  = offcenter_y + y_buffer + y_coeff*current_vel + angle_coeff*current_angle*current_vel;
		poly_t dynamic_safety_zoon;
		geometry_msgs::Point32 corner[4];
		corner[0].x = front_boundary; corner[0].y = -side_boundary;
		corner[1].x = front_boundary; corner[1].y = side_boundary;
		corner[2].x = -back_boundary; corner[2].y = side_boundary;
		corner[3].x = -back_boundary; corner[3].y = -side_boundary;

		for(int i=0; i<4; i++)
			dynamic_safety_zoon.polygon.points.push_back(corner[i]);

		dynamic_safety_zoon.header = odomIn.header;
		dynamic_safety_zoon.header.frame_id = base_frame;
		dynamic_zone_pub_.publish(dynamic_safety_zoon);

		danger_zone.header = odomIn.header;
		danger_zone.header.frame_id = base_frame;
		dead_zone_pub_.publish(danger_zone);
	}

	void SMSpeedControl::AdvisedSpeedCallBack(const twist_t velIn){
		advised_vel = velIn;
	}

	void SMSpeedControl::LaserCallBack(const laser_t::ConstPtr &laserIn){
        laser_pc.points.clear();
		try{
        	projector_.transformLaserScanToPointCloud(base_frame, *laserIn, laser_pc, tf_);
        }
        catch (tf::TransformException& e){
        	ROS_INFO_STREAM(e.what());
        	return;
        }
	}

	void SMSpeedControl::SteerAngleCallBack(const pnc_msgs::move_status moveStatusIn){
		current_angle = fabs(moveStatusIn.steer_angle);
	}

	void SMSpeedControl::GetObstacleStatus(){
		obst_pc.header = laser_pc.header;
	    obst_pc.points.clear();
		for(size_t i = 0; i<laser_pc.points.size(); i++){
			geometry_msgs::Point32 point_tmp = laser_pc.points[i];
			if((point_tmp.x < -back_boundary || point_tmp.x > front_boundary) || fabs(point_tmp.y)>side_boundary)
				continue;
			obst_pc.points.push_back(point_tmp);
		}
		obst_view_pub_.publish(obst_pc);
	}

	void SMSpeedControl::GetDynamicSpeed(double &dynamic_speed){
    	double minimum_speed = advised_vel.linear.x;
    	// Find the appropriate speed according to obstacle distance
    	for(size_t i = 0; i<obst_pc.points.size(); i++){
			geometry_msgs::Point32 point_tmp = obst_pc.points[i];
			double speed_from_x, speed_from_y, min_speed_tmp;
			speed_from_x = (point_tmp.x - offcenter_x - x_buffer)/x_coeff;
			speed_from_y = ( fabs(point_tmp.y) - offcenter_y - y_buffer - angle_coeff*current_angle*current_vel)/y_coeff;
			if(speed_from_x >0.0 || speed_from_y > 0.0){
				min_speed_tmp = speed_from_x > speed_from_y ? speed_from_x:speed_from_y;
			}
			else{
				min_speed_tmp = 0.0;
			}
    		if(min_speed_tmp < minimum_speed)
    			minimum_speed = min_speed_tmp;
    	}
    	//	Smoothly increase the speed.
		if(minimum_speed > current_vel){
            double speed_temp;
            if(minimum_speed-current_vel > vel_inc )
            	speed_temp = current_vel + vel_inc;
            else
            	speed_temp = minimum_speed;

            minimum_speed = speed_temp > 0.7? speed_temp:0.7;
		}
		dynamic_speed = minimum_speed;
	}

	void SMSpeedControl::NormalPlanSpeedControl(){
		ROS_INFO("Normal Speed Phase");
		double dynamic_speed;
		GetDynamicSpeed(dynamic_speed);
    	processed_vel = advised_vel;
    	processed_vel.linear.x = dynamic_speed;
    	processed_vel_pub_.publish(processed_vel);
	}

	void SMSpeedControl::SlowMoveSpeedControl(){
		ROS_INFO("SlowMove Speed Phase");
		double dynamic_speed;
		GetDynamicSpeed(dynamic_speed);
		processed_vel = advised_vel;
		processed_vel.linear.x = slow_move_ratio*dynamic_speed;
		processed_vel_pub_.publish(processed_vel);
	}

	void SMSpeedControl::ReplanSpeedControl(){
		//TODO: Test the necessary for add dynamic_speed inside replan speed for safety.
		ROS_INFO("Replan Speed Control Phase");
		double dynamic_speed, replan_speed;
		GetDynamicSpeed(dynamic_speed);
		replan_speed = replan_vel;
		if(replan_speed > current_vel){
            double speed_temp;
            if(replan_speed - current_vel > replan_vel_inc)
            	speed_temp = current_vel + replan_vel_inc;
            else
            	speed_temp = replan_speed;
            replan_speed = speed_temp > 0.7 ? speed_temp:0.7;
		}
		replan_speed = dynamic_speed < 0.1 ? 0:replan_speed;
		processed_vel = advised_vel;
		processed_vel.linear.x = replan_speed;
		processed_vel_pub_.publish(processed_vel);
	}

	void SMSpeedControl::TempStopControl(){
		processed_vel = advised_vel;
		double speed_temp = current_vel - temp_stop_dec;
		processed_vel.linear.x = speed_temp > 0 ? speed_temp : 0;
		processed_vel_pub_.publish(processed_vel);
	}

	void SMSpeedControl::SetStatus(int status){
		SpeedState = SPEEDSTATUS(status);
	}

	void SMSpeedControl::SpeedControlTimer(const ros::TimerEvent &e){
		GetObstacleStatus();
		switch(SpeedState){
			case Normal:
				NormalPlanSpeedControl();
				break;
			case SlowMove:
				SlowMoveSpeedControl();
				break;
			case Replan:
				ReplanSpeedControl();
				break;
			case TempStop:
				TempStopControl();
			default:
				NormalPlanSpeedControl();
		}
		PedInterfaceVoice();
	}

	void SMSpeedControl::PedInterfaceVoice(){
		SongStatus.data = 255;
		if(obst_pc.points.size()>0){
			SongStatus.data = 4;
			if (current_vel < 0.1){
				if(!stop_time_init_){
					stop_time_init_ = true;
					stop_time_ = ros::Time::now();
				}
				else{
					double stop_time_duration = ros::Time::now().toSec() - stop_time_.toSec();
					if (stop_time_duration > stop_time_threshold)
						SongStatus.data = 3;
				}
			}
			else
				stop_time_ = ros::Time::now();
		}
		song_status_pub.publish(SongStatus);
	}
}
