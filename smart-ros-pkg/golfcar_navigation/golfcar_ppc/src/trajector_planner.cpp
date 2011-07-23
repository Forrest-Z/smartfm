#include <pluginlib/class_list_macros.h>
#include <math.h>
#include <golfcar_ppc/trajector_planner.h>

PLUGINLIB_DECLARE_CLASS(golfcar_purepursuit, PurePursuitBase, golfcar_purepursuit::PurePursuitBase, nav_core::BaseLocalPlanner)

namespace golfcar_purepursuit{

	void PurePursuitBase::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
		ros::NodeHandle global_node;
		ros::NodeHandle private_nh("~/" + name);
		private_nh.param("slow_speed", slow_speed_, 0.5);
		private_nh.param("maximum_speed", maximum_speed_, 1.5);
		private_nh.param("highrisk_speed", highrisk_speed_, 0.5);
		l_plan_pub_ = global_node.advertise<nav_msgs::Path>("local_plan", 1);
		g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
		clear_space_pub_ = global_node.advertise<geometry_msgs::PolygonStamped>("clear_space",1);
		golfcar_direction_ =global_node.subscribe("direction", 1, &PurePursuitBase::golfcar_direction, this);
		tf_ = tf;
		costmap_ros_ = costmap_ros;
		pp_ = new PurePursuit();
		forward_ = true;
		stopped_ = false;
		//ROS_INFO("%s initialized",name);
		std::cout<<name<<"\n";
	}
	   void PurePursuitBase::golfcar_direction(geometry_msgs::Point32 p)
  {
	  if(p.x == 255) forward_ = true;
	  else forward_ = false;
  }
	void PurePursuitBase::UpdatePosition()
	{
		//use robot pose from cost map
		tf::Stamped<tf::Pose> global_pose;
		costmap_ros_->getRobotPose(global_pose);
		
		tf::poseStampedTFToMsg(global_pose, robot_pose);
		
		
		//return tf::getYaw(robot_pose.pose.orientation);
	}
	bool PurePursuitBase::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
			//ROS_INFO("44");
			PurePursuitBase::UpdatePosition();
			std::vector<double> proposed_velocities;
			//just a hack to let it stop at junction
			//ROS_INFO("48");
			/*
			if(robot_pose.pose.position.x >=192.5) 
			{
				if(!stopped_)
				{
					expected_end_ = ros::Time::now() + ros::Duration(0.5);
					stopped_ = true;
				}
				if(expected_end_ > ros::Time::now())
					proposed_velocities.push_back(0);
			}
			else stopped_ = false;*/
			//ROS_INFO("60");
			pp_->vehicle_base_ = robot_pose.pose;
			
			
			double steer_angle;
			//ROS_INFO("65");
			path_flag_= pp_->steering_control(steer_angle);
			/*if(forward_ && (robot_pose.pose.position.x >185 && robot_pose.pose.position.x <197))
				proposed_velocities.push_back(slow_speed_);
			if(!forward_ && (robot_pose.pose.position.x >195 && robot_pose.pose.position.x <208))
				proposed_velocities.push_back(slow_speed_);*/
			if(path_flag_){
				if(steer_angle<0) proposed_velocities.push_back(slow_speed_+exp(steer_angle/0.3)*(maximum_speed_-slow_speed_));
				else proposed_velocities.push_back(slow_speed_+exp(-steer_angle/0.3)*(maximum_speed_-slow_speed_));
				
				cmd_vel.angular.z = steer_angle;
				//ROS_INFO("76");
				//implementation of collision detection
				geometry_msgs::PointStamped pointst;
			
				pointst.header.frame_id = costmap_ros_->getBaseFrameID();
				pointst.header.stamp = ros::Time();
				ros::Time current_time = ros::Time::now();
				lookahead_points.clear();
				//collision detection area
				pointst.point.x = 1.985; pointst.point.y = -1; lookahead_points.push_back(pointst);
				pointst.point.x = 1.985+7*cos(steer_angle); pointst.point.y = -1+7*sin(steer_angle);  lookahead_points.push_back(pointst);
				pointst.point.x = 1.985+7*cos(steer_angle);pointst.point.y = 1+7*sin(steer_angle);  lookahead_points.push_back(pointst);
				pointst.point.x = 1.985; pointst.point.y = 1;  lookahead_points.push_back(pointst);
				//ROS_INFO("89");
				//for visualization
				clear_space_.header.stamp = ros::Time();
				clear_space_.header.frame_id = "map";
				clear_space_.polygon.points.clear();
				
				//ROS_INFO("95");
				geometry_msgs::PointStamped temp;
				geometry_msgs::Point32 tempPoint32;
				std::vector<geometry_msgs::Point32> polygon_in_world;
				//get proper transfrom to convert world coordinate to costmap coordinate
				
				for(unsigned int i=0;i<lookahead_points.size();i++)
				{
					try{
						lookahead_points[i].header.stamp = ros::Time();
						tf_->transformPoint("map",lookahead_points[i],temp);
						tempPoint32.x = temp.point.x;
						tempPoint32.y = temp.point.y;
						polygon_in_world.push_back(tempPoint32);
					}
					catch(tf::TransformException& e){
						std::cout << e.what();
						return false;
					}
					
				}
				//ROS_INFO("115");
				//add critical line
				geometry_msgs::Point32 critical_line_p1, critical_line_p2;
				critical_line_p1.x = (polygon_in_world[1].x - polygon_in_world[0].x)*2/7 + polygon_in_world[0].x;
				critical_line_p1.y = (polygon_in_world[1].y - polygon_in_world[0].y)*2/7 + polygon_in_world[0].y;
				critical_line_p2.x = (polygon_in_world[2].x - polygon_in_world[3].x)*2/7 + polygon_in_world[3].x;
				critical_line_p2.y = (polygon_in_world[2].y - polygon_in_world[3].y)*2/7 + polygon_in_world[3].y;
				
				//adding the new polygons for visualization and further process for costmap
				std::vector<geometry_msgs::Point32> critical_polygon;
				critical_polygon.push_back(polygon_in_world[0]);
				critical_polygon.push_back(polygon_in_world[3]);
				critical_polygon.push_back(critical_line_p2);
				critical_polygon.push_back(critical_line_p1);
				
				std::vector<geometry_msgs::Point32> observed_polygon;
				observed_polygon.push_back(critical_line_p1);
				observed_polygon.push_back(critical_line_p2);
				observed_polygon.push_back(polygon_in_world[2]);
				observed_polygon.push_back(polygon_in_world[1]);
				//ROS_INFO("135");
				//for visualization
				for(unsigned int i=0;i<critical_polygon.size();i++)
					clear_space_.polygon.points.push_back(critical_polygon[i]);
				for(unsigned int i=0;i<observed_polygon.size();i++)
					clear_space_.polygon.points.push_back(observed_polygon[i]);
				clear_space_pub_.publish(clear_space_);
				
				//evaluate on the cost map
				std::vector<MapLocation> critical_polygon_costmap;
				std::vector<MapLocation> observed_polygon_costmap;
				MapLocation ml;
				Costmap2D c2d; 
				costmap_ros_->getCostmapCopy(c2d);
				int obstacle_cost(0);
				std::vector<MapLocation> observed_polygon_cells;
				std::vector<MapLocation> critical_polygon_cells;
				
				for(unsigned int i=0;i<observed_polygon.size();i++)
				{
					c2d.worldToMap(observed_polygon[i].x, observed_polygon[i].y, ml.x, ml.y);
					observed_polygon_costmap.push_back(ml);
				}
				c2d.polygonOutlineCells(observed_polygon_costmap,observed_polygon_cells);
				
				//get fill cells using function from base_local_planner
				//ROS_INFO("161");
				PurePursuitBase::getFillCells(observed_polygon_cells);
				
				int obstacle(0), observed_obstacle(0);
				for(unsigned int i=0;i<observed_polygon_cells.size();i++)
				{
					int cells_cost = (int)c2d.getCost(observed_polygon_cells[i].x, observed_polygon_cells[i].y);
						//std::cout<<observed_polygon_cells[i].x<<"\t"<<observed_polygon_cells[i].y<<"\t"<<cells_cost<<"\n";
					if(cells_cost<255) obstacle_cost+=cells_cost;
					if(cells_cost==254) observed_obstacle++;
				}
				
				//start of critical polygon evaluation
				for(unsigned int i=0;i<critical_polygon.size();i++)
				{
					c2d.worldToMap(critical_polygon[i].x, critical_polygon[i].y, ml.x, ml.y);
					critical_polygon_costmap.push_back(ml);
				}
				c2d.polygonOutlineCells(critical_polygon_costmap,critical_polygon_cells);
				PurePursuitBase::getFillCells(critical_polygon_cells);
				for(unsigned int i=0;i<critical_polygon_cells.size();i++)
				{
					double obstacle_x,obstacle_y;
					
					int cells_cost = (int)c2d.getCost(critical_polygon_cells[i].x, critical_polygon_cells[i].y);
					
					if(cells_cost==254)
					{
						c2d.mapToWorld(critical_polygon_cells[i].x, critical_polygon_cells[i].y,obstacle_x, obstacle_y);
						std::cout<<"Hazard, obstacle in front! Stopping...."<<obstacle_x<<" "<<obstacle_y<<"\n";
						cmd_vel.linear.x = 0;
						obstacle++;
					}
					else
					{
						//we ignore unknown spaces here.
						if(cells_cost<255) obstacle_cost+=cells_cost;
					}
				}
				//ROS_INFO("200");
				double risk = (255 - (double)obstacle_cost/(critical_polygon_cells.size()+observed_polygon_cells.size()))/255;
				ROS_DEBUG("Risk (0-1): %lf", risk);
				//double risk_factor =
				if(obstacle==0) 
				{
					//changed for more safety margin
					double propose_speed = maximum_speed_ * risk;
					if(propose_speed < slow_speed_) propose_speed = slow_speed_;
					
					proposed_velocities.push_back(propose_speed);
					if(observed_obstacle>0) proposed_velocities.push_back(slow_speed_);
				}
				else proposed_velocities.push_back(0);
				//search for the minimum velocity
				double min_velocity = proposed_velocities[0];
				for(unsigned int i=1;i<proposed_velocities.size();i++)
					if(proposed_velocities[i]<min_velocity) 
						min_velocity = proposed_velocities[i];
				//std::cout<<obstacle<<" critical cells no:"<<critical_polygon_cells.size()<<" observed cells no:"<<observed_polygon_cells.size()<<" "<<observed_obstacle<<"\n";
				cmd_vel.linear.x = min_velocity;
			}
			else{
				cmd_vel.linear.x = 0;
				cmd_vel.angular.z = 0;
			}
			return true;
		
	}
	
	bool PurePursuitBase::isGoalReached(){
		//fix goal criteria. Only declare goal when the car hit the very last point in addition to the path_flag
		//std::cout<<pp_->path_n_<<"/"<<pp_->path_.poses.size()-1<<std::endl;
		if(!path_flag_ && pp_->path_n_<pp_->path_.poses.size()-1)
		{
			std::cout<<"Goal reached"<<std::endl;
			return true;
		}
		else return false;
		std::cout<<"Goal "<<pp_->path_n_<<'/'<<pp_->path_.poses.size()-1<<std::endl;
	}
			
	bool PurePursuitBase::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
		path_flag_=true;
		pp_-> path_.poses = orig_global_plan;
		pp_-> initialized_ = false;
		pp_-> dist_to_final_point = 100;
		pp_-> path_n_ =0;		
		return true;
	}
	//taken from base_local_planner
	 void PurePursuitBase::getFillCells(std::vector<MapLocation>& footprint){
    //quick bubble sort to sort pts by x
    MapLocation swap, pt;
    unsigned int i = 0;
    while(i < footprint.size() - 1){
      if(footprint[i].x > footprint[i + 1].x){
        swap = footprint[i];
        footprint[i] = footprint[i + 1];
        footprint[i + 1] = swap;
        if(i > 0)
          --i;
      }
      else
        ++i;
    }

    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = footprint[0].x;
    unsigned int max_x = footprint[footprint.size() -1].x;
    //walk through each column and mark cells inside the footprint
    for(unsigned int x = min_x; x <= max_x; ++x){
      if(i >= footprint.size() - 1)
        break;

      if(footprint[i].y < footprint[i + 1].y){
        min_pt = footprint[i];
        max_pt = footprint[i + 1];
      }
      else{
        min_pt = footprint[i + 1];
        max_pt = footprint[i];
      }

      i += 2;
      while(i < footprint.size() && footprint[i].x == x){
        if(footprint[i].y < min_pt.y)
          min_pt = footprint[i];
        else if(footprint[i].y > max_pt.y)
          max_pt = footprint[i];
        ++i;
      }

      //loop though cells in the column
      for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
        pt.x = x;
        pt.y = y;
        footprint.push_back(pt);
      }
    }
  }
};
