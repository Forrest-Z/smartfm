#include <golfcar_ppc/golfcar_purepursuit.h>

namespace golfcar_purepursuit {


		PurePursuit::PurePursuit(){
			Lfw_ = 4;
			lfw_ = 1;
			car_length = 1.632;
			nextPathThres_ = 5;
			dist_to_final_point = 100;
			initialized_=false;
			path_n_ = 0;
			nextPathCount_ = 0;
		}
		
		
		bool PurePursuit::steering_control(double& wheel_angle, bool outOfRange){
			geometry_msgs::Point pt;
			outOfRange = false;
			if(!initialized_)
			{
				current_point_ = vehicle_base_.position;
				next_point_ = path_.poses[0].pose.position;
				initialized_=true;
			}
			double heading_lh=0;
			int status=0;
			bool got_heading = heading_lookahead(heading_lh, status);
			if(status == -1) 
			{			
				outOfRange = true;
				return false;
			}
			wheel_angle = atan((car_length * sin(heading_lh))/(Lfw_/2+lfw_*cos(heading_lh)));
			if(wheel_angle > 0.65) wheel_angle = 0.65;
			else if(wheel_angle < -0.65) wheel_angle = -0.65;
			ROS_DEBUG("pursuing point: %lf %lf, %lf %lf", current_point_.x, current_point_.y, next_point_.x, next_point_.y);
			return got_heading;
			
			
		}
		

		bool PurePursuit::heading_lookahead(double &heading_la, int &status){
			double vehicle_heading = tf::getYaw(vehicle_base_.orientation);
			geometry_msgs::Point anchor_pt;
			anchor_pt.x = vehicle_base_.position.x + lfw_ * cos(vehicle_heading);
			anchor_pt.y = vehicle_base_.position.y + lfw_ * sin(vehicle_heading); 
			
			geometry_msgs::Point collided_pt; 
			int st;
			while(!circle_line_collision(anchor_pt,collided_pt, st))
			{
				
				//ROS_DEBUG("increment path_n_ %d, path size %d", path_n_, path_.poses.size());
				if(path_n_+1<path_.poses.size()){
					current_point_ = path_.poses[path_n_].pose.position;
					next_point_ = path_.poses[path_n_+1].pose.position;
					//ROS_DEBUG("Updating points");
				}
				else{/*
					collided_pt = next_point_;
					path_n_=path_.poses.size();
					double temp = PurePursuit::sqrt_distance(vehicle_base_.position,next_point_);
					if(dist_to_final_point<temp) {
						return false;
					}
					else {
						dist_to_final_point = temp; 
						break;
					}*/ return false;
				}
				nextPathCount_++;
				if(nextPathCount_%nextPathThres_==0)
				{
					path_n_++;
					ROS_INFO("Increment happen! %d points in path", path_n_);
				}
			}
			
			
			//heading_la = -1 indicate the controller couldn't find a path to follow, it has to be handle by trajectory_planner
			if(st==-1) status = st;
			else heading_la = atan2(collided_pt.y-anchor_pt.y, collided_pt.x-anchor_pt.x) - vehicle_heading;
			return true;
		}
		
		double PurePursuit::sqrt_distance(geometry_msgs::Point wp_a, geometry_msgs::Point wp_b)
		{
			return sqrt((wp_a.x - wp_b.x)*(wp_a.x - wp_b.x)+(wp_a.y - wp_b.y)*(wp_a.y - wp_b.y));
		} 
		 bool PurePursuit::circle_line_collision(geometry_msgs::Point& anchor_point, geometry_msgs::Point& intersect_point, int &status){
			//http://stackoverflow.com/questions/1073336/circle-line-collision-detection
			double Ex = current_point_.x;
			double Ey = current_point_.y;
			double Lx = next_point_.x;
			double Ly = next_point_.y;
			double Cx = anchor_point.x;
			double Cy = anchor_point.y;
			double r = Lfw_;
			double dx = Lx - Ex; double dy = Ly - Ey;
			double fx = Ex - Cx; double fy = Ey - Cy;
			
			float a = dx * dx + dy * dy;
			float b = 2 * (fx * dx + fy * dy);
			float c = (fx * fx + fy * fy) - (r * r);
			
			float discriminant = b*b-4*a*c;
			status =0;
			if(discriminant < 0)
			{
				ROS_WARN("No intersection, cur:x=%lf y%lf, next:x=%lf y=%lf, anchor_point:x=%lf y=%lf", Ex,Ey,Lx,Ly,Cx,Cy);
				//Very unlikely to happen! If it is, some very wrong happen. For example large localization error
				//Based on log it is due to sudden shift in localization, and it simply doesn't have a chance to execute the last path, try to go back to the second last segment is the safe way out!
				ROS_WARN("Reverting to the second last path.");
				status=-1;
				path_n_-=2;
				return true;
			}
			else
			{
				discriminant = sqrt(discriminant);
				float t1 = (-b + discriminant)/(2*a);
				//float t2 = (-b - discriminant)/(2*a);
				
				if(t1 >=0 && t1 <=1)
				{
					
					geometry_msgs::Point p2;
					intersect_point.x = Ex+t1*dx;
					intersect_point.y = Ey+t1*dy;
					
					return true;
				}
				else
				{
					ROS_WARN("No solution, cur:x=%lf y%lf, next:x=%lf y=%lf", Ex,Ey,Lx,Ly);
					
					return false;
				}
			}
		}
				
};

		
