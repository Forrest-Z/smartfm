/*
 * nearest_neighbor_tracking.cpp
 *
 *  Created on: Sep 24, 2012
 *      Author: demian
 */
#include <geometry_msgs/PoseStamped.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_filter.h>
#include <geometry_msgs/Point.h>

// new added code
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <cstdio>

#define POSE_HISTORY_NUM 100



class Refer_pose
{
public:
	double x;
	double y;
	double th;

};

class Threshold_filter
{
public:
	double last_y;
	double last_t;
	bool initial;
	Threshold_filter()
	{
        initial = false;
        last_y = 0;
        last_t = 0;
	}
    double update(double y,double t,double threshold)
    {
          if(initial == false)
          {
              initial = true;
              last_y = y;
          }
          else if (t-last_t < 1e-9)
          {
        	  last_y = y;
          }
          else
          {
              if(fabs((y-last_y)/(t-last_t)) > threshold)
              {
            	    double sign_ = (y-last_y)/(t-last_t);
            	    sign_ = sign_/fabs(sign_);
                    last_y =sign_ * threshold * (t - last_t) + last_y;
              }
              else
              {
            	  last_y = y;
              }

          }


          last_t = t;
          return last_y;
    }
};

class Average_filter
{
	double data[100];
	int cur_index;
	int num;
	bool is_full;
public:
	Average_filter(int data_num):cur_index(0),num(data_num),is_full(false){};

    double update(double input)
    {
    	double ret;
    	data[cur_index] = input;
    	if(is_full == true)
    	{
    		double data_min = data[0];
    		double data_max = data[0];
    		double sum = 0;
    		for(int i = 0; i < num; i++)
    		{
                if(data_min > data[i])
                {
                	data_min = data[i];
                }
                if(data_max < data[i])
                {
                	data_max = data[i];
                }
                sum = sum + data[i];
    		}

    		sum = sum -data_max - data_min;
    		ret = sum/(num - 2);
    	}
    	else
    	{
            ret = data[cur_index];
    	}

        cur_index = (cur_index + 1)%num;
        if(cur_index >= (num -1))
        {
        	is_full = true;
        }

        return ret;
    }

};
class NearestNeighborTracking
{
    geometry_msgs::PoseStamped cur_pose_;
    geometry_msgs::PoseStamped cur_new_pose_;
    geometry_msgs::PoseStamped last_valid_pose_;
    geometry_msgs::PoseStamped last_pose_;
    geometry_msgs::PoseStamped pose_history_[POSE_HISTORY_NUM];
    int cur_index_;
    bool is_history_full_;
    double yaw_from_path_;
    fmutil::LowPassFilter *filter_yaw_;
    fmutil::LowPassFilter *filter_x_;
    fmutil::LowPassFilter *filter_y_;
    fmutil::LowPassFilter * filter_new_yaw_;
    Threshold_filter thres_filter;
    Average_filter ave_filter;

    tf::TransformListener * tf_listener_;
    tf::TransformBroadcaster * tf_broadcaster_;
    tf::StampedTransform lastest_trans_;
    ros::Time last_update_time_;
    ros::Time start_time_;
    ifstream * pfs_;
    std::vector<Refer_pose> ref_pose_array_;
    unsigned int ref_index_;
    unsigned int search_area_;
    bool is_found_;
    double ref_angle_;
    tf::StampedTransform parent_trans;
    ros::Publisher robot_1_tf_pub_;



    double getYawFromQuadMsg(geometry_msgs::Quaternion quad_msg)
    {
        double y,p,r;
        tf::Quaternion quad;
        tf::quaternionMsgToTF(quad_msg, quad);
        btMatrix3x3(quad.asBt()).getEulerYPR(y,p,r);
        return y;
    }

    geometry_msgs::Quaternion getQuadMsgFromYaw(double yaw)
    {
        tf::Quaternion quad;
        quad.setRPY(0,0,yaw);
        geometry_msgs::Quaternion quadmsg;
        tf::quaternionTFToMsg(quad, quadmsg);
        return quadmsg;
    }
    double compute_dist(double x1, double y1,double x2,double y2)
    {
    	double delta_x = x1 - x2;
    	double delta_y = y1 - y2;
    	double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
    	return dist;
    }
public:
    NearestNeighborTracking(ros::NodeHandle * nh_,geometry_msgs::PoseStamped pose_init): cur_pose_(pose_init),ave_filter(25)
    {
        cout<<"Initialized with pose "<<cur_pose_.pose.position.x<<" "<<cur_pose_.pose.position.y<<endl;
        filter_yaw_ = new fmutil::LowPassFilter(0.4);

        filter_new_yaw_ = new fmutil::LowPassFilter(20);
        filter_x_ = new fmutil::LowPassFilter(0.2);
        filter_y_ = new fmutil::LowPassFilter(0.2);
        tf_listener_ = new tf::TransformListener;
        tf_broadcaster_ = new tf::TransformBroadcaster;
        robot_1_tf_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/robot_1/ref_tf",10);

        cur_index_ = 0;
        is_history_full_ = false;
        start_time_ = cur_pose_.header.stamp;
        pfs_ = new ifstream;
        pfs_->open("./physical_launch/golfcar_pose_downsample.txt",
        		fstream::in);
        cout<<"reading files"<<endl;
        Refer_pose temp_ref_pose;
        ref_index_ = 0;
        search_area_ = 300;
        is_found_ = false;
        ref_angle_ = 0;

        while(!pfs_->eof())
        {
             (*pfs_)>>temp_ref_pose.x>>temp_ref_pose.y>>temp_ref_pose.th;
             ref_pose_array_.push_back(temp_ref_pose);
        }
        cout<<"end of reading files"<<endl;
        cout<<temp_ref_pose.x<<" "<<temp_ref_pose.y<<" "<<temp_ref_pose.th<<endl;
        delete pfs_;

    }
    double findYawLeastSquare(geometry_msgs::PoseStamped pts[],geometry_msgs::Point p)
    {
           //changing the coordinate into normal xy coordinate

           for(int i=0; i<POSE_HISTORY_NUM; i++)
           {
               double x = pts[i].pose.position.x;
               double y = pts[i].pose.position.y;
               pts[i].pose.position.x = x;
               pts[i].pose.position.y = y;
           }
           //adapted from www.ccas.ru/mmes/educat/lab04k/02/least-squares.c
           double SUMx, SUMy, SUMxy, SUMxx, slope,
           y_intercept;
           SUMx = 0; SUMy = 0; SUMxy = 0; SUMxx = 0;




           for (int i=0; i<POSE_HISTORY_NUM; i++) {
               SUMx = SUMx + pts[i].pose.position.x;
               SUMy = SUMy + pts[i].pose.position.y;
               SUMxy = SUMxy + pts[i].pose.position.x*pts[i].pose.position.y;
               SUMxx = SUMxx + pts[i].pose.position.x*pts[i].pose.position.x;
           }
           slope = ( SUMx*SUMy - POSE_HISTORY_NUM*SUMxy ) / ( SUMx*SUMx - POSE_HISTORY_NUM*SUMxx );
           y_intercept = ( SUMy - slope*SUMx ) / POSE_HISTORY_NUM;

           double y1 = y_intercept, x1 = 0;
           double y2 = slope + y_intercept, x2 = 1;

           //using the changed coordinate to properly calculate the orientation (x = -y, y = x)
           //cout<<slope<<endl;
           //cout<<"-----------end-------------"<<endl;
           double yaw = atan2((y2-y1),x2-x1 );
           //if(slope>=0) return yaw+M_PI/2;
           //else return yaw-M_PI/2;
           //yaw+=M_PI/2;
           //if(isnan(yaw)) yaw=M_PI/2;
           if((p.x*(x2-x1) + p.y * (y2-y1)) < 0 )
        	{
        	   yaw+=M_PI;
            }
           cout<<"yaw is "<<yaw<<endl;
           return yaw;
       }

    geometry_msgs::Pose PoseMultTF(geometry_msgs::PoseStamped ps, tf::StampedTransform trans)
    {
    	tf::StampedTransform temp_trans1;
		tf::StampedTransform temp_trans2;
		tf::poseMsgToTF(ps.pose,temp_trans2);
		temp_trans1.mult(trans,temp_trans2);
		geometry_msgs::Pose temp_pose;
		tf::poseTFToMsg(temp_trans1,temp_pose);
		return temp_pose;
    }

    double convertToPi(double x)
    {
         while(x >= M_PI) x -= 2*M_PI;
         while(x < -M_PI) x += 2*M_PI;
         return x;
    }

    geometry_msgs::PoseStamped updateMeasurement(vector<geometry_msgs::PoseStamped> possible_poses)
    {

        double dist_now = 1e9;
        geometry_msgs::PoseStamped nearest_pose;
        //cout<<"-------------start----------"<<endl;
        for(size_t i=0; i<possible_poses.size(); i++)
        {
            double dist =fmutil::distance<geometry_msgs::Point>(possible_poses[i].pose.position, cur_pose_.pose.position);
            double yaw = getYawFromQuadMsg(possible_poses[i].pose.orientation);
            //add a little yaw cost into dist function such that when the same dist
            dist+=0.1*fabs(yaw);
            if(dist<dist_now)
            {
                nearest_pose = possible_poses[i];
                dist_now = dist;
                //cout<<dist_now<<" "<<dist<<" "<<possible_poses[i].pose.position.x<<" "<<possible_poses[i].pose.position.y<<" "<<getYawFromQuadMsg(possible_poses[i].pose.orientation)<<endl;
            }
        }
        if(dist_now<3.0) cur_pose_ = nearest_pose;
        else cout<<"Not updating pose"<<endl;
        //cout<<"-------------end----------"<<endl;



        double yaw = getYawFromQuadMsg(cur_pose_.pose.orientation);
        double yaw_filtered = filter_yaw_->filter(cur_pose_.header.stamp.toSec(), yaw);
        double x_filtered = filter_x_->filter(cur_pose_.header.stamp.toSec(), cur_pose_.pose.position.x);
        double y_filtered = filter_y_->filter(cur_pose_.header.stamp.toSec(), cur_pose_.pose.position.y);
        if(isnan(yaw_filtered))
        {
            cout<<"isnan detected, resetting filter"<<endl;
            filter_yaw_->reset();
            yaw_filtered = filter_yaw_->filter(cur_pose_.header.stamp.toSec(), yaw);
        }
        cur_pose_.pose.orientation = getQuadMsgFromYaw(yaw_filtered);
        cur_pose_.pose.position.x = x_filtered;
        cur_pose_.pose.position.y = y_filtered;

        //new added code
        tf_listener_->waitForTransform("/map",cur_pose_.header.frame_id,ros::Time::now(),ros::Duration(0.01));

        tf::StampedTransform trans;
        try{
        // tf_listener_->transformPose("/odom",cur_pose_,cur_new_pose_);
        tf_listener_->lookupTransform("/map",cur_pose_.header.frame_id,ros::Time(0),trans);
        parent_trans = trans;
      //  tf_listener_->transformPose("/odom",ros::Time(0),cur_pose_,"/odom",cur_new_pose_);
        }
        catch (tf::TransformException& e)
        {
        	ROS_ERROR("%s",e.what());
            //return cur_new_pose_;
        	cur_new_pose_.header.seq = 0;
        	return cur_new_pose_;
        }
        //ensure there is a valid yaw

        cur_new_pose_.pose = PoseMultTF(cur_pose_,trans);
        cur_new_pose_.header.frame_id = "/map";
        cur_new_pose_.header.seq = cur_index_ + 1;
        cur_new_pose_.header.stamp = cur_pose_.header.stamp;


        yaw_from_path_ = getYawFromQuadMsg(cur_new_pose_.pose.orientation);
        double dist_interval_ = fmutil::distance<geometry_msgs::Point>(cur_new_pose_.pose.position,last_valid_pose_.pose.position);

        static double dist_interval_thres_ = 0.02; // for the first time, go a longer distance to path differential
        if(dist_interval_ > dist_interval_thres_ &&  (cur_pose_.header.stamp.toSec() - start_time_.toSec() > 500))
        {
            cur_index_ = (cur_index_ + 1)% POSE_HISTORY_NUM;
            if(cur_index_ >= (POSE_HISTORY_NUM -1))
            {
                 is_history_full_ = true;
            }
            pose_history_[cur_index_] = cur_new_pose_;
            last_valid_pose_ = cur_new_pose_;
            cout<<"add new point"<<endl;
            if(is_history_full_ == true)
            {
            	dist_interval_thres_ = 0.01;
            }
        }



         if(is_history_full_ == true)
         {
        	 if(cur_pose_.header.frame_id == "/robot_1/base_link")
        	  {
        	   double temp_y, temp_p,temp_r;
        	   double temp_min_y, temp_min_dist,temp_dist,temp_test_y;
        	   btMatrix3x3(trans.getRotation().asBt()).getEulerYPR(temp_y,temp_p,temp_r);
        	   temp_min_y = temp_y;
        	   temp_min_dist = 1e9;
        	   tf::Quaternion temp_q;
        	   pose_history_[cur_index_].pose.position = pose_history_[(cur_index_+ POSE_HISTORY_NUM -1)%POSE_HISTORY_NUM].pose.position;
        	  // pose_history_[cur_index_].pose.position.x =2 *pose_history_[(cur_index_+ POSE_HISTORY_NUM -1)%POSE_HISTORY_NUM].pose.position.x
        		//	    - pose_history_[(cur_index_+ POSE_HISTORY_NUM -2)%POSE_HISTORY_NUM].pose.position.x;
        	  // pose_history_[cur_index_].pose.position.y =2 *pose_history_[(cur_index_+ POSE_HISTORY_NUM -1)%POSE_HISTORY_NUM].pose.position.y
        	    //       			    - pose_history_[(cur_index_+ POSE_HISTORY_NUM -2)%POSE_HISTORY_NUM].pose.position.y;
        	   for(int temp_i = -5; temp_i < 5; temp_i++)
        	   {
                     temp_test_y = temp_y + 0.5* temp_i * M_PI / 180.0;
                     temp_q.setRPY(0,0,temp_test_y);
                     trans.setRotation(temp_q);
                     temp_dist = fmutil::distance<geometry_msgs::Point>(PoseMultTF(cur_pose_,trans).position,
                    		 pose_history_[cur_index_].pose.position);
                     if(temp_dist < temp_min_dist)
                     {
                    	 temp_min_dist = temp_dist;
                    	 temp_min_y = temp_test_y;
                     }
        	   }
        	   temp_q.setRPY(0,0,temp_min_y);
        	   trans.setRotation(temp_q);

				   cur_new_pose_.pose = PoseMultTF(cur_pose_,trans);
				   pose_history_[cur_index_] = cur_new_pose_;
				   for(int ii = 0; ii < 100 ; ii++)
				   {
				     // trans.stamp_ = ros::Time::now() ;
				     // tf_broadcaster_->sendTransform(trans);
				   }
        	   }

               //TODO get the yaw angle by fitting the points to a small line
        	   geometry_msgs::Point p;
        	   p.x = pose_history_[cur_index_].pose.position.x  -  pose_history_[(cur_index_ + 1)%POSE_HISTORY_NUM].pose.position.x;
        	   p.y = pose_history_[cur_index_].pose.position.y - pose_history_[(cur_index_ + 1)%POSE_HISTORY_NUM].pose.position.y;
        	   yaw_from_path_ = findYawLeastSquare(pose_history_,p);



         }
         else
         {
        	 yaw_from_path_ = yaw_from_path_;
         }
         /*
         double new_yaw_filtered = filter_new_yaw_->filter(cur_pose_.header.stamp.toSec(), yaw_from_path_);
         if(isnan(new_yaw_filtered))
         {
                  cout<<"isnan detected, resetting filter"<<endl;
                  filter_new_yaw_->reset();
                  new_yaw_filtered = filter_new_yaw_->filter(cur_pose_.header.stamp.toSec(), yaw);
         }
         if(is_history_full_ == true && (cur_pose_.header.stamp.toSec() - last_update_time_.toSec()) > 1.0)
         {
        	 //nothing to do
        	 yaw_from_path_ = yaw_from_path_;
         }
         else
         {
            yaw_from_path_ = new_yaw_filtered;
         }
         */
        //yaw_from_path_ = ave_filter.update(yaw_from_path_);

        /* if(cur_pose_.header.frame_id != "/robot_1/base_link")
         {
			 if((cur_pose_.header.stamp.toSec() - start_time_.toSec() > 180))
			 {
				yaw_from_path_ = thres_filter.update(yaw_from_path_,cur_pose_.header.stamp.toSec(),M_PI/180*1.0);
			 }
			 else
			 {
				 yaw_from_path_ = thres_filter.update(yaw_from_path_,cur_pose_.header.stamp.toSec(),M_PI/180*1.0);
			 }
         }*/
        is_found_ = false;
        int searching_index = 0;
        int temp_search_index = 0;
        double min_search_dist = 1e9;
        double temp_search_dist = 0;
        for(int temp_index = 0; temp_index < 2 * search_area_ ;temp_index++)
        {
             searching_index = temp_index + ref_index_ -  search_area_ ;
             if(searching_index < 0)
            	 continue;

             if(searching_index > (ref_pose_array_.size() - 1))
            	 break;

             temp_search_dist = compute_dist(ref_pose_array_[searching_index].x,ref_pose_array_[searching_index].y,
            		 cur_new_pose_.pose.position.x,cur_new_pose_.pose.position.y);
             if(min_search_dist > temp_search_dist)
             {
            	 min_search_dist = temp_search_dist;
            	 temp_search_index = searching_index;
            	 is_found_ = true;
             }

             //cout<<"searching index is "<<searching_index<<endl;

        }
        if(is_found_)
        {
        	cout<<"referencing"<<endl;
        	ref_index_ = temp_search_index;
        	ref_angle_ = ref_pose_array_[ref_index_].th;
        	ref_angle_= convertToPi(ref_angle_);
        	yaw_from_path_ = convertToPi(yaw_from_path_);

        	if(fabs(ref_angle_ - yaw_from_path_) >  M_PI/180*10)
        	{
        		yaw_from_path_ = ref_angle_;
        	}
        	else if(fabs(ref_angle_ - yaw_from_path_) >  M_PI/180*5)
        	{
        		yaw_from_path_ = ref_angle_ * 0.5 + yaw_from_path_ *0.5;
        	}
        	else
        	{
        		yaw_from_path_ = ref_angle_ * 0.2 + yaw_from_path_ *0.8;
        	}


        	//find the nearest point to stay on the road with the laser distance constraint
        	if(cur_pose_.header.frame_id == "/robot_1/base_link")
			  {
			   double temp_y, temp_p,temp_r;
			   double temp_min_y, temp_min_dist,temp_dist,temp_test_y;
			   btMatrix3x3(trans.getRotation().asBt()).getEulerYPR(temp_y,temp_p,temp_r);
			   temp_min_y = temp_y;
			   temp_min_dist = 1e9;
			   tf::Quaternion temp_q;
			   for(int temp_i = -20; temp_i < 20; temp_i++)
			   {
					 temp_test_y = temp_y + 0.5 * temp_i * M_PI / 180.0;
					 temp_q.setRPY(0,0,temp_test_y);
					 trans.setRotation(temp_q);
					 temp_dist = compute_dist(PoseMultTF(cur_pose_,trans).position.x,PoseMultTF(cur_pose_,trans).position.y,
							 ref_pose_array_[ref_index_].x,ref_pose_array_[ref_index_].y);
					 if(temp_dist < temp_min_dist)
					 {
						 temp_min_dist = temp_dist;
						 temp_min_y = temp_test_y;
					 }
			   }
			   temp_q.setRPY(0,0,temp_min_y);
			   trans.setRotation(temp_q);
			   ros::Time now_time = ros::Time::now();
			   ros::Duration delta_t =  now_time - cur_pose_.header.stamp;
			   delta_t.fromNSec(delta_t.toNSec()/2);
			   geometry_msgs::PoseStamped temp_pose_msg;
			   temp_pose_msg.header.stamp = cur_pose_.header.stamp;
			   temp_pose_msg.pose.orientation = getQuadMsgFromYaw(temp_min_y);


			   //trans.stamp_ = ros::Time::now();
			   trans.stamp_ = cur_pose_.header.stamp;
			   tf_broadcaster_->sendTransform(trans);
			   for(int ii = 1;ii < 30;ii++)
			   {
				   trans.stamp_ = trans.stamp_ + ros::Duration(0.001);
				   tf_broadcaster_->sendTransform(trans);
			   }




			   robot_1_tf_pub_.publish(temp_pose_msg);

			   cur_new_pose_.pose = PoseMultTF(cur_pose_,trans);
/*
				   for(int ii = 0; ii <  10; ii++)
				   {
					  now_time = now_time - delta_t;
					  trans.stamp_ = now_time;
					  temp_q.setRPY(0,0,temp_min_y - (temp_min_y - temp_y)*ii/10.0);
					  trans.setRotation(temp_q);
					  tf_broadcaster_->sendTransform(trans);

				   }
				   */

			   }
        }

        cur_new_pose_.pose.orientation = getQuadMsgFromYaw(yaw_from_path_);
        last_update_time_ = cur_pose_.header.stamp;
        last_pose_ = cur_new_pose_;

        //ensure that the transform is valid,namely seq != 0
        cur_new_pose_.header.seq = cur_index_ + 1;
        cout<<"New position= "<<cur_new_pose_.pose.position.x<<" "<<cur_new_pose_.pose.position.y<<" "<<yaw_from_path_<<endl;
        return cur_new_pose_;
        //return cur_pose_;

    }

};
