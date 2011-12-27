/*
 * This class is to project object position in 3D world onto 2D image;
 * Reference: "Fast Extrinsic Calibration of a Laser Rangefinder to a Camera"
 * http://www.ri.cmu.edu/publication_view.html?pub_id=5293
 */ 


#include "camera_project.h"

using namespace std;
using namespace ros;

#define MAX_HEIGHT 2.0
#define PIXEL_ALLOWANCE 30
#define LASER_MOUNTING_HEIGHT 1.0

namespace camera_projector{
	
    camera_project::camera_project()
	{
	    private_nh_.param("laser_frame_id",     ldmrs_single_id_,      std::string("ldmrsAssemb"));
        private_nh_.param("camera_frame_id",    camera_frame_id_,      std::string("usb_cam"));
        
        pd_pcl_sub_.subscribe(nh_, "ped_laser_pcl", 10);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pd_pcl_sub_, tf_, camera_frame_id_, 10);
        tf_filter_->registerCallback(boost::bind(&camera_project::pcl_callback, this, _1));
        tf_filter_->setTolerance(ros::Duration(0.05));
        pd_laser_batch_sub_ = nh_.subscribe("ped_laser_batch", 2, &camera_project::project_to_image, this);
        pd_vision_pub_ = nh_.advertise<sensing_on_road::pedestrian_vision_batch>("pd_vision_batch", 2);
    }
    
    void camera_project::pcl_callback(const sensor_msgs::PointCloud::ConstPtr& pcl_in){}
	
	void camera_project::project_to_image(const sensing_on_road::pedestrian_laser_batch &pd_laser_para)
	{
        ROS_INFO("Camera project to image");
        pd_vision_batch_.header = pd_laser_para.header;
		pd_vision_batch_.pd_vector.clear();
		
		for(unsigned int i=0; i<pd_laser_para.pedestrian_laser_features.size(); i++)
		{
            sensing_on_road::pedestrian_vision temppr;
            
            //transform from "sick_laser" to "webcam" of ROS-convension coordinate; 
            geometry_msgs::PointStamped stamped_point;
            try{ tf_.transformPoint(camera_frame_id_, pd_laser_para.pedestrian_laser_features[i].pedestrian_laser, stamped_point);}
            catch (tf::TransformException& e){ROS_INFO("camera project tf error");std::cout << e.what();return;}
            
            //"webcam" further tranform from ROS-convension to Vision-convension;
            geometry_msgs::Point32 centroid_point;
            centroid_point.x   =   -stamped_point.point.y;    
            centroid_point.y   =    stamped_point.point.z;    
            centroid_point.z   =    stamped_point.point.x;
            camera_project::projection(centroid_point, temppr);
            ROS_INFO("temppr %d, %d, %3f", temppr.x, temppr.y, temppr.disz);
            //the central point should fall inside of the picture.
            if(temppr.x>0&&temppr.x<webcam_.raw_image_width &&temppr.y>0&&temppr.y<webcam_.raw_image_height) 
            {
                //tempprcorner represents a rectangle box with corner.x, corner,y, height and width;
                sensing_on_road::pedestrian_vision tempprcorner;

                tempprcorner.decision_flag = false;    
                tempprcorner.complete_flag = true;
                tempprcorner.object_label  = pd_laser_para.pedestrian_laser_features[i].object_label;

                float dx_coef     =  webcam_.fc[0]/temppr.disz;
                float dy_coef     =  webcam_.fc[1]/temppr.disz;
                float size_dim    =  pd_laser_para.pedestrian_laser_features[i].size;

                int   conner_x    =  temppr.x - (int)(dx_coef*size_dim/2) - PIXEL_ALLOWANCE;
                int   conner_y    =  temppr.y - (int)(dx_coef*LASER_MOUNTING_HEIGHT) - PIXEL_ALLOWANCE;
                int   upper_x     =  temppr.x + (int)(dy_coef*size_dim/2) + PIXEL_ALLOWANCE;
                int   upper_y     =  temppr.y + (int)(dy_coef*(MAX_HEIGHT-LASER_MOUNTING_HEIGHT)) + PIXEL_ALLOWANCE;
                
                if(conner_x<0){tempprcorner.x=0;}
                else {tempprcorner.x=conner_x;}
                if(upper_x>webcam_.raw_image_width){tempprcorner.width=webcam_.raw_image_width-tempprcorner.x;}
                else{tempprcorner.width=upper_x-tempprcorner.x;}		 

                if(conner_y<0){tempprcorner.y=0;}
                else {tempprcorner.y = conner_y;}
                if(upper_y>webcam_.raw_image_height) {tempprcorner.height = webcam_.raw_image_height-tempprcorner.y;}
                else{tempprcorner.height=upper_y-tempprcorner.y;}		 
                
                tempprcorner.disz=temppr.disz;
                
                if(tempprcorner.disz>=20||tempprcorner.disz<=0.5){tempprcorner.complete_flag = false;}  
                if(tempprcorner.complete_flag==true){pd_vision_batch_.pd_vector.push_back(tempprcorner);}
            }
        }
        pd_vision_pub_.publish(pd_vision_batch_);
	}
	
	void camera_project::projection(const geometry_msgs::Point32 &temp3Dpara, sensing_on_road::pedestrian_vision &tempprpara)
	{ 
		if(temp3Dpara.z >0.0)   
		{
            double xn[2]={0,0};
            xn[0]=temp3Dpara.x/temp3Dpara.z;	
            xn[1]=temp3Dpara.y/temp3Dpara.z;	
		  
            double r2power=xn[0]*xn[0]+xn[1]*xn[1];
            double dx[2]={0,0};
            dx[0]=2*(webcam_.kc[2])*xn[0]*xn[1]+(webcam_.kc[3])*(r2power+2*xn[0]*xn[0]);
            dx[1]=(webcam_.kc[2])*(r2power+2*xn[0]*xn[0])*2*(webcam_.kc[3])*xn[0]*xn[1];
            double xd[2]={0,0};
            xd[0]=(1+webcam_.kc[0]*r2power+webcam_.kc[1]*r2power*r2power+webcam_.kc[4]*r2power*r2power*r2power)*xn[0]+dx[0];
            xd[1]=(1+webcam_.kc[0]*r2power+webcam_.kc[1]*r2power*r2power+webcam_.kc[4]*r2power*r2power*r2power)*xn[1]+dx[1];
		  
            int pixel_x, pixel_y;
            pixel_x=(int)(webcam_.fc[0]*xd[0]+webcam_.alpha_c*webcam_.fc[0]*xd[1]+webcam_.cc[0]); 
            pixel_y=(int)(webcam_.fc[1]*xd[1]+webcam_.cc[1]);
		  		  
            tempprpara.x    = pixel_x;
            tempprpara.y    = webcam_.raw_image_height-pixel_y;        //remember to change coordinate upside-down;
            tempprpara.disz = temp3Dpara.z;
        }
        else
        {
            tempprpara.x=10000;
			tempprpara.y=10000;
		}
	}
	
}

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "camera_projector");
	 
	 camera_projector::camera_project* camera_project_node;
	 camera_project_node = new camera_projector::camera_project();
	 
	 ros::spin();
}

