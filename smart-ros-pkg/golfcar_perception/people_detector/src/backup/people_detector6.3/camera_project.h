#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <tf/transform_listener.h>
#include <cmath>

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <people_detector/people_rect.h>
#include <people_detector/people_rects.h>

#include "people_detector/labeled_obj.h"
#include "people_detector/labeled_objs.h"

//version3--labeled_objects,laser & camera fusion. Corresponding to people_detect-5.0

namespace camera_projector{

     class camera_calib
     {
       public:
       double fc[2]; 	//focal_length
       double cc[2];  	//principal_point
       double alpha_c; 	//skew_coeff
       double kc[5];	//distortions
       float raw_image_width;
       float raw_image_height;
       float scaled_image_width;
       float scaled_image_height;

       //fill in corresponding parameters here.
     
      //try and error;
       camera_calib()
       {
	fc[0]=1394.70152;
	fc[1]=1393.64149;

	cc[0]=648.64173;
        cc[1]=400.00655;

	alpha_c=0;

	kc[0]=-0.05849;
	kc[1]=-0.00812;
	kc[2]=0.00437;
	kc[3]=0.00926;
	kc[4]=0;

	raw_image_width=1280;
        raw_image_height=1024;
	scaled_image_width=640;
	scaled_image_height=480;
       }

       ~camera_calib(){}
      };

     
     class camera_project
     {
       public:
       camera_project();
       ~camera_project();

       private:
       //void project_to_image(const sensor_msgs::PointCloud& pedestrians_cloud_para); //serve as "callback"
       void project_to_image(const people_detector::labeled_objs& lb_objs_para); 

       void projection(const geometry_msgs::Point32& temp3Dpara, people_detector::people_rect& tempprpara);
       ros::Publisher pr_vector_pub;
       camera_calib webcam;
       //tf::TransformListener map_to_sick;
       //sensor_msgs::PointCloud ped3D_sick;

       people_detector::people_rects prs;
       ros::Subscriber lb_objs_sub;
      };
};
       
       
       

       
