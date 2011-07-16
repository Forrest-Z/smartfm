#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <tf/transform_listener.h>
#include <cmath>

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

namespace camera_projector{

     class camera_calib
     {
       public:
       double fc[2]; 	//focal_length
       double cc[2];  	//principal_point
       double alpha_c; 	//skew_coeff
       double kc[5];	//distortions

       //fill in corresponding parameters here.
       camera_calib()
       {
	fc[0]=1407.34674;
	fc[1]=1411.80220;

	cc[0]=617.72473;
        cc[1]=456.39705;

	alpha_c=0;

	kc[0]=-0.05879;
	kc[1]=-0.43182;
	kc[2]=-0.01582;
	kc[3]=-0.00892;
	kc[4]=0;
       }

       ~camera_calib(){}
      }; 

     
     class camera_project
     {
       public:
       camera_project();
       ~camera_project();

       private:
       void projection(const geometry_msgs::Point32& temp3Dpara, geometry_msgs::Point32& temp2Dpara);
       camera_calib webcam;
      };
};
       
       
       

       
