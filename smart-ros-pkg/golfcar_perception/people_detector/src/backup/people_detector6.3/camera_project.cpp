#include <camera_project.h>

/*
 * version3--labeled_objects,laser & camera fusion. Corresponding to people_detect-5.0
 */ 

namespace camera_projector{
	
	camera_project::camera_project()
	{
	
    ROS_INFO("projection begins");
    ros::NodeHandle nh;
    
    //"lb_objs" provides distance information, further changed into pixel information;
    lb_objs_sub=nh.subscribe("labeled_objs_cloud", 2, &camera_project::project_to_image, this);
    
    //"prs" is sended to "HOG" classifier, providing corresponding pixel information;
    //"HOG" classifier will change its "decision flag", then send it back;
    pr_vector_pub = nh.advertise<people_detector::people_rects>("pr_vector", 2);
    }
	
	void camera_project::project_to_image(const people_detector::labeled_objs& lb_objs_para) 
	{
		prs.pr_vector.clear();
		
		for(int i=0; i<(int)lb_objs_para.lb_objs_vector.size(); i++)
		{
		geometry_msgs::Point32 temp3Dpoint;
		
		people_detector::people_rect temppr; 

		//Transformation from "sick_laser" to "webcam". Here the unit of length is meter.
		temp3Dpoint.x=-lb_objs_para.lb_objs_vector[i].pedestrian_point.y;   //Here should be minus.
		temp3Dpoint.y=lb_objs_para.lb_objs_vector[i].pedestrian_point.z-0.17;  //with fixed differencials
		temp3Dpoint.z=lb_objs_para.lb_objs_vector[i].pedestrian_point.x-0.07; 
		
		camera_project::projection(temp3Dpoint, temppr);
		
		/* temppr point is just the breast point in the coordinate of the image;
		 * the following part is to get the pixel of the rectangle corner.
		 */
		
		if(temppr.x>0&&temppr.x<1280&&temppr.y>0&&temppr.y<1024) //at least the breast point is inside of the picture.
		{
		people_detector::people_rect tempprcorner;
		tempprcorner.decision_flag = false;    //set false as default value; need further to be changed by Demian's HOG.
		tempprcorner.complete_flag = true;
		
		tempprcorner.object_label=lb_objs_para.lb_objs_vector[i].object_label;
		
		//Here "+-40" serves as allowance for calibration error.
		//to get tempprcorner.x
		if((temppr.x-(int)(0.5/temppr.disz*webcam.fc[0])-40)<0)
		{tempprcorner.x=0;
		 //tempprcorner.complete_flag=false;
		 ROS_INFO("1-----set to false");}
		 
		else
		{tempprcorner.x=temppr.x-(int)(0.5/temppr.disz*webcam.fc[0])-40;}
		
		//to get tempprcorner.width
		if(((int)(0.5/temppr.disz*webcam.fc[0])+temppr.x+40)>1280)
		{tempprcorner.width=1280-tempprcorner.x;
		ROS_INFO("2-----set to false");}
		
		else
		{tempprcorner.width=(int)(0.5/temppr.disz*webcam.fc[0])+temppr.x+40-tempprcorner.x;}		 

		
		//to get tempprcorner.y
		if((temppr.y-(int)(1.2/temppr.disz*webcam.fc[0])-40)<0)
		{tempprcorner.y=0;
		ROS_INFO("3-----set to false");}
		
		else
		{tempprcorner.y=temppr.y-(int)(1.2/temppr.disz*webcam.fc[0])-40;}
		
		//to get tempprcorner.height
		if(((int)(1.8/temppr.disz*webcam.fc[0])+temppr.y+40)>1024)
		{tempprcorner.height=1024-tempprcorner.y;
		 ROS_INFO("4-----can set to false, but not set");
		 }
		 
		else
		{ROS_INFO("4-----is passed");
		
		tempprcorner.height=(int)(1.8/temppr.disz*webcam.fc[0])+temppr.y+40-tempprcorner.y;}		 
		
		tempprcorner.scaled_x=(int)((webcam.scaled_image_width/webcam.raw_image_width)*tempprcorner.x);
		tempprcorner.scaled_y=(int)((webcam.scaled_image_height/webcam.raw_image_height)*tempprcorner.y);
		tempprcorner.scaled_width=(int)((webcam.scaled_image_width/webcam.raw_image_width)*tempprcorner.width);
		tempprcorner.scaled_height=(int)((webcam.scaled_image_height/webcam.raw_image_height)*tempprcorner.height);
		
		
		//distance in Z direction in "laser" coordinate.
		tempprcorner.disz=temppr.disz;
		
		/*
		//pedestrians 3D position in "map" coordinate.
		tempprcorner.positionx=pedestrians_cloud_para->points[i].x;
		tempprcorner.positiony=pedestrians_cloud_para->points[i].y;
		tempprcorner.positionz=pedestrians_cloud_para->points[i].z;
		*/
		
		//2 conditions for credible judgement. Below is the 2nd one.
		if(tempprcorner.disz>=15||tempprcorner.disz<=2){tempprcorner.complete_flag=false;}  
		
		if(tempprcorner.complete_flag==true)
		{ROS_INFO("--------------------Credible Judge---------------------");}
		
		prs.pr_vector.push_back(tempprcorner); 
		}
		
	    }
		pr_vector_pub.publish(prs);
	}
	
	void camera_project::projection(const geometry_msgs::Point32& temp3Dpara, people_detector::people_rect& tempprpara)
	{
		if(temp3Dpara.z!=0)    //in case of infinity.
		{
		  double xn[2]={0,0};
		  xn[0]=temp3Dpara.x/temp3Dpara.z;	//x
		  xn[1]=temp3Dpara.y/temp3Dpara.z;	//y
		  
		  double r2power=0;	//2nd power of r
		  r2power=xn[0]*xn[0]+xn[1]*xn[1];
		  
		  double dx[2]={0,0};
		  dx[0]=2*(webcam.kc[2])*xn[0]*xn[1]+(webcam.kc[3])*(r2power+2*xn[0]*xn[0]);
		  dx[1]=(webcam.kc[2])*(r2power+2*xn[0]*xn[0])*2*(webcam.kc[3])*xn[0]*xn[1];	  
		  
		  double xd[2]={0,0};
		  xd[0]=(1+webcam.kc[0]*r2power+webcam.kc[1]*r2power*r2power+webcam.kc[4]*r2power*r2power*r2power)*xn[0]+dx[0];
		  xd[1]=(1+webcam.kc[0]*r2power+webcam.kc[1]*r2power*r2power+webcam.kc[4]*r2power*r2power*r2power)*xn[1]+dx[1];
		  
		  int breast_x, breast_y;
		  breast_x=(int)(webcam.fc[0]*xd[0]+webcam.alpha_c*webcam.fc[0]*xd[1]+webcam.cc[0]);  //temp2Dpara is Point32.
		  breast_y=(int)(webcam.fc[1]*xd[1]+webcam.cc[1]);
	
		  /*
		  if(breast_x>0&&breast_x<1280&&breast_y>0&&breast_y<1024)		  
		  {ROS_INFO("x axis pixel %d, y axis pixel %d, distance z %3f", breast_x, breast_y, temp3Dpara.z);}
		  */   
		  		  
		  tempprpara.x=breast_x;
		  tempprpara.y=1024-breast_y;         //change coordinate;
		  tempprpara.disz=temp3Dpara.z;
		  
		  /*
		  tempprpara.width=(int)(1/tempprpara.disz*webcam.fc[0]);  		//not used any more. Just for debugging.
		  tempprpara.height=(int)(3/tempprpara.disz*webcam.fc[1]);   
		  */
	    }
	    
	    else
	    {
			tempprpara.x=10000;
			tempprpara.y=10000;
			tempprpara.disz=0;
			tempprpara.height=0;
			tempprpara.width=0;
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

