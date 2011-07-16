#include <camera_project.h>

/*
 * version2.2
 */ 

namespace camera_projector{
	
	camera_project::camera_project()
	{
	
    ROS_INFO("projection begins");
    ros::NodeHandle nh;
    //ped3D_sub=nh.subscribe("pedestrians_cloud", 2, &camera_project::project_to_image, this); //pay attention to usage here
   
    ped3D_sub.subscribe(nh, "pedestrians_cloud", 10);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(ped3D_sub, tf_, "sick_laser", 1);
	tf_filter_->registerCallback(boost::bind(&camera_project::project_to_image, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.05));
	
    pr_vector_pub = nh.advertise<people_detector::people_rects>("pr_vector", 2);
    }
	
	void camera_project::project_to_image(const boost::shared_ptr<const sensor_msgs::PointCloud>& pedestrians_cloud_para) 
	{
		prs.pr_vector.clear();
		
		ROS_INFO("pedestrian cloud subscribed");
		
		try 
		{
		tf_.transformPointCloud("sick_laser", *pedestrians_cloud_para, ped3D_sick);
		}
        
		catch (tf::TransformException &ex) 
		{
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
		
		for(int i=0; i<(int)ped3D_sick.points.size(); i++)
		{
		geometry_msgs::Point32 temp3Dpoint;
		people_detector::people_rect temppr; 
		
		//Transformation from "sick_laser" to "webcam". Here the unit of length is meter.
		temp3Dpoint.x=-ped3D_sick.points[i].y;   //Here should be minus.
		temp3Dpoint.y=ped3D_sick.points[i].z-0.17;
		temp3Dpoint.z=ped3D_sick.points[i].x;	
		
		
		camera_project::projection(temp3Dpoint, temppr);
		
		/* temppr point is just the breast point in the coordinate of the image;
		 * the following part is to get the pixel of the rectangle corner.
		 */
		if(temppr.x>0&&temppr.x<1280&&temppr.y>0&&temppr.y<1024) //at least the breast point is inside of the picture.
		{
		people_detector::people_rect tempprcorner;
		
		//Here "+-50" serves as allowance for calibration error.
		
		//to get tempprcorner.x
		if((temppr.x-(int)(0.5/temppr.disz*webcam.fc[0])-50)<0)
		{tempprcorner.x=0;}
		else
		{tempprcorner.x=temppr.x-(int)(0.5/temppr.disz*webcam.fc[0])-50;}
		
		//to get tempprcorner.width
		if(((int)(0.5/temppr.disz*webcam.fc[0])+temppr.x+50)>1280)
		{tempprcorner.width=1280-tempprcorner.x;}
		else
		{tempprcorner.width=(int)(0.5/temppr.disz*webcam.fc[0])+temppr.x+50-tempprcorner.x;}		 
		
		//to get tempprcorner.y
		if((temppr.y-(int)(1.2/temppr.disz*webcam.fc[0])-50)<0)
		{tempprcorner.y=0;}
		else
		{tempprcorner.y=temppr.y-(int)(1.2/temppr.disz*webcam.fc[0])-50;}
		
		//to get tempprcorner.height
		if(((int)(1.8/temppr.disz*webcam.fc[0])+temppr.y+50)>1024)
		{tempprcorner.height=1024-tempprcorner.y;}
		else
		{tempprcorner.height=(int)(1.8/temppr.disz*webcam.fc[0])+temppr.y+50-tempprcorner.y;}		 
		
		tempprcorner.scaled_x=(int)((webcam.scaled_image_width/webcam.raw_image_width)*tempprcorner.x);
		tempprcorner.scaled_y=(int)((webcam.scaled_image_height/webcam.raw_image_height)*tempprcorner.y);
		tempprcorner.scaled_width=(int)((webcam.scaled_image_width/webcam.raw_image_width)*tempprcorner.width);
		tempprcorner.scaled_height=(int)((webcam.scaled_image_height/webcam.raw_image_height)*tempprcorner.height);
		
		
		//distance in Z direction in "laser" coordinate.
		tempprcorner.disz=temppr.disz;
		
		//pedestrians 3D position in "map" coordinate.
		tempprcorner.positionx=pedestrians_cloud_para->points[i].x;
		tempprcorner.positiony=pedestrians_cloud_para->points[i].y;
		tempprcorner.positionz=pedestrians_cloud_para->points[i].z;
		
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
	
		  if(breast_x>0&&breast_x<1280&&breast_y>0&&breast_y<1024)		  
		  {ROS_INFO("x axis pixel %d, y axis pixel %d, distance z %3f", breast_x, breast_y, temp3Dpara.z);}
		  
		  tempprpara.x=breast_x;
		  tempprpara.y=1024-breast_y;         //change coordinate;
		  tempprpara.disz=temp3Dpara.z;
		  
		  tempprpara.width=(int)(1/tempprpara.disz*webcam.fc[0]);
		  tempprpara.height=(int)(3/tempprpara.disz*webcam.fc[1]);   
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

