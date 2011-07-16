#include <project_test.h>

namespace camera_projector{
	
	camera_project::camera_project()
	{
		geometry_msgs::Point32 temp3Dpoint;
		geometry_msgs::Point32 temp2Dpoint;
		
		temp3Dpoint.x=0;
		temp3Dpoint.y=0.216;
		temp3Dpoint.z=0.3;	
		
		camera_project::projection(temp3Dpoint, temp2Dpoint);
		ROS_INFO("temp2Dpoint (x,y,z):(%5f, %5f,%5f)",temp2Dpoint.x,temp2Dpoint.y,temp2Dpoint.z);
    }

	
	void camera_project::projection(const geometry_msgs::Point32& temp3Dpara, geometry_msgs::Point32& temp2Dpara)
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
		  
		  temp2Dpara.x=webcam.fc[0]*xd[0]+webcam.alpha_c*webcam.fc[0]*xd[1]+webcam.cc[0];
		  temp2Dpara.y=webcam.fc[1]*xd[1]+webcam.cc[1];
		  temp2Dpara.z=0;
		  ROS_INFO("temp2Dpoint (x,y,z):(%5f, %5f,%5f)",temp2Dpara.x,temp2Dpara.y,temp2Dpara.z);
	}
	
}

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "camera_projector");
	 
	 camera_projector::camera_project* camera_project_node;
	 camera_project_node = new camera_projector::camera_project();
	 
	 ros::spin();
}

