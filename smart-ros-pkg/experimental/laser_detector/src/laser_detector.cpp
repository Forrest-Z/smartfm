
/*
 *  laser_detector.cpp
 *
 *  Created on: Feb 20, 2013
 *      Author: Shen Xiaotong
 */
#include <ros/ros.h>
#include <queue>
#include <deque>
#include <utility>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>



#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <visualization_msgs/Marker.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>


using std::string;
using std::vector;
using std::cout;
using std::endl;

namespace enc = sensor_msgs::image_encodings;

using namespace cv;

template<class Point_Type>
class cube{
	// 1 2 3 4 correspond to  the foreground rectangle - clockwise
	// 5 6 7 8 correspond to  the background rectangle - clockwise
	// x y (z) is necessary for connecting lines
public:
   Point_Type corners[8];
};

// a class just for projecting the cube to the image
// the original cubes should be in the "/base_link" frame
// input is a vector of cubes(8 points), a switch needs to turn on in order to show the cubes each time
// output is a image shows those rectangles
// tf is needed for projecting the points to the image

class cube_project_to_image
{
   ros::NodeHandle * nh_;
   cube<cv::Point> projected_cube;
   image_transport::ImageTransport it;
   image_transport::Publisher  image_pub;
   image_transport::CameraSubscriber camera_sub;
   image_geometry::PinholeCameraModel camera_model;
   tf::TransformListener * tfl_;
   tf::Transform static_camera_tf;

   string win_name;
   Mat image_rectified;
   tf::Transform target_transfrom;
   tf::Transform result_transform;
   cv::Point3d temp_point;



   bool is_tf_init;

public:
   bool can_show_cube;
   vector< cube<cv::Point3d> > * cube_vector_;



   cube_project_to_image(ros::NodeHandle * node, tf::TransformListener * tf,string topic_name):
	   nh_(node),
	   it(*node),
	   tfl_(tf),
	   win_name("cube_projection"),
	   is_tf_init(false),
	   can_show_cube(false),
	   cube_vector_(NULL),
	   target_transfrom(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0))
   {

	   cv::namedWindow(win_name,CV_WINDOW_NORMAL);
	   camera_sub = it.subscribeCamera(topic_name,1,&cube_project_to_image::Camera_CB,this);
   }

   ~cube_project_to_image()
   {
	   cv::destroyWindow(win_name);
   }

   void Camera_CB(const sensor_msgs::ImageConstPtr & msg_,const sensor_msgs::CameraInfoConstPtr & cam_info_);
   void Draw_Cube(void);



};
void cube_project_to_image::Draw_Cube(void)
{
	cv::line(image_rectified,projected_cube.corners[0],projected_cube.corners[1],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[1],projected_cube.corners[2],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[2],projected_cube.corners[3],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[3],projected_cube.corners[0],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[4],projected_cube.corners[5],CV_RGB(0,0,255));
	cv::line(image_rectified,projected_cube.corners[5],projected_cube.corners[6],CV_RGB(0,0,255));
	cv::line(image_rectified,projected_cube.corners[6],projected_cube.corners[7],CV_RGB(0,0,255));
	cv::line(image_rectified,projected_cube.corners[7],projected_cube.corners[4],CV_RGB(0,0,255));
	cv::line(image_rectified,projected_cube.corners[0],projected_cube.corners[4],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[1],projected_cube.corners[5],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[2],projected_cube.corners[6],CV_RGB(255,0,0));
	cv::line(image_rectified,projected_cube.corners[3],projected_cube.corners[7],CV_RGB(255,0,0));


}

void cube_project_to_image::Camera_CB(const sensor_msgs::ImageConstPtr & msg_,const sensor_msgs::CameraInfoConstPtr & cam_info_)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
	{
	   cv_ptr = cv_bridge::toCvCopy(msg_,enc::BGR8);
	}
	catch(cv_bridge::Exception & e)
	{
	    ROS_ERROR("cv_bridge exception: %s",e.what());
	    return;
	}

	//get the static transform
    if(is_tf_init == false)
    {
         camera_model.fromCameraInfo(cam_info_);

         tf::StampedTransform trans;
		 try{
			 ros::Duration timeout(1.0/30);
			 (*tfl_).waitForTransform(camera_model.tfFrame(),"/base_link",cam_info_->header.stamp,timeout);
			 (*tfl_).lookupTransform(camera_model.tfFrame(),"/base_link",cam_info_->header.stamp,trans);
		 }
		 catch(tf::TransformException & ex)
		 {
			 ROS_ERROR("can not find the transform because of %s",ex.what());
			 return;
		 }

		 static_camera_tf = (tf::Transform) trans;
		 ROS_INFO("camera tf is initialized! \n");
		 is_tf_init = true;
    }
    camera_model.rectifyImage(cv_ptr->image,image_rectified);
    if(can_show_cube)
    {
    	can_show_cube = false;
        if(cube_vector_ == NULL)
        {
        	ROS_ERROR("trying to project empty cubes \n");
        }
        else
        {
        	if(cube_vector_->size() > 0)
        	{


                for(size_t i = 0; i < cube_vector_->size(); i++ )
                {
                	//get the projection
                	for(int j = 0; j < 8; j++)
                	{
                		temp_point = ((*cube_vector_)[i]).corners[j];
                		target_transfrom.setOrigin(tf::Vector3(temp_point.x,temp_point.y,temp_point.z));
                		result_transform = static_camera_tf * target_transfrom;
                		temp_point.x = result_transform.getOrigin().x();
                		temp_point.y = result_transform.getOrigin().y();
                		temp_point.z = result_transform.getOrigin().z();


                		projected_cube.corners[j] = camera_model.project3dToPixel(temp_point);

                	}
                	//draw the cubes
                	cout<<"drawing the "<<i<<"th cube"<<endl;
                	Draw_Cube();
                }
        	}
        }
    }

    cv::imshow(win_name,image_rectified);
    // waitkey  to view is essential
    cv::waitKey(1);

}


class laser_accu{


	tf::TransformBroadcaster * tfb_;

	string  odom_frame,laser_frame,base_frame;

	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
	tf::MessageFilter<sensor_msgs::LaserScan> * tf_laser_filter_;


	sensor_msgs::LaserScan scan_laser_in;
	sensor_msgs::PointCloud scan_pts_in;
	laser_geometry::LaserProjection laser_proj;
	ros::Time start_time_;
	bool start_flag_;
	bool full_flag_;
	ros::Duration interval_time_;


	std::deque<sensor_msgs::PointCloud> laser_accu_;
	sensor_msgs::PointCloud laser_accu_cloud_;
	unsigned int max_queue_size_;
	ros::Publisher laser_accu_pub_;
	ros::Publisher laser_3d_cluster_pub_;
	ros::Publisher marker_pub_;

public:
	ros::NodeHandle * nh_;
    tf::TransformListener * tfl_;
    string camera_topic_name;
    cube_project_to_image * projection_;
    vector< cube<cv::Point3d> > cubes;




public:
    laser_accu(void):
		nh_(new ros::NodeHandle),
		tfl_(new tf::TransformListener),
		tfb_(new tf::TransformBroadcaster),
	    start_flag_(false),
	    full_flag_(false),
	    max_queue_size_(50)
	{
        ros::NodeHandle private_nh("~");

        private_nh.param("odom_frame",odom_frame,string("/odom"));
        private_nh.param("laser_frame",laser_frame,string("/sick"));
        private_nh.param("base_frame",base_frame,string("/base_link"));
        private_nh.param("camera_topic_name",camera_topic_name,string("/camera_front/image_raw"));

        laser_sub.subscribe(*nh_,"/laser_in",10);
        tf_laser_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub,*tfl_,odom_frame,10);
        tf_laser_filter_->registerCallback(boost::bind(&laser_accu::scanCallback,this,_1));

        laser_accu_pub_ = nh_->advertise<sensor_msgs::PointCloud>("laser_accu_pts",5,false);
        laser_3d_cluster_pub_ = nh_->advertise<sensor_msgs::PointCloud>("laser_cluster_pts",5,false);
         marker_pub_ = nh_->advertise<visualization_msgs::Marker>("moving_marker",20);

         projection_ = new cube_project_to_image(nh_,tfl_,camera_topic_name);


	}
	~laser_accu(void)
	{
         if(nh_  != NULL)  delete nh_;
         if(tfl_ != NULL) delete tfl_;
         if(tfb_ != NULL) delete tfb_;
         if(projection_ != NULL) delete projection_;
	}
private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr msg_);
	void cluster_laser_pts(sensor_msgs::LaserScan & msg);
};

void cluster_laser_pts(sensor_msgs::LaserScan & msg)
{
    return;
}
void laser_accu::scanCallback(const sensor_msgs::LaserScan::ConstPtr msg_)
{
      cube<cv::Point3d> temp_cube;
      cubes.clear();

      double length = 15;
      double width  = 4;
      double height = 2.5;
      double center_x = 15;
      double center_y = 0;
      double center_z = height/2;

      for(int i = 0 ; i < 8 ; i++)
      {
    	  temp_cube.corners[i].x = center_x + ((i/4==0)? -1:1) * length /2;
    	  temp_cube.corners[i].z = center_z + ((i%4>=2)? -1:1) * height /2;
    	  temp_cube.corners[i].y = center_y + (((i%4==2 || i%4==1)) ? -1:1) * width /2;

      }

      cubes.push_back(temp_cube);

      length = 0.5;
      width  = 0.5;
      height = 1.6;
      center_x = 10;
      center_y = 2;
      center_z = height/2;
       for(int i = 0 ; i < 8 ; i++)
	   {
		  temp_cube.corners[i].x = center_x + ((i/4==0)? -1:1) * length /2;
		  temp_cube.corners[i].z = center_z + ((i%4>=2)? -1:1) * height /2;
		  temp_cube.corners[i].y = center_y + (((i%4==2 || i%4==1)) ? -1:1) * width /2;

	   }
      cubes.push_back(temp_cube);

      projection_->cube_vector_ = & cubes;
      projection_->can_show_cube = true;



}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"LaserMovement");
    laser_accu laser_accumulation;

    ros::spin();
	return 0;
}
