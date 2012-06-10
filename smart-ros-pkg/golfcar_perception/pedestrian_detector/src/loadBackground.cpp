#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv.h>
#include <highgui.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	ros::init (argc, argv, "backgrounds_pcl");
	ros::NodeHandle nh, n("~");
	string img_file;
	n.param("img_file", img_file, string(""));
	if(img_file.size() == 0) 
	{
		cout<<"No image is provided, please specify through parameter img_file"<<endl;
		return 0;
	}

	Mat img_mat = imread(img_file);
	if(img_mat.rows == 0 || img_mat.cols == 0)
	{
		cout<<"Image is not loaded. Make sure parameter img_file is provided correctly"<<endl;
		return 0;
	}

	ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1, true);

	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "map";
	msg->height =1;
	ros::Time start = ros::Time::now();
	for (int i=0; i<img_mat.rows; i++)
	{
		for(int j=0; j<img_mat.cols; j++)
		{
			pcl::PointXYZRGB p;
			p.b=(int)img_mat.at<Vec3b>( i , j )[0];
			p.g=(int)img_mat.at<Vec3b>( i , j )[1];
			p.r=(int)img_mat.at<Vec3b>( i , j )[2];
			p.data[0]=(float)j*0.1;
			p.data[1]=(img_mat.rows-i)*0.1;
			p.data[2]=-0.02;
			if(p.b==111 && p.g==111 && p.r==111) continue;
			msg->points.push_back (p);
		}
	}
	ros::Duration time_diff = ros::Time::now() - start;
	cout<<"Background Loaded"<<endl;

	msg->width = msg->points.size();
	msg->header.stamp = ros::Time::now ();
	pub.publish (msg);
	ros::spin ();

}
