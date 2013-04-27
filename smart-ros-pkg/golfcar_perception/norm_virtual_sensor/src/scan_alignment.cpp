#include "AccumulateData.h"

class ScanAlignment
{
public:
	ros::Publisher scan_aligned_pub_, acc_scan_pub_, acc_pcl_pub_;
	tf::TransformListener *tf_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan>       *laser_scan_filter_;
	message_filters::Subscriber<sensor_msgs::PointCloud> pc_scan_sub_;
	tf::MessageFilter<sensor_msgs::PointCloud> *pc_scan_filter_;
	laser_geometry::LaserProjection projector_;
	AccumulateData *ad_;
	ScanAlignment() : tf_(new tf::TransformListener()), ad_(new AccumulateData("odom", 0.05, 10))
	{
		ros::NodeHandle nh;
		laser_scan_sub_.subscribe(nh, "scan_in", 10);
		laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, "base_link", 10);
		laser_scan_filter_->registerCallback(boost::bind(&ScanAlignment::scanCallback, this, _1));

		pc_scan_sub_.subscribe(nh, "scan_aligned", 10);
		pc_scan_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pc_scan_sub_, *tf_, "odom", 10);
		pc_scan_filter_->registerCallback(boost::bind(&ScanAlignment::oblScanCallback, this, _1));

		scan_aligned_pub_ = nh.advertise<sensor_msgs::PointCloud>("scan_aligned", 10);
		acc_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("acc_scan", 10);
		acc_pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("acc_pcl", 10);
	}

	void scanCallback(sensor_msgs::LaserScanConstPtr scan)
	{
		sensor_msgs::PointCloud laser_cloud;
		try{projector_.transformLaserScanToPointCloud("base_link", *scan, laser_cloud, *tf_);}
		catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());}
		laser_cloud.header.frame_id = "odom_baselink";
		scan_aligned_pub_.publish(laser_cloud);

	}

	void oblScanCallback(sensor_msgs::PointCloudConstPtr scan)
	{
		sensor_msgs::PointCloud accumulated_cloud, pc_received = *scan;
		ad_->addData(pc_received, *tf_);
		vector<sensor_msgs::PointCloud> data;


		if(ad_->getLatestAccumulated(data))
		{
			accumulated_cloud.header = data[0].header;
			for(size_t i=0; i<data.size(); i++)
			{
				accumulated_cloud.points.insert(accumulated_cloud.points.end(), data[i].points.begin(), data[i].points.end());
			}
			sensor_msgs::PointCloud2 accumulated_pc2;
			pcl::PointCloud<pcl::PointXYZ> accumulated_pcl;
			sensor_msgs::convertPointCloudToPointCloud2(accumulated_cloud, accumulated_pc2);
			pcl::fromROSMsg(accumulated_pc2, accumulated_pcl);
			pcl_downsample(accumulated_pcl, 0.1, 0.1, 0.1);
			pcl::toROSMsg(accumulated_pcl, accumulated_pc2);
			sensor_msgs::convertPointCloud2ToPointCloud(accumulated_pc2, accumulated_cloud);
			tf_->transformPointCloud("odom_baselink", accumulated_cloud, accumulated_cloud);
			acc_scan_pub_.publish(accumulated_cloud);
			acc_pcl_pub_.publish(accumulated_pcl);
		}

	}

	template<class T>
	void pcl_downsample(pcl::PointCloud<T> &point_cloud, double size_x, double size_y, double size_z)
	{
		if(point_cloud.size()<100) return;
		pcl::VoxelGrid<T> sor;
		// always good not to use in place filtering as stated in
		// http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
		pcl::PointCloud<T> input_msg_filtered = *(new pcl::PointCloud<T> ());
		//float downsample_size_ = 0.05;
		sor.setInputCloud(point_cloud.makeShared());
		sor.setLeafSize (size_x, size_y, size_z);
		pcl::PointIndicesPtr pi;
		sor.filter (input_msg_filtered);
		/*cout<<"Downsampled colors ";
	        for(size_t i=0; i<input_msg_filtered->points.size(); i++)
	        {
	            cout<<input_msg_filtered->points[i].rgb<<" ";
	        }
	        cout<<endl;*/
		point_cloud = input_msg_filtered;
	}
};

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "scan_align");
	ScanAlignment sa;
	ros::spin();
	return 0;
}
