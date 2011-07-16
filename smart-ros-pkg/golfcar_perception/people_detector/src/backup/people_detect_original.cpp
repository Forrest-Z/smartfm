#include <people_detect.h>

namespace people_detector{
	
	people_detect::people_detect()
	{
		//ROS_INFO("Initializing the map...\n");
		ros::NodeHandle nh;
		//map_sub_ = nh.subscribe("map", 1, &people_detect::initMap, this);
		laser_pub = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
		laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&people_detect::scanCallback, this, _1));
	}
	
	void people_detect::initMap(const nav_msgs::OccupancyGrid& map)
	{
		boost::recursive_mutex::scoped_lock lock(map_data_lock_);
		map_width_ = map.info.width;
		map_height_ = map.info.height;
		unsigned int numCells =  map_width_ * map_height_;
		for(unsigned int i = 0; i < numCells; i++)
		{
			input_data_.push_back((unsigned int) map.data[i]);
		}
		ROS_INFO("Map of %d cells initialized", numCells);
		
		people_detect::drawMap(input_data_);
	}
	
	void people_detect::drawMap(std::vector<unsigned int> &cells)
	{
		for(int i=map_height_-1;i>=0;i--)
		{
			for(unsigned int j=0; j< map_width_; j++)
			{
				unsigned int cell_num = map_width_*i + j;
				if(cells[cell_num] == 100) std::cout << "=";
				else if(cells[cell_num]==0)std::cout << "0";
				else std::cout<<"-";
				
			}
			std::cout << "\n";
		}
	}
	
	void people_detect::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		
		laser_cloud_transformed.header.frame_id = "base_link";
		try
		{
			projector_.transformLaserScanToPointCloud("base_link", *scan_in, laser_cloud, listener);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			return;
		}
		laser_pub.publish(laser_cloud);
	
	}
			
		
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "peopledetect_node");
	 
	 people_detector::people_detect* people_detect_node;
	 people_detect_node = new people_detector::people_detect();
	 
	 ros::spin();
	 
}
	 
