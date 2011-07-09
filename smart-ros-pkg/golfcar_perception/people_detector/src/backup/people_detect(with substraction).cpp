#include <people_detect.h>
#include <cmath>
namespace people_detector{
	
	people_detect::people_detect()
	{
		ROS_INFO("Initializing the map...\n");
		ros::NodeHandle nh;
		map_sub_ = nh.subscribe("map", 1, &people_detect::initMap, this);  // Here, service is better than topic? Change it later?
		laser_pub = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
		laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&people_detect::scanCallback, this, _1));
	}
	
	void people_detect::initMap(const nav_msgs::OccupancyGrid& map)
	{
		boost::recursive_mutex::scoped_lock lock(map_data_lock_);
		map_width_ = map.info.width;                                       //pay attention here.
		map_height_ = map.info.height;
		map_resolution=map.info.resolution;
		map_origin_x=map.info.origin.position.x;
		map_origin_y=map.info.origin.position.y;
	    map_origin_z=map.info.origin.position.z;
		
		unsigned int numCells =  map_width_ * map_height_;
	    unsigned int a;
	    
	    input_data_.clear();
	    
		for(unsigned int i = 0; i < numCells; i++)
		{
			if(map.data[i]==-1 || map.data[i]==100) a=100;                   //change -1 to 0, treat unknown points as clear.
			else a=0;
			input_data_.push_back(a);                                   //Will the input_data_ be too large?
		}
		ROS_INFO("Map of %d cells initialized", numCells);
		
		//people_detect::drawMap(input_data_);                          //display map only when debugging.    
	}
	
		/*	
		 * void people_detect::drawMap(std::vector<unsigned int> &cells)
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
		*/	
		
	void people_detect::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{		
		try
		{
			projector_.transformLaserScanToPointCloud("map", *scan_in, laser_cloud, listener);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			return;
		}
		//laser_pub.publish(laser_cloud);             //no need to publish. Just for debugging.
			
		people_detect::extractPoints(input_data_, laser_cloud);		
		laser_pub.publish(laser_cloud);             //To view the extracted points.
		

		
	}
	
	  void people_detect::extractPoints(const std::vector<unsigned int> &cells, sensor_msgs::PointCloud &cloud_points)
	  {
		extracted_points.clear();
		for(int i=0; i<=cloud_points.points.size(); i++)
			{
				int cell_xlabel, cell_ylabel, cell_vectorlabel[5][5];
				bool extract_flag= true;
				cell_xlabel=floor((cloud_points.points[i].x-map_origin_x)/map_resolution);
				cell_ylabel=floor((cloud_points.points[i].y-map_origin_y)/map_resolution);  
				
				if((cloud_points.points[i].x>1)&&(cloud_points.points[i].y>1))
				{
				 
			        for(int j=0; j<5; j++)
				   {
					   for(int k=0; k<5; k++)
					  {
						cell_vectorlabel[j][k]=(cell_xlabel+(k-2))+map_width_*(cell_ylabel+(j-2));
						if(cells[cell_vectorlabel[j][k]]==100)extract_flag=false;
					  }
				   } 
				} 
				
				if(extract_flag)
				{
					labeled_point temp_point(cell_xlabel, cell_ylabel, cloud_points.points[i].x, cloud_points.points[i].y, cloud_points.points[i].z);
					extracted_points.push_back(temp_point);        //Will vector here be too large? 
					// ROS_INFO("I heard: %d", extracted_points[0].label_x);       //just for debugging.   
				 }				 
				 else
				 { 	
					cloud_points.points[i].x=0;     
					cloud_points.points[i].y=0;
					cloud_points.points[i].z=0; 		                                    //ignore "/" will also cause segmentation fault.
				 }	
			 }
      }
      
        
}
    
    

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "peopledetect_node");
	 
	 people_detector::people_detect* people_detect_node;
	 people_detect_node = new people_detector::people_detect();
	 
	 ros::spin();
}
