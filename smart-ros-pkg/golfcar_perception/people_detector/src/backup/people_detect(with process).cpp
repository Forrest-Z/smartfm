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
		
		people_detect::extractClusters(extracted_points); 
		//laser_pub.publish(laser_cloud);    
		
		people_detect::processClusters(extracted_clusters);
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
      
       void people_detect::extractClusters(const std::vector<labeled_point> &extracted_points_para)
      {
		extracted_clusters.clear();
		labeled_cluster temp_cluster;                                      //attention: this line should be outside of "for" loop
		for(std::vector<labeled_point>::size_type ix=0; ix!=extracted_points_para.size(); ix++)
		{
			if(ix==0)
			{
				temp_cluster.label_num=1;
				temp_cluster.points_sum=1;
				temp_cluster.points_serial.push_back(ix);
			}

			else 
			{				
				int dis_x, dis_y;
				dis_x = extracted_points_para[ix].label_x-extracted_points_para[ix-1].label_x;
				dis_y = extracted_points_para[ix].label_y-extracted_points_para[ix-1].label_y;
				if(dis_x<4&&dis_x>-4&&dis_y<4&&dis_y>-4)
				{
					temp_cluster.points_sum++;
					temp_cluster.points_serial.push_back(ix);
					if(ix==(extracted_points_para.size()-1))
					{
						extracted_clusters.push_back(temp_cluster);
						//ROS_INFO("cluster_con: %d, point_number: %d", temp_cluster.label_num, temp_cluster.points_sum);
						//ROS_INFO("finished! total cluster: %d",temp_cluster.label_num);
					}
				}
				else 
				{
					extracted_clusters.push_back(temp_cluster);
					//ROS_INFO("cluster_uncon: %d, point_number: %d", temp_cluster.label_num, temp_cluster.points_sum);
					
					temp_cluster.label_num++;
					temp_cluster.points_sum=1;
					temp_cluster.points_serial.clear();
					temp_cluster.points_serial.push_back(ix);	      //Here, ix also serves as an index to extracted_points_para
					if(ix==(extracted_points_para.size()-1))
					{
						extracted_clusters.push_back(temp_cluster);	
						//ROS_INFO("cluster_uncon: %d, point_number: %d", temp_cluster.label_num, temp_cluster.points_sum);
						//ROS_INFO("finished! total cluster: %d",temp_cluster.label_num);
					}			
				}
			}
			
	     }  
	   }  //corresponding to void function()
	   
	   void people_detect::processClusters(std::vector<labeled_cluster> &extracted_clusters_para)
	   {
		   processed_clusters.clear();
		   labeled_cluster temp_cluster;              //store processed clusters temporarily.
		   std::vector <labeled_point> extracted_points_para(extracted_points);
		   int remain_cluster_num=0;
		   
		   for(std::vector<labeled_cluster>::size_type ic=0; ic!=extracted_clusters_para.size(); ic++)
		   {
			  temp_cluster=extracted_clusters_para[ic];
			  std::vector<int> points_serial_para(extracted_clusters_para[ic].points_serial); //just to simplify expression
			 
			   //first,sort all labeled_points, according to x and y respectively.
			  for(std::vector<int>::size_type iz=0; iz<points_serial_para.size()-1; iz++)   //swap n-1 times;
			  {
				 for(std::vector<int>::size_type ip=0; ip<points_serial_para.size()-1-iz; ip++) 
				 {
					 int p_number=points_serial_para[ip]; // p_number 
					 int t;
					 if(extracted_points_para[p_number].label_x>extracted_points_para[p_number+1].label_x)
					 {
						 t=extracted_points_para[p_number].label_x; 
						 extracted_points_para[p_number].label_x=extracted_points_para[p_number+1].label_x;
						 extracted_points_para[p_number+1].label_x=t;
				     }
			     }			     		  
		      }
		      int minimum_x= extracted_points_para[points_serial_para.front()].label_x;
		      int maximum_x= extracted_points_para[points_serial_para.back()].label_x;
		      
	          temp_cluster.x_range= maximum_x-minimum_x+1;
	           
		      //x_range finished; same for y_range;
		      
		      
		       for(std::vector<int>::size_type iz=0; iz<points_serial_para.size()-1; iz++)   //swap n-1 times;
			  {
				 for(std::vector<int>::size_type ip=0; ip<points_serial_para.size()-1-iz; ip++) 
				 {
					 int p_number=points_serial_para[ip]; // p_number 
					 int t;
					 if(extracted_points_para[p_number].label_y>extracted_points_para[p_number+1].label_y)
					 {
						 t=extracted_points_para[p_number].label_y; 
						 extracted_points_para[p_number].label_y=extracted_points_para[p_number+1].label_y;
						 extracted_points_para[p_number+1].label_y=t;
				     }
			     }			     		  
		      }
		      int minimum_y= extracted_points_para[points_serial_para.front()].label_y;
		      int maximum_y= extracted_points_para[points_serial_para.back()].label_y;
	          temp_cluster.y_range= maximum_y-minimum_y+1;
		      
		      if(temp_cluster.x_range>=2&&temp_cluster.x_range<17&&temp_cluster.y_range>=2&&temp_cluster.y_range<17&&temp_cluster.x_range*temp_cluster.y_range<60)
		      {
				float temp_sum_x=0;
				float temp_sum_y=0;
				float temp_sum_z=0;
				for(std::vector<int>::size_type ia=0; ia<points_serial_para.size(); ia++)
				{
				   int a_number=points_serial_para[ia];
				   temp_sum_x=temp_sum_x+extracted_points_para[a_number].pointx;
				   temp_sum_y=temp_sum_y+extracted_points_para[a_number].pointy;
				   temp_sum_z=temp_sum_z+extracted_points_para[a_number].pointz;
			    }
				temp_cluster.average_x=temp_sum_x/points_serial_para.size();
				temp_cluster.average_y=temp_sum_y/points_serial_para.size();
				temp_cluster.average_z=temp_sum_z/points_serial_para.size();
				
				ROS_INFO("cluster_number: %d, x_range: %d, y_range: %d, average position:(%5f, %5f)", temp_cluster.label_num, temp_cluster.x_range, temp_cluster.y_range, temp_cluster.average_x, temp_cluster.average_y);
				processed_clusters.push_back(temp_cluster);
				remain_cluster_num++;
		      }
		   }
		   ROS_INFO("Processsing Finished, clustered remained: %d", remain_cluster_num);
	   }
	   
	   
        
}
    
    

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "peopledetect_node");
	 
	 people_detector::people_detect* people_detect_node;
	 people_detect_node = new people_detector::people_detect();
	 
	 ros::spin();
}
