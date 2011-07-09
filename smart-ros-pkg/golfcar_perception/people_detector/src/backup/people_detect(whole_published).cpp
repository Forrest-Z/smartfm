#include <people_detect.h>
#include <cmath>
namespace people_detector{
	
	people_detect::people_detect()
	{
		pedestrians_cloud.header.frame_id="map";
		
		object_total_number=0;
		ROS_INFO("Initializing the map...\n");
		ros::NodeHandle nh;
		map_sub_ = nh.subscribe("map", 1, &people_detect::initMap, this);  // Here, service is better than topic? Change it later?
		laser_pub = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
		laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&people_detect::scanCallback, this, _1));
		pedestrians_pub=nh.advertise<sensor_msgs::PointCloud>("pedestrians_cloud",2);
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
		
		people_detect::associateHistory(processed_clusters);
		
		pedestrians_pub.publish(pedestrians_cloud);   //remember to clear pedestrians_cloud every round.
	}
	
	  void people_detect::extractPoints(const std::vector<unsigned int> &cells, sensor_msgs::PointCloud &cloud_points)
	  {
		extracted_points.clear();
		for(int i=0; i<=cloud_points.points.size(); i++)
			{
				int cell_xlabel, cell_ylabel, cell_vectorlabel[13][13];
				bool extract_flag= true;
				cell_xlabel=floor((cloud_points.points[i].x-map_origin_x)/map_resolution);
				cell_ylabel=floor((cloud_points.points[i].y-map_origin_y)/map_resolution);  
				
				if((cloud_points.points[i].x>1)&&(cloud_points.points[i].y>1))
				{
				 
			        for(int j=0; j<13; j++)
				   {
					   for(int k=0; k<13; k++)
					  {
						cell_vectorlabel[j][k]=(cell_xlabel+(k-6))+map_width_*(cell_ylabel+(j-6));
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
	   }  
	   
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
				
				//ROS_INFO("cluster_number: %d, x_range: %d, y_range: %d, average position:(%5f, %5f)", temp_cluster.label_num, temp_cluster.x_range, temp_cluster.y_range, temp_cluster.average_x, temp_cluster.average_y);
				processed_clusters.push_back(temp_cluster);
				remain_cluster_num++;
		      }
		   }
		  // ROS_INFO("Processsing Finished, clustered remained: %d", remain_cluster_num);
	   }
	   	   
	   void people_detect::associateHistory(std::vector<labeled_cluster> &processed_clusters_para)
	   {
		   pedestrians_cloud.points.clear();
		   pedestrians_cloud.header.stamp=ros::Time::now();	
		   
		   std::vector<int> stale_numbers;
		   multi_moving_objects.clear();
		   //hitory_pool should be reserved, and should not be cleared during whole program.
		   
		   if(history_pool.size()==0)    //history_pool initialization;
		   {
			    for(std::vector<labeled_cluster>::size_type ic=0; ic!=processed_clusters_para.size(); ic++)
  			  {
			    cluster_history temp_history;  
			    object_total_number++;
				temp_history.object_label=object_total_number;  //from "1" to _para.size();
				temp_history.cluster_data.push_back(processed_clusters_para[ic]);
				history_pool.push_back(temp_history);
				
			   }
			   ROS_INFO("History Initialization Finished: %d", history_pool.size());
		   }
		   
		   else                          //association with history;
		   {
			    for(std::vector<labeled_cluster>::size_type ic=0; ic!=processed_clusters_para.size(); ic++)  //one way: clusters try to find matching history;
			    {
					double dis2d=1;
					int nearest_cluster=0;  //serves as a label for cluster history.
					for(std::vector<cluster_history>::size_type ih=0; ih!=history_pool.size(); ih++)
					{
						double dis_x, dis_y, dis2d_temp;
						dis_x= history_pool[ih].cluster_data.back().average_x-processed_clusters_para[ic].average_x;
						dis_y= history_pool[ih].cluster_data.back().average_y-processed_clusters_para[ic].average_y;
						dis2d_temp=sqrt(dis_x*dis_x+dis_y*dis_y);
						if(dis2d_temp<dis2d){dis2d=dis2d_temp; nearest_cluster=ih;}        //clusters try to find the nearest history;
						
					}
			        // ROS_INFO("distance: %5f, cluster: %d, history: %d",dis2d,ic,nearest_cluster);
			        //debug: 1st break here;

					if(dis2d<(1+history_pool[nearest_cluster].unreg_times)*0.1)   // (1)nearest history; (2)within 0.1 meter.
					{
						if(history_pool[nearest_cluster].reg_flag==false)   // avoid multiple registration for one cluster history.
						{
						   history_pool[nearest_cluster].reg_flag=true;
						   history_pool[nearest_cluster].unreg_times=0;
						   history_pool[nearest_cluster].cluster_data.push_back(processed_clusters_para[ic]);
						 //  ROS_INFO("cluster update history: vector_number %d, object_label %d, matching_distance %5f, cluster %d", nearest_cluster, history_pool[nearest_cluster].object_label, dis2d,ic);
						}
					}
					else                 //no matching history; to create one new piece of history.
					{
						object_total_number++;
						cluster_history temp_history;  
						temp_history.object_label=object_total_number;
						temp_history.cluster_data.push_back(processed_clusters_para[ic]);
						history_pool.push_back(temp_history);
						//ROS_INFO("new history created %d, object_label %d, by cluster %d", history_pool.size(),temp_history.object_label,ic);				
					}		
				}
				//ROS_INFO("****************clusters associated, history_pool_size: %d***************", history_pool.size());
				//debug: 2nd break here;
				
			  for(std::vector<cluster_history>::size_type ih=0; ih!=history_pool.size(); ih++)  //two way: from the view of history.
				{
					if(history_pool[ih].reg_flag==false)
					{
						history_pool[ih].unreg_times=history_pool[ih].unreg_times+1;
						//ROS_INFO("history not updated %d, object_label %d, un_reg_times %d", ih, history_pool[ih].object_label, history_pool[ih].unreg_times);
						
					}
					history_pool[ih].reg_flag= false;       
										
					if(history_pool[ih].unreg_times==4)
					{
					    stale_numbers.push_back(ih);
					   // ROS_INFO("*****stale history_pool: %d, object_label %d*****", ih, history_pool[ih].object_label);
					} 
					
					else
					{
						//ROS_INFO("cluster history: %d, number %d, unregisted times: %d", ih,history_pool[ih].cluster_data.size(), history_pool[ih].unreg_times);
						moving_object moving_object_temp;
						double moving_range_x=history_pool[ih].cluster_data.back().average_x-history_pool[ih].cluster_data.front().average_x;
						double moving_range_y=history_pool[ih].cluster_data.back().average_y-history_pool[ih].cluster_data.front().average_y;
						double moving_range=sqrt(moving_range_x*moving_range_x+moving_range_y*moving_range_y);
						if(moving_range>0.6&&moving_range<4)
						{
							history_pool[ih].mov_flag=true;     //do not clear it back to false next round; even it is only detected once as moving pedestrians.
							moving_object_temp.moving_label=history_pool[ih].object_label;
							moving_object_temp.obj_x=history_pool[ih].cluster_data.back().average_x;
							moving_object_temp.obj_y=history_pool[ih].cluster_data.back().average_y;
							ROS_INFO("history %d, object_label:%d, distance:%5f, cluster_size:%d, position: %5f, %5f", ih, moving_object_temp.moving_label, moving_range, history_pool[ih].cluster_data.size(),moving_object_temp.obj_x, moving_object_temp.obj_y);
							//ROS_INFO("history_pool %d, cluster:%d, position: %5f, %5f", history_pool.size(), history_pool[ih].cluster_data.size(),moving_object_temp.obj_x, moving_object_temp.obj_y);						
							multi_moving_objects.push_back(moving_object_temp);
						}
						//else{ROS_INFO("moving_object position:%5f",moving_range);}
						
						if(history_pool[ih].mov_flag)
						{
					       geometry_msgs::Point32 ped_point_temp;
					       ped_point_temp.x = (float)history_pool[ih].cluster_data.back().average_x;
					       ped_point_temp.y = (float)history_pool[ih].cluster_data.back().average_y;
					       ped_point_temp.z = (float)history_pool[ih].cluster_data.back().average_z;
					       pedestrians_cloud.points.push_back(ped_point_temp);
						}
						
					}
					
					if(history_pool[ih].cluster_data.size()>=31)
					{
					   history_pool[ih].cluster_data.erase(history_pool[ih].cluster_data.begin());   //differentiate .front() and .begin() 
					   //ROS_INFO("cluster history filled, delete the oldest one: %d", ih);
					}						
					
				} 
				
				if(stale_numbers.size()>0)
				{
					int stl_num=stale_numbers.size();
					//ROS_INFO("stale numbers total number: %d",stl_num);
					for(int it=stale_numbers.size()-1; it>=0; it--)
					{
					 //ROS_INFO("stale history deleted: %d, object %d",stale_numbers[it], history_pool[stale_numbers[it]].object_label);    //put this before the next scentence. 
					 history_pool.erase(history_pool.begin()+stale_numbers[it]);
					}
				}
				
				// ROS_INFO("********************************One Round of History Association Finished, History_pool Size: %d*****************************", history_pool.size());	
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
