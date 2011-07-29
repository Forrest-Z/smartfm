#include <people_detect.h>
#include <cmath>

/*
 * version 6.4 
 */

namespace people_detector{
	
	people_detect::people_detect()
	{
		pedestrians_cloud.header.frame_id="map";
		veri_pedestrians_cloud.header.frame_id="map";
		
		sick_pose.point.x=0;
		sick_pose.point.y=0;
		sick_pose.point.z=0; 
		object_total_number=0;
		
		ROS_INFO("Initializing the map...\n");
		ros::NodeHandle nh;
		map_sub_ = nh.subscribe("curb_map", 1, &people_detect::initMap, this);
		laser_pub = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
		
	    laser_sub_.subscribe(nh, "scan", 2);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, "map", 2);
		tf_filter_->registerCallback(boost::bind(&people_detect::scanCallback, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.05));
		
		//"pedestrians_cloud" stores laser-extracted pedestrians in "map" coordinate;
		pedestrians_pub=nh.advertise<sensor_msgs::PointCloud>("pedestrians_cloud",2);
		//"veri_pedestrians_cloud" stores vision-verified pedestrians after probability check;
		veri_pedestrians_pub=nh.advertise<sensor_msgs::PointCloud>("veri_pedestrians_cloud",2);
		
		//"lb_objs" sends laser-extracted pedestrians in "sick_laser" coordinate, 
		//to "camera_project" node to provide detailed pixel information, for vision verification procession;
		lb_objs_pub=nh.advertise<people_detector::labeled_objs>("labeled_objs_cloud",2);
		
		//"temp_vision_veri_obj" stores parts of "veri_objs" recieving from "camera_project", after probability check, in FOV of camera;
		//send this back to vision part, to show on the screen;
		veri_obj_shows=nh.advertise<people_detector::people_rects>("pedestrian_verified",2);
		
		//"temp_laser_extract_obj" stores parts of "veri_objs" recieving from "camera_project" in FOV of camera;
		// just for analysis purpose;	
		extract_obj_shows=nh.advertise<people_detector::people_rects>("pedestrian_extracted",2);
		
		//"pedestrian_mixtured" stores mixtured results, to improve "Precision" and "Recall" Rate;
		mix_obj_shows=nh.advertise<people_detector::people_rects>("pedestrian_mixtured",2);
		
		//Bayes filter
		//"veri_objs" stores data recieving from "camera_project", use its "decision_flag" to update belief of the filter;
		veri_objs_sub=nh.subscribe("verified_objects", 1, &people_detect::Bayesfilter, this);
		person_sense_person=0.8;
		not_person_sense_person=0.3;
		TracVeriNum_pub=nh.advertise<geometry_msgs::Point>("TracVeriNum", 2);	
		
		LaserHistoPub_  = nh.advertise<people_detector::dis_histogram>("LaserDisHisto_",2);
		VisionHistoPub_ = nh.advertise<people_detector::dis_histogram>("VisionDisHisto_",2);
		MixHistoPub_ 	= nh.advertise<people_detector::dis_histogram>("MixDisHisto_",2);		
	}
	
	void people_detect::initMap(const nav_msgs::OccupancyGrid& map)
	{
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
	}
	
	void people_detect::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
		//ROS_INFO("laser callback begin");	
		try
		{
			projector_.transformLaserScanToPointCloud("map", *scan_in, laser_cloud, tf_);
		}
		catch (tf::TransformException& e)
		{
			ROS_INFO("Wrong!!!!!!!!!!!!!");
			std::cout << e.what();
			return;
		}
			    	    
	    people_detect::tfSickPose(sick_to_map);
		people_detect::extractPoints(input_data_, laser_cloud);		
		laser_pub.publish(laser_cloud);             //To view the extracted points.
		//ROS_INFO("extractPoints finished");
		
		people_detect::extractClusters(extracted_points); 
		//laser_pub.publish(laser_cloud);    
		//ROS_INFO("extractClusters finished");
		
		people_detect::processClusters(extracted_clusters);
		//ROS_INFO("processClusters finished");
		
		people_detect::associateHistory(processed_clusters);		
		pedestrians_pub.publish(pedestrians_cloud); 
		//ROS_INFO("Associate history finished");
		
		lb_objs_pub.publish(lb_objs);
		people_detect::ProbabilityCheck();
	}
	
	  void people_detect::extractPoints(const std::vector<unsigned int> &cells, sensor_msgs::PointCloud &cloud_points)
	  {
		extracted_points.clear();
		for(unsigned int i=0; i<cloud_points.points.size(); i++)      //i<(int)cloud_points.points.size();
		{
			int cell_xlabel, cell_ylabel, cell_vectorlabel[7][7];
			bool extract_flag= true;
			cell_xlabel=floor((cloud_points.points[i].x-map_origin_x)/map_resolution);
			cell_ylabel=floor((cloud_points.points[i].y-map_origin_y)/map_resolution);  
			
			if((cell_xlabel>3)&&(cell_ylabel>3))  //very tricky here.
			{
			//to substract point if its distance to background is smaller than 6*0.05m
			   for(int j=0; j<7; j++)
			   {
				   for(int k=0; k<7; k++)
				  {
					cell_vectorlabel[j][k]=(cell_xlabel+(k-3))+map_width_*(cell_ylabel+(j-3));
					int temp_cellnum=cell_vectorlabel[j][k];
					if(cells[temp_cellnum]==100)extract_flag=false;
				  }
			   } 
			}
			 
			//use extract_flag to determine whether it is should be extracted or not. 
			if(extract_flag)
			{
				labeled_point temp_point(cell_xlabel, cell_ylabel, cloud_points.points[i].x, cloud_points.points[i].y, cloud_points.points[i].z);
				extracted_points.push_back(temp_point);        
				// ROS_INFO("I heard: %d", extracted_points[0].label_x);       //just for debugging.   
			}				 
			else
			{ 	
				//set all these irrelavant points to 0. Then we can view those extracted points clearly. 
				cloud_points.points[i].x=0;     
				cloud_points.points[i].y=0;
				cloud_points.points[i].z=0; 		                            
			}	
		}
		ROS_INFO("extracted_points size %d",extracted_points.size());
      }
      
       void people_detect::extractClusters(const std::vector<labeled_point> &extracted_points_para)
      {
		extracted_clusters.clear();
		double dis_to_sick_x, dis_to_sick_y, dis_to_sick, dis_threshold, dis_px, dis_py,dis_pxy;
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
			  /*
			   *int dis_x, dis_y;
				dis_x = extracted_points_para[ix].label_x-extracted_points_para[ix-1].label_x;
				dis_y = extracted_points_para[ix].label_y-extracted_points_para[ix-1].label_y;
			  */
			    if(sick_pose.point.x==0&&sick_pose.point.y==0&&sick_pose.point.z==0){dis_threshold=0.5;}
			    else{
					dis_to_sick_x=extracted_points_para[ix].pointx-sick_pose.point.x;
					dis_to_sick_y=extracted_points_para[ix].pointy-sick_pose.point.y;
					dis_to_sick=sqrt(dis_to_sick_x*dis_to_sick_x+dis_to_sick_y*dis_to_sick_y);
					if(dis_to_sick>=23){dis_threshold=(double)(0.008723*dis_to_sick);}
					else{dis_threshold=0.5;}
					}
				//ROS_INFO("dis_to_sick: %.5f", dis_to_sick);	
				dis_px=extracted_points_para[ix].pointx-extracted_points_para[ix-1].pointx;
				dis_py=extracted_points_para[ix].pointy-extracted_points_para[ix-1].pointy;
				dis_pxy=(double)(sqrt(dis_px*dis_px+dis_py*dis_py));
				
				if(dis_pxy<dis_threshold)
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
	     ROS_INFO("extracted_clusters size %d",extracted_clusters.size());
	   }  
	   
	   //to filter out those too big or too small clusters.
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
	          
		      //ROS_INFO("x_range, y_range (%d, %d)",temp_cluster.x_range, temp_cluster.y_range);
		      if(temp_cluster.x_range>=1&&temp_cluster.x_range<15&&temp_cluster.y_range>=1&&temp_cluster.y_range<15&&temp_cluster.x_range*temp_cluster.y_range<200)
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
		   ROS_INFO("Processsing Finished, clustered remained: %d", remain_cluster_num);
	   }
	   	   
	   void people_detect::associateHistory(std::vector<labeled_cluster> &processed_clusters_para)
	   {
		   pedestrians_cloud.points.clear();
		   pedestrians_cloud.header.stamp=laser_cloud.header.stamp;	// also pay attention here.
		   
		   std::vector<int> stale_numbers;
		   lb_objs.lb_objs_vector.clear();
		   //hitory_pool should be reserved, and should not be cleared during whole program.
		   
		   if(history_pool.size()==0)    //history_pool initialization;
		   {
			  //ROS_INFO("processed_clusters_para size: %d", processed_clusters_para.size());
			  for(std::vector<labeled_cluster>::size_type ic=0; ic!=processed_clusters_para.size(); ic++)
  			  {
			    cluster_history temp_history;  
			    object_total_number++;
				temp_history.object_label=object_total_number;  //from "1" to _para.size();
				temp_history.cluster_data.push_back(processed_clusters_para[ic]);
				history_pool.push_back(temp_history);
			   }
			   ROS_INFO("History Initialization Finished: %d", (int)history_pool.size());
		   }
		   
		   //association with history;
		   else      
		   {
			    //ROS_INFO("---------------------------------------NEW ROUND--------------------------------------"); 
			    int unreg_limit;
			    float searching_range=0.3;
			    
			    //ROS_INFO("A.Loop-------Clusters Associate History: Cluster Pool Size %d", processed_clusters_para.size());
			    for(std::vector<labeled_cluster>::size_type ic=0; ic!=processed_clusters_para.size(); ic++)  //First Loop: clusters try to find matching history;
			    {	
					double dis2d=1000;
					int nearest_cluster=0;  
					for(std::vector<cluster_history>::size_type ih=0; ih!=history_pool.size(); ih++)
					{	//ROS_INFO("---Cluster Number: %d---", ic);
						double dis_x, dis_y, dis2d_temp;
						dis_x= history_pool[ih].cluster_data.back().average_x-processed_clusters_para[ic].average_x;
						dis_y= history_pool[ih].cluster_data.back().average_y-processed_clusters_para[ic].average_y;
						dis2d_temp=sqrt(dis_x*dis_x+dis_y*dis_y);
						if(dis2d_temp<dis2d){dis2d=dis2d_temp; nearest_cluster=ih;}        //clusters try to find the nearest history;
					}
					
			       // ROS_INFO("Cluster Find Its Nearest History---Cluster Number: %d; History Number: %d, History's Obj_label %d, History's Descendant(-1): %d", ic, nearest_cluster, history_pool[nearest_cluster].object_label,history_pool[nearest_cluster].its_possible_descendant);
			        
			        if(history_pool[nearest_cluster].mov_flag==false){unreg_limit=2;searching_range=0.3;}
			        else {unreg_limit=3;searching_range=0.4;}
			        
					if(dis2d<(1+history_pool[nearest_cluster].unreg_times)*searching_range)   // (1)nearest history; (2)within 0.1 meter.
					{
						//ROS_INFO("1.Pair Within Searching Range");
						if(history_pool[nearest_cluster].reg_flag==false)   // avoid multiple registration for one cluster history.
						{
						   history_pool[nearest_cluster].reg_flag=true;
						   history_pool[nearest_cluster].cluster_serial=ic;
						   
						   // put this block in next history loop. Important for the "else" block below.
						   
						   /*
						   history_pool[nearest_cluster].its_possible_descendant=-1;  //break connection;
						   history_pool[nearest_cluster].unreg_times=0;
						   history_pool[nearest_cluster].cluster_data.push_back(processed_clusters_para[ic]);
						   * */ 
						   //ROS_INFO("2.Cluster Update no-descendant History");
						}
						
						else
						{
							if(history_pool[nearest_cluster].mov_flag==true)
							{
								object_total_number++;
								cluster_history temp_history=history_pool[nearest_cluster];
								
								temp_history.object_label=object_total_number;
								temp_history.its_possible_descendant=-1;  
								temp_history.unreg_times=0;
								temp_history.cluster_data.push_back(processed_clusters_para[ic]);
								history_pool.push_back(temp_history);
								ROS_INFO("People Separate!!!");
							}
							else{}  //static objects will not have this phenomenon.
						}
					}
					
					/* Revise "else" block below.
					 * 1)The unassociated cluster will create new history branch "a".The old nearest history branch "b" record the tag of this new history branch.
					 * 2)Then in next round, "a" get updated with new income cluster "c". "b" check whether "c" is also within its possible searching area. If it is, even if once in unreg_limit times, we assume it is the same object. 
					 * 3)When "b" approach its "deadline"--the unreg_limit, it check whether it is reborn by "a". If so, push all elements of "a" into "b", and deleted "b".
					 * Here we need to set reborn_flag, hand_label a and hand_label b. 
					*/					
					
					else     //no matching history; to create one new piece of history.
					{
						//ROS_INFO("1.ELSE!!! NOT WITHIN RANGE, STORE THIS CLUSTER IN TEMP_CLUSTER_POOL");
						
						object_total_number++;
						cluster_history temp_history;  
						temp_history.object_label=object_total_number;
						temp_history.cluster_data.push_back(processed_clusters_para[ic]);
											
						history_pool.push_back(temp_history);
						//ROS_INFO("New History Created %d, Obj_label %d",history_pool.size(),temp_history.object_label);
						
						if(history_pool[nearest_cluster].reg_flag==false)
						{history_pool[nearest_cluster].its_possible_descendant=temp_history.object_label;
						//ROS_INFO("2.Connection Built between Two History");
						}
						
						//Actually, below is a special case that can also happen. 
						else
						{}
						
					}
				}
				//ROS_INFO("B.Loop------------History Self Check, History Pool Size %d--------------", history_pool.size());
				
			  for(std::vector<cluster_history>::size_type ih=0; ih!=history_pool.size(); ih++)  //2nd view: from the view of history.
				{
					std::vector<cluster_history>::size_type descendant_serial =0;
					//ROS_INFO("---History Number %d, History Object_label %d---", ih, history_pool[ih].object_label);
					
					if(history_pool[ih].reg_flag==false)
					{
						history_pool[ih].unreg_times=history_pool[ih].unreg_times+1;
						//ROS_INFO("1.History not updated, unreg_times+1: %d; possible descendant Obj_label: %d", history_pool[ih].unreg_times, history_pool[ih].its_possible_descendant);			
										
						if(history_pool[ih].its_possible_descendant!=-1)
						{	//ROS_INFO("2.Reborn Chance!!!");
						
							for(std::vector<cluster_history>::size_type ihh=0; ihh!=history_pool.size(); ihh++) 
							{if(history_pool[ih].its_possible_descendant==history_pool[ihh].object_label) descendant_serial=ihh;}
						   // ROS_INFO("3.The possible descendant history is %d, history's object label is %d", descendant_serial, history_pool[descendant_serial].object_label);
						    
							double disx, disy,dis2dtemp;
							disx= history_pool[ih].cluster_data.back().average_x-history_pool[descendant_serial].cluster_data.back().average_x;
							disy= history_pool[ih].cluster_data.back().average_y-history_pool[descendant_serial].cluster_data.back().average_y;
							dis2dtemp=sqrt(disx*disx+disy*disy);
							if(dis2dtemp<(0+history_pool[ih].unreg_times)*searching_range){history_pool[ih].reborn_label=true;}
							else{
							//ROS_INFO("4.History not reborned, waiting to be doomed"); 
							history_pool[ih].its_possible_descendant=-1;}
						}
				
						if(history_pool[ih].reborn_label==true)
						{
						//ROS_INFO("4.Yep! History Get Reborned");						
						//update ancestor's information with its descendant's. Not Reversable for one special considerations----descendant's descendant
					    history_pool[ih].object_label=history_pool[descendant_serial].object_label;      
						history_pool[ih].cluster_data.push_back(history_pool[descendant_serial].cluster_data.back());
						history_pool[ih].unreg_times=history_pool[descendant_serial].unreg_times;
						//history_pool[ih].its_possible_descendant=-1;
						history_pool[ih].its_possible_descendant=history_pool[descendant_serial].its_possible_descendant;
						history_pool[ih].reborn_label=false;
						history_pool.erase(history_pool.begin()+descendant_serial);			
						}
					}
					
					else{
						   int temp_cluster_serial= history_pool[ih].cluster_serial;
						   history_pool[ih].its_possible_descendant=-1;  //break connection;
						   history_pool[ih].unreg_times=0;
						   history_pool[ih].cluster_data.push_back(processed_clusters_para[temp_cluster_serial]);
						}	

					if(history_pool[ih].unreg_times==unreg_limit)
					{
					    stale_numbers.push_back(ih); // ROS_INFO("*****stale history_pool: %d, object_label %d*****", ih, history_pool[ih].object_label);
					} 
					
					else
					{
						//ROS_INFO("cluster history: %d, number %d, unregisted times: %d", ih,history_pool[ih].cluster_data.size(), history_pool[ih].unreg_times);
						moving_object moving_object_temp;
						double moving_range_x=history_pool[ih].cluster_data.back().average_x-history_pool[ih].cluster_data.front().average_x;
						double moving_range_y=history_pool[ih].cluster_data.back().average_y-history_pool[ih].cluster_data.front().average_y;
						double moving_range=sqrt(moving_range_x*moving_range_x+moving_range_y*moving_range_y);
						if(moving_range>0.5&&moving_range<8)
						{
							history_pool[ih].mov_flag=true;     //do not clear it back to false next round; even it is only detected once as moving pedestrians.
							moving_object_temp.moving_label=history_pool[ih].object_label;
							moving_object_temp.obj_x=history_pool[ih].cluster_data.back().average_x;
							moving_object_temp.obj_y=history_pool[ih].cluster_data.back().average_y;
							//ROS_INFO("history %d, object_label:%d, distance:%5f, cluster_size:%d, position: %5f, %5f", ih, moving_object_temp.moving_label, moving_range, history_pool[ih].cluster_data.size(),moving_object_temp.obj_x, moving_object_temp.obj_y);
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
					       
					       //ROS_INFO("pedestrians in map frame: %3f, %3f, %3f", ped_point_temp.x,ped_point_temp.y,ped_point_temp.z);
					       
					       //transform ped_point_temp into "sick_laser" frame.
						   people_detect::tfMaptoSick(ped_point_temp, map_to_sick);
						   
						   if(ped_point_temp.y>0)
						   {
							    if(ped_point_temp.x>1.732*ped_point_temp.y) 
								{history_pool[ih].inFOV_flag=true;}
								else {history_pool[ih].inFOV_flag=false;}
						   }
						   else
						   {
							    if(ped_point_temp.x>(-1.732)*ped_point_temp.y) 
							    {history_pool[ih].inFOV_flag=true;}
								else {history_pool[ih].inFOV_flag=false;}
						   }
						
						   geometry_msgs::PointStamped hokuyo_ped;
						   geometry_msgs::PointStamped camerabase_ped;
						   hokuyo_ped.header.frame_id="laser";
						   hokuyo_ped.point.x=ped_point_temp.x;
						   hokuyo_ped.point.y=ped_point_temp.y;
						   hokuyo_ped.point.z=ped_point_temp.z;
						   tf_.transformPoint("camera_base", hokuyo_ped, camerabase_ped);
						   
					       people_detector::labeled_obj temp_lbobj;
						   temp_lbobj.object_label = history_pool[ih].object_label;
						   
						   temp_lbobj.pedestrian_point.x =  camerabase_ped.point.x;
						   temp_lbobj.pedestrian_point.y =  camerabase_ped.point.y;
						   temp_lbobj.pedestrian_point.z =  camerabase_ped.point.z;
						   
						   lb_objs.lb_objs_vector.push_back(temp_lbobj);
						}
					}
					
					if(history_pool[ih].cluster_data.size()>=31)
					{
					   history_pool[ih].cluster_data.erase(history_pool[ih].cluster_data.begin());   //differentiate .front() and .begin() 
					   //ROS_INFO("cluster history filled, delete the oldest one: %d", ih);
					}						
					history_pool[ih].reg_flag= false; 
				} 
				
				if(stale_numbers.size()>0)
				{
					//int stl_num=stale_numbers.size();
					//ROS_INFO("stale numbers total number: %d",stl_num);
					for(int it=stale_numbers.size()-1; it>=0; it--)
					{
					 //ROS_INFO("stale history deleted: %d, object %d",stale_numbers[it], history_pool[stale_numbers[it]].object_label);    //put this before the next scentence. 
					 history_pool.erase(history_pool.begin()+stale_numbers[it]);
					}
				}
				// ROS_INFO("*********************************One Round of History Association Finished, History_pool Size: %d*************************", (int)history_pool.size());	
		    }
		}
		
		//function used to get the position of SICK in "map" frame.
        void people_detect::tfSickPose(const tf::TransformListener& sick_to_map_para)
        {
			geometry_msgs::PointStamped sick_local;
			sick_local.header.frame_id="laser";
			sick_local.point.x=0;
			sick_local.point.y=0;
			sick_local.point.z=0;
			sick_to_map_para.transformPoint("map",sick_local, sick_pose);
			//ROS_INFO("sick_pose: (%.5f, %.5f)", sick_pose.point.x, sick_pose.point.y);			
		}
		
		//function used to get position of pedestrians in "sick" frame.
		void people_detect::tfMaptoSick(geometry_msgs::Point32& ped_point_temp_para,const tf::TransformListener& sick_to_map_para)
		{
			geometry_msgs::PointStamped pose_map_temp;
			geometry_msgs::PointStamped pose_sick_temp;
			
			pose_map_temp.header.frame_id="map";
			pose_map_temp.point.x= ped_point_temp_para.x;
			pose_map_temp.point.y= ped_point_temp_para.y;
			pose_map_temp.point.z= ped_point_temp_para.z;
			
			sick_to_map_para.transformPoint("laser",pose_map_temp, pose_sick_temp);
			
			ped_point_temp_para.x=pose_sick_temp.point.x;
			ped_point_temp_para.y=pose_sick_temp.point.y;
			ped_point_temp_para.z=pose_sick_temp.point.z;
		}
		
		void people_detect::Bayesfilter(const people_detector::people_rects& veri_objs_para)
		{
			veri_objs = veri_objs_para;
			for(std::vector<cluster_history>::size_type ih=0; ih!=history_pool.size(); ih++)
			{
				if(history_pool[ih].mov_flag==true)      //Pay attention: only for those moving objects
				{
					for(unsigned int i=0; i<veri_objs_para.pr_vector.size(); i++)
					{
						if(history_pool[ih].object_label==veri_objs_para.pr_vector[i].object_label)
						{ 
							//if(veri_objs_para.pr_vector[i].complete_flag==true)
							{
								if(veri_objs_para.pr_vector[i].decision_flag==true)
								{float sense_person;
								sense_person=(history_pool[ih].belief_person)*person_sense_person+(1-history_pool[ih].belief_person)*not_person_sense_person;
								history_pool[ih].belief_person=((history_pool[ih].belief_person)*person_sense_person)/sense_person;
								}
								else
								{float sense_not_person;
								sense_not_person=(history_pool[ih].belief_person)*(1-person_sense_person)+(1-history_pool[ih].belief_person)*(1-not_person_sense_person);
								history_pool[ih].belief_person=(history_pool[ih].belief_person)*(1-person_sense_person)/sense_not_person;
								}
							}
						}
					}
				}
			}
		}
		
		void people_detect::ProbabilityCheck()
		{
			//if not clear, path of these pedestrians will be showed.
			veri_pedestrians_cloud.points.clear();  
		    veri_pedestrians_cloud.header.stamp=laser_cloud.header.stamp;
		    ros::Time tempTime = laser_cloud.header.stamp;	    

		    int TrackedNum_temp=0;
		    int VerifiedNum_temp=0;
		    int MixNum_temp=0;
		    
			people_detector::people_rects temp_laser_extract_obj;	    
		    people_detector::people_rects temp_vision_veri_obj;
		    people_detector::people_rects temp_mixture_obj;
		    
			for(std::vector<cluster_history>::size_type ih=0; ih!=history_pool.size(); ih++)
			{
				if(history_pool[ih].mov_flag && history_pool[ih].inFOV_flag)
				{
					TrackedNum_temp++;
					
					for(unsigned int i=0; i<veri_objs.pr_vector.size(); i++)
					{
						people_detector::people_rect temp_veri_obj_show;
						if(history_pool[ih].object_label==veri_objs.pr_vector[i].object_label)
						{
								temp_veri_obj_show=veri_objs.pr_vector[i];
								
								temp_laser_extract_obj.pr_vector.push_back(temp_veri_obj_show);
								people_detect::HistorCreat(temp_veri_obj_show, LaserDisHisto_);
								
								if(history_pool[ih].belief_person>0.6||history_pool[ih].OnceforAll_flag==true)
								{ 	
									VerifiedNum_temp++;
									MixNum_temp++;
									
									geometry_msgs::Point32 veri_ped_point_temp;
									veri_ped_point_temp.x = (float)history_pool[ih].cluster_data.back().average_x;
									veri_ped_point_temp.y = (float)history_pool[ih].cluster_data.back().average_y;
									veri_ped_point_temp.z = (float)history_pool[ih].cluster_data.back().average_z;
									veri_pedestrians_cloud.points.push_back(veri_ped_point_temp);	
									
									temp_vision_veri_obj.pr_vector.push_back(temp_veri_obj_show);
									people_detect::HistorCreat(temp_veri_obj_show, VisionDisHisto_);
									
									temp_mixture_obj.pr_vector.push_back(temp_veri_obj_show);
									people_detect::HistorCreat(temp_veri_obj_show, MixDisHisto_);

									if(history_pool[ih].belief_person>0.6)
									{
										history_pool[ih].OnceforAll_flag=true;
										history_pool[ih].veriTime=tempTime.toSec();
									}
									else
									{
										double veriInterval = tempTime.toSec()-history_pool[ih].veriTime;
										if(veriInterval>0.3){history_pool[ih].OnceforAll_flag=false;}
									}	




								}
								//one point worth thinking:
								//mixture of "laser" and "vision", according to complete_flag;
								else if(veri_objs.pr_vector[i].complete_flag==false)
								{
									MixNum_temp++;
									temp_mixture_obj.pr_vector.push_back(temp_veri_obj_show);
									people_detect::HistorCreat(temp_veri_obj_show, MixDisHisto_);
								}
								
								
						}	
					}
				}
			}
			veri_pedestrians_pub.publish(veri_pedestrians_cloud);
			
			TracVeriNum.x=TrackedNum_temp;
			TracVeriNum.y=VerifiedNum_temp;
			TracVeriNum.z=MixNum_temp;
			TracVeriNum_pub.publish(TracVeriNum);
						
			veri_obj_shows.publish(temp_vision_veri_obj);
			extract_obj_shows.publish(temp_laser_extract_obj);
			mix_obj_shows.publish(temp_mixture_obj);
			
			LaserHistoPub_.publish(LaserDisHisto_);
			VisionHistoPub_.publish(VisionDisHisto_);
			MixHistoPub_.publish(MixDisHisto_);
		}
		
		void people_detect::HistorCreat(const people_detector::people_rect& temp_veri_obj_para, people_detector::dis_histogram& HistorPara)
		{
			//"tempinteger" decides which historgram bin this pedestrian fall into;
			// the historgram is evenly spaced with the interval 2 meters;
			unsigned int tempinteger=0;
			float tempdis = temp_veri_obj_para.disz;
			float tempquo = tempdis/2.0;
			tempinteger = (int)tempquo;
			
			if(tempinteger>=HistorPara.disHisto.size())
			{	
				unsigned int tempsize =tempinteger+1;
				HistorPara.disHisto.resize(tempsize);
				HistorPara.disHisto.back()=1;
			}
			else
			{
				HistorPara.disHisto[tempinteger]=HistorPara.disHisto[tempinteger]+1;
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
