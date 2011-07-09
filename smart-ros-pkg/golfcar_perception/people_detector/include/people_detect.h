#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

//laser geometery
#include <laser_geometry/laser_geometry.h>

//add message_filter
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "people_detector/labeled_obj.h"
#include "people_detector/labeled_objs.h"

#include "people_detector/people_rect.h"
#include "people_detector/people_rects.h"

//version 6 with Bayesfilter and display show; newly add number calculation.

class labeled_point
      { 
	public:
	int label_x;
	int label_y;
        float pointx;
        float pointy;
        float pointz;

	labeled_point(int a, int b, float c,float d, float e):label_x(a),label_y(b),pointx(c),pointy(d),pointz(e){}
      };

class labeled_cluster
{
	public:	
	int label_num;
	int points_sum;
	std::vector<int> points_serial;
	int x_range;
	int y_range;	
	double cluster_property;                      //The two following variables will be calculated in Classification Part.
	double average_x;
	double average_y;
	double average_z;

	labeled_cluster()
	{label_num=0;
	 points_sum=0;}

	~labeled_cluster(){}
};

class  cluster_history
{	
	public:
	int object_label;
	bool reg_flag;
	bool mov_flag;
	int unreg_times;
	std::vector<labeled_cluster> cluster_data;
	
	//newly added in version3
	bool reborn_label;                       //the last chance to survive, for this history;
	int its_possible_descendant;             //when this history dies away, it tries to find its possible descendant;
	int its_possible_ancestor;               //this history may serves as one new history(descendant), it will has its own ancestor;

	//newly added in version4	
	int cluster_serial;
	
	//newly	added in version5
	float belief_person;

	//newly added in version6
	bool inFOV_flag;
	bool OnceforAll_flag;

	cluster_history()
	{
	 object_label=0;
	 reg_flag=false;
	 mov_flag=false;
	 inFOV_flag=false;
	 OnceforAll_flag=false;
	 unreg_times=0;

	//newly added in version3
	 reborn_label=false;
	 its_possible_descendant=-1;
	 its_possible_ancestor=-1;
	
	//newly added in version4
  	 cluster_serial=-1;

	//newly added in version5, for bayes filter.
	belief_person=0.5;
	}

	~cluster_history(){}	
};

class moving_object
{
	public:
	int moving_label;
	double obj_x;
	double obj_y;
	double obj_z;
};


namespace people_detector{
	/**
   * @class people_detect
   * @brief Detect people by comparing with a base map to eliminate known static object
   * thereby select the possible candidates from this segmented data
   */

   
   class people_detect{
	   public:
	   
	   people_detect();
	   
	   ~people_detect();
	   
	   private:
	   
	   //bool map_flag;
           //map
	   void initMap(const nav_msgs::OccupancyGrid& map);
	   void drawMap(std::vector<unsigned int> &cells);
	   ros::Subscriber map_sub_;
	   boost::recursive_mutex map_data_lock_;
	   std::vector<unsigned int> input_data_;     // vector to store value of map_cell
	   unsigned int map_width_, map_height_;
	   float map_resolution, map_origin_x, map_origin_y, map_origin_z;        //float64 according to MapMetaData.msg. Will it be a problem?
	   int width_gridnum;

           //laser geometry
	   laser_geometry::LaserProjection projector_;
	   sensor_msgs::PointCloud laser_cloud;
	   //sensor_msgs::PointCloud laser_cloud_transformed;		
	   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
    //1	   tf::TransformListener listener;  //already have tf_
    	   ros::Publisher laser_pub;        
    //2	   ros::Subscriber laser_sub;       //have point_sub
	 
	   //extract points
           std::vector<labeled_point> extracted_points; 
           void extractPoints(const std::vector<unsigned int> &cells, sensor_msgs::PointCloud &cloud_points);

           //extract cluster
	   std::vector<labeled_cluster> extracted_clusters;
       	   void extractClusters(const std::vector<labeled_point> &extracted_points_para);

	   //process cluster
	   std::vector<labeled_cluster> processed_clusters;
           void processClusters(std::vector<labeled_cluster> &extracted_clusters_para);

 	   //associate with history; label clusters; select dynamic objects.
	   std::vector<cluster_history> history_pool;
	   std::vector<moving_object> multi_moving_objects;
	   int object_total_number;
	   void associateHistory(std::vector<labeled_cluster> &processed_clusters_para);	
	   
	   //publish PointCloud of pedestrians, with each point representing one person.
           sensor_msgs::PointCloud pedestrians_cloud;
	   ros::Publisher pedestrians_pub;
	   
           //added in version2 to deal with old tf.
 	   message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
           tf::TransformListener tf_; 
           tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;

	   //added in version3
	   geometry_msgs::PointStamped sick_pose;
	   tf::TransformListener sick_to_map; 
           void tfSickPose(const tf::TransformListener& sick_to_map_para);

	   //added in version5, the parameters need to be revised.
	   tf::TransformListener map_to_sick;    //let pedestrians cloud be transformed into "sick" coordinate.
           void tfMaptoSick(geometry_msgs::Point32& ped_point_temp_para, const tf::TransformListener& sick_to_map_para);	   

	   people_detector::labeled_objs lb_objs;
	   ros::Publisher lb_objs_pub;

	   //people_detector::verified_objs veri_objs;
	   ros::Subscriber veri_objs_sub;
	   people_detector::people_rects veri_objs;
	   void Bayesfilter(const people_detector::people_rects& veri_objs_para);  

		//change data type here, from "verified_objs" to "people_rects".
	   float person_sense_person;
	   float not_person_sense_person;
	   
	   void ProbabilityCheck();
           sensor_msgs::PointCloud veri_pedestrians_cloud;
	   ros::Publisher veri_pedestrians_pub;

	   ros::Publisher veri_obj_shows;
		ros::Publisher extract_obj_shows;
           
	   geometry_msgs::Point TracVeriNum;   // TracVeriNum.x is trackedNum; TracVeriNum.y is VeriNum;
	   ros::Publisher TracVeriNum_pub;	

		void HistorCreat(const people_detector::people_rect& temp_veri_obj_para, std::vector<unsigned int>& HistorPara);
		std::vector<unsigned int> LaserDisHistor_;
		std::vector<unsigned int> VisionDisHistor_;
   };
};
