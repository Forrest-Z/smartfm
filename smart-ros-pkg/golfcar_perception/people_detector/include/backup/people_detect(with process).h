#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

//laser geometery
#include <laser_geometry/laser_geometry.h>

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
	   tf::TransformListener listener;
           ros::Publisher laser_pub;
	   ros::Subscriber laser_sub;
	   
	   //extract points
           std::vector<labeled_point> extracted_points; 
           void extractPoints(const std::vector<unsigned int> &cells, sensor_msgs::PointCloud &cloud_points);

           //extract cluster
	   std::vector<labeled_cluster> extracted_clusters;
       	   void extractClusters(const std::vector<labeled_point> &extracted_points_para);

	   //process cluster
	   std::vector<labeled_cluster> processed_clusters;
           void processClusters(std::vector<labeled_cluster> &extracted_clusters_para);
	        
   };
};
