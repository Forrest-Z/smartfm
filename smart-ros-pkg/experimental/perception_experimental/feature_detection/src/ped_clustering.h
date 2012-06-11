/*
 * ped_clustering.h
 *
 *  Created on: Jun 1, 2012
 *      Author: demian
 */

#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/octree/octree_search.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Int64.h>

#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <feature_detection/clusters.h>
#include <feature_detection/cluster.h>

#include <octomap_ros/OctomapROS.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/OctomapBinary.h>
#include <octomap/octomap.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "pnpoly.h"
using namespace std;


class ped_clustering
{
public:
    ped_clustering();
private:
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void clustering(const sensor_msgs::PointCloud2& pc2, sensor_msgs::PointCloud &ped_poi,
                    double tolerance, int minSize, int maxSize, bool publish);
    void filterLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out);
    void extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices,
                        double clusterTolerance, int minSize,int maxSize);
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan_in);
    void filterPriorMap(octomap::OcTree& priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold);
    void octomapTreeToPCLoctree(octomap_msgs::OctomapBinary& octomap, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* pcl_octree);
    void filterPCLOctreeNN(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    ros::Publisher cloud_pub_, filter_pub_, after_line_filter_pub_, after_prior_filter_pub_;
    ros::Publisher poi_pub_, line_filtered_pub_;
    ros::Publisher clusters_pub_, boundary_pub_;

    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pc2_filter_;
    double prior_distance_filter_;
    laser_geometry::LaserProjection projector_;
    string laser_frame_id_, global_frame_;
    bool bounding_boxfilter_, line_filter_;
    bool sequential_clustering_, use_octomap_;
    octomap::OcTree* global_octMap;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* global_pclOctree_;
    void getOctomap();
    ros::NodeHandle nh;

    vector<Point32> boundary_;
    sensor_msgs::PointCloud laser_global_pc_;
};


#ifdef TINYXML_API_PRE26
#define TINYXML_ELEMENT ELEMENT
#define TINYXML_TEXT TEXT
#endif

#include <stdexcept>

#include <tinyxml.h>
#define VERBOSE 1
class svg_boundary
{
public:
	TiXmlDocument svgDoc_;
	geometry_msgs::Point32 size_;
	double res_;

	void loadFile(const char* pFilename, double res)
	{
	    TiXmlElement* svgElement;
	    bool loadOkay = svgDoc_.LoadFile(pFilename);
	    if (loadOkay)
	    {
	        svgElement = svgDoc_.FirstChildElement();
	        const char * s = svgElement->Value();
	        if( strcmp(s,"svg")==0 )
	        {
	            if(VERBOSE) cout <<"Svg file " <<pFilename <<" loaded!" <<endl;
	            size_ = getSize();
	            res_ = res;
	        }
	        else
	            throw runtime_error(string("Is SVG file loaded? The value found is ")+s);
	    }
	    else
	    {
	        throw runtime_error(string("Failed to load file ")+pFilename);
	    }
	}

	void convert_to_meter(vector<geometry_msgs::Point32>& pose)
	{
	    for(unsigned i=0; i<pose.size(); i++)
	    {
	        (pose)[i].x = (pose)[i].x*res_;
	        (pose)[i].y = (size_.y - (pose)[i].y)*res_;
	    }
	}
	geometry_msgs::Point32 getSize()//(TiXmlDocument* svgDoc)//PathPoint* size)
	{
	    TiXmlElement* svgElement;
	    svgElement = svgDoc_.FirstChildElement();

	    double width,height;
	    if( svgElement->QueryDoubleAttribute("width", &width)==TIXML_NO_ATTRIBUTE
	            || svgElement->QueryDoubleAttribute("height", &height)==TIXML_NO_ATTRIBUTE )
	    {
	        throw runtime_error("Height or width information not found");
	    }
	    else
	    {
	        if(VERBOSE)
	        {
	            cout <<"Width found " <<width <<endl;
	            cout <<"Height found " <<height <<endl;
	        }
	    }
	    geometry_msgs::Point32 pp;
	    pp.x = width;
	    pp.y = height;
	    return pp;
	}
	svg_boundary(const char* pFilename, double res)
	{
	    loadFile(pFilename, res);
	}

	vector<geometry_msgs::Point32> getPath(string id)
	{
	    TiXmlElement* svgElement;
	    svgElement = svgDoc_.FirstChildElement();
	    TiXmlNode* pChild;
	    for ( pChild = svgElement->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
	    {
	        TiXmlElement* childElement= pChild->ToElement();
	        cout<<childElement->Value()<<endl;
	        if( strcmp(childElement->Value(),"path")==0 )
	        {
	            const char * value = childElement->Attribute("id");
	            assert( value!=NULL );
	            if( strcmp(id.c_str(),value)==0 )
	            {
	                value = childElement->Attribute("d");
	                assert( value!=NULL );
	                vector<geometry_msgs::Point32> path = StringToPath(value);
	                convert_to_meter(path);
	                return path;
	            }
	        }
	    }
	    throw runtime_error("Path id not found");
	}

	vector<string> SplitString(const char *data, const char* delimiter)
	{
		char *str = strdup(data);
		char *pch = strtok(str, delimiter);
		vector<string> data_s;
		while (pch != NULL)
		{
			data_s.push_back(string(pch));
			pch = strtok(NULL, delimiter);
		}
		free(str);
		return data_s;
	}

	vector<geometry_msgs::Point32> StringToPath(string data)
	{
	    unsigned int found_points=0;
	    //split the data assuming the delimiter is either space or comma
	    if(VERBOSE) cout <<"Data received: " <<data <<endl;
	    vector<string> data_s = SplitString(data.c_str(), " ,");
	    bool abs_rel;
	    //a path must start with m or M
	    if(data_s[0].find_first_of("Mm")==string::npos)
	    {
	        throw runtime_error(string("Unexpected data start character, expected M or m but received ")+data_s[0]);
	    }
	    else
	    {
	        //need to differentiate if it is abs or rel
	        if(data_s[0].find_first_of("M")!=string::npos) abs_rel = true;
	        else abs_rel = false;
	    }

	    vector<geometry_msgs::Point32> positions;
	    geometry_msgs::Point32 pos;
	    pos.x = atof(data_s[1].c_str());
	    pos.y = atof(data_s[2].c_str());
	    found_points++;
	    positions.push_back(pos);
	    for(unsigned int i=3; i<data_s.size(); i++)
	    {
	        if(data_s[i].find_first_of("HhVvSsQqTtAa")!=string::npos)
	        {
	            throw runtime_error(string("Only line path is supported, given ")+data_s[i]);
	        }
	        else if(data_s[i].find_first_of("Mm")!=string::npos)
	        {
	        	throw runtime_error("Unexpected end of line.\n"
	        			"Please check if the SVG path is drawn without new path.");

	        }
	        else if(data_s[i].find_first_of("Cc")!=string::npos)
	        {
	            if(data_s[i].find_first_of("C")!=string::npos) abs_rel=true;
	            else if(data_s[i].find_first_of("c")!=string::npos) abs_rel=false;
	            //only take the third point
	            i+=4;
	            double offsetx=0, offsety=0;
	            if(!abs_rel)
	            {
	                offsetx = positions[positions.size()-1].x;
	                offsety = positions[positions.size()-1].y;
	            }
	            pos.x = atof(data_s[++i].c_str())+offsetx;
	            pos.y = atof(data_s[++i].c_str())+offsety;
	            found_points++;
	            positions.push_back(pos);

	        }
	        else if(data_s[i].find_first_of("Zz")==string::npos)
	        {
	            if(data_s[i].find_first_of("L")!=string::npos) abs_rel=true;
	            else if(data_s[i].find_first_of("l")!=string::npos) abs_rel=false;
	            else i--;

	            double offsetx=0, offsety=0;
	            if(!abs_rel)
	            {
	                offsetx = positions[positions.size()-1].x;
	                offsety = positions[positions.size()-1].y;
	            }
	            pos.x = atof(data_s[++i].c_str())+offsetx;
	            pos.y = atof(data_s[++i].c_str())+offsety;
	            found_points++;
	            positions.push_back(pos);
	        }
	        else
	        {
	            if(VERBOSE) cout<<"Closepath command ignored."<<endl;
	        }


	    }
	    if(VERBOSE)
	    {
	        cout<<"Found points: "<< found_points<<" . Size of path: "<< positions.size()<<endl;
	        cout<<"The above should match for a successful parse"<<endl;
	    }
	    if(found_points!=positions.size()) throw (string)"Size not match";
	    return positions;
	}
};
