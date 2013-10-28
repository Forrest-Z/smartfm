/*
 * norm_virtual_sensor.cpp
 *
 *  Created on: Sep 2, 2012
 *      Author: demian
 */
#include "AccumulateData.h"
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <norm_virtual_sensor/NormVirtualSensorConfig.h>
#include <pcl/point_types.h>

class NormVirtualSensor
{
    AccumulateData *laser_accumulate_;
    string target_frame_;



    vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > accumulated_normals_;
    ros::Publisher accumulated_pub_, normal_pc2_pub_;
    ros::Publisher final_pc2_pub_, final_pc_pub_, final_pcl_pub_;
    ros::Publisher latest_normal_pub_, laser_pub_;
    ros::Publisher normals_poses_pub_, filtered_normal_pub_;
    tf::TransformListener *tf_;
    tf::StampedTransform cur_sensor_trans_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>       *laser_scan_filter_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    tf::MessageFilter<sensor_msgs::PointCloud2>       *pointcloud_filter_;

    dynamic_reconfigure::Server<norm_virtual_sensor::NormVirtualSensorConfig> dynamic_server_;
    dynamic_reconfigure::Server<norm_virtual_sensor::NormVirtualSensorConfig>::CallbackType dynamic_server_cb_;

    bool publish_normals_;
    bool min_pc2laser_;
    double downsample_size_, norm_radius_search_, min_move_dist_, normal_thres_,
    		density_radius_search_,density_min_neighbors_;
    int accummulate_buffer_;
    unsigned int accumulate_size_;

    void dynamicCallback(norm_virtual_sensor::NormVirtualSensorConfig &config, uint32_t level)
    {
    	min_pc2laser_ = config.min_pc2laser;
    	publish_normals_ = config.publish_normals;
    	downsample_size_ = config.downsample_size;
    	norm_radius_search_ = config.normal_radius_search;
    	normal_thres_ = config.normal_thres;
    	density_radius_search_ = config.density_radius_search;
    	density_min_neighbors_ = config.density_min_neighbors;
    	min_move_dist_ = config.min_move_dist;
    	accumulate_size_ = config.accumulate_size;//100;
    	accummulate_buffer_ = config.accummulate_buffer;

    	int scan_buffer = norm_radius_search_/min_move_dist_ + accummulate_buffer_;
    	laser_accumulate_->updateParameter(min_move_dist_, scan_buffer);

    }

    inline float labelToRGB(uint8_t i)
    {
        //pack scan number into RGB
        uint8_t r = i, g = 0, b = 0; // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        return *reinterpret_cast<float*>(&rgb);
    }

    pcl::PointCloud<pcl::PointXYZRGB> labelScan(vector<sensor_msgs::PointCloud> &data, std_msgs::Header header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pcl;

        for(size_t i=0; i<data.size(); i++)
        {

            sensor_msgs::PointCloud2 temp;
            sensor_msgs::convertPointCloudToPointCloud2(data[i], temp);
            pcl::PointCloud<pcl::PointXYZ> pcl_temp;
            pcl::fromROSMsg(temp,  pcl_temp);
            pcl::PointCloud<pcl::PointXYZRGB> pclxyzrgbnorm_temp;
            pclxyzrgbnorm_temp.resize(pcl_temp.size());

            for(size_t j=0; j<pcl_temp.points.size(); j++)
            {
                pclxyzrgbnorm_temp.points[j].x = pcl_temp[j].x;
                pclxyzrgbnorm_temp.points[j].y = pcl_temp[j].y;
                pclxyzrgbnorm_temp.points[j].z = pcl_temp[j].z;
                pclxyzrgbnorm_temp.points[j].rgb = labelToRGB(i);
            }
            pcl += pclxyzrgbnorm_temp;
        }
        pcl_downsample(pcl);
        cout<<"Downsampled size "<<pcl.size()<<endl;
        sensor_msgs::PointCloud concatenated_pts;
        concatenated_pts.header = header;
        concatenated_pts.header.frame_id = target_frame_;
        concatenated_pts.points.resize(pcl.size());
        for(size_t i=0; i<pcl.size(); i++)
        {
            concatenated_pts.points[i].x = pcl.points[i].x;
            concatenated_pts.points[i].y = pcl.points[i].y;
            concatenated_pts.points[i].z = pcl.points[i].z;
        }
        vector<sensor_msgs::PointCloud> all_accumulated = all_data_;
        
	
	sensor_msgs::PointCloud all_accumulated_pc;
	all_accumulated_pc.header = concatenated_pts.header;
	for(size_t i=0; i<all_accumulated.size(); i++) {
	  all_accumulated_pc.points.insert(all_accumulated_pc.points.begin(), all_accumulated[i].points.begin(), all_accumulated[i].points.end());
	}
	accumulated_pub_.publish(all_accumulated_pc);
	
	//accumulated_pub_.publish(concatenated_pts);
        return pcl;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>& input)
    {
        //if(input.points.size() == 0) return;
        //down sampling moved to rolling windows
        fmutil::Stopwatch sw;

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::PointXYZRGB min, max, centroid;
        pcl::getMinMax3D(input, min, max);
        pcl::PointCloud<pcl::PointXYZRGB> normalized_input;
        centroid.x = (max.x + min.x)/2.;
        centroid.y = (max.y + min.y)/2.;
        centroid.z = (max.z + min.z)/2.;
        Eigen::Vector3f offset(-centroid.x, -centroid.y, -centroid.z);
        Eigen::Quaternion<float> qt(1.0,0,0,0);

        pcl_utils::transformPointCloud(input, normalized_input, offset, qt);

        ne.setInputCloud (normalized_input.makeShared());

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal> cloud_normals;

        // Use all neighbors in a sphere of radius 10cm
        ne.setRadiusSearch (norm_radius_search_);

        // Set use point using current sensor pose

        double viewpoint[] = {cur_sensor_trans_.getOrigin().x()-centroid.x, cur_sensor_trans_.getOrigin().y()-centroid.y, cur_sensor_trans_.getOrigin().z()-centroid.z};
        //cout<<"Sensor pose in target frame: "<<viewpoint[0]<<" "<<viewpoint[1]<<" "<<viewpoint[2]<<endl;
        ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
        //ne.setViewPoint(0.,0.,0.);
        // Compute the features
        stringstream ss;
        ss<<"Compute normals in frame "<<input.header.frame_id;
        sw.start(ss.str());
        ne.compute (cloud_normals);
        sw.end();


        // concatentate the fileds
        pcl::PointCloud<pcl::PointXYZRGBNormal> point_normals;
        sw.start("Concatenate fields");
        pcl::concatenateFields(input, cloud_normals, point_normals);
        sw.end();
        //cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;

        //publish normals as pose arrays if needed
        if(publish_normals_)
        	publishNormals(point_normals);

        return point_normals;

    }

    void publishNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_cloud)
    {
    	fmutil::Stopwatch sw;
    	sw.start("Normal calculate");
    	geometry_msgs::PoseArray normals_poses;
    	normals_poses.header.frame_id = target_frame_;
    	normals_poses.header.stamp = ros::Time::now();
    	for(unsigned int i=0; i<pcl_cloud.points.size(); i++)
    	{

    		geometry_msgs::Pose normals_pose;
    		geometry_msgs::Point pos;
    		pos.x = pcl_cloud.points[i].x; pos.y = pcl_cloud.points[i].y; pos.z = pcl_cloud.points[i].z;

    		normals_pose.position = pos;
    		btVector3 axis(pcl_cloud.points[i].normal[0],pcl_cloud.points[i].normal[1],pcl_cloud.points[i].normal[2]);
    		if(isnan(pcl_cloud.points[i].normal[0])||isnan(pcl_cloud.points[i].normal[1])||isnan(pcl_cloud.points[i].normal[2])) continue;
    		//cout<<axis.x()<<" "<<axis.y()<<" "<<axis.z()<<" "<<axis.w()<<endl;
    		btVector3 marker_axis(1, 0, 0);
    		btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
    		double yaw, pitch, roll;
    		btMatrix3x3(qt).getEulerYPR(yaw, pitch, roll);
    		geometry_msgs::Quaternion quat_msg;
    		tf::quaternionTFToMsg(qt, quat_msg);
    		if(isnan(qt.x())||isnan(qt.y())||isnan(qt.z())||isnan(qt.w())) continue;
    		normals_pose.orientation.x = qt.x();// = quat_msg;
    		normals_pose.orientation.y = qt.y();
    		normals_pose.orientation.z = qt.z();
    		normals_pose.orientation.w = qt.w();

    		normals_poses.poses.push_back(normals_pose);
    	}
    	normals_poses_pub_.publish(normals_poses);
    	sensor_msgs::PointCloud2 pc2_msg;
    	pcl::toROSMsg(pcl_cloud, pc2_msg);
    	pc2_msg.header = normals_poses.header;
    	normal_pc2_pub_.publish(pc2_msg);
    	sw.end();
    }
    pcl::PointCloud<pcl::PointXYZRGBNormal> filterNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_cloud)
    {
        fmutil::Stopwatch sw;
        stringstream ss;
        ss<<"Filter clouds with "<<pcl_cloud.size()<<" pts";
        sw.start(ss.str());

        //absolutely stunningly quick!
        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZRGBNormal> ("normal_z", pcl::ComparisonOps::LT, normal_thres_)));
        //range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        //      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem (range_cond);
        condrem.setInputCloud (pcl_cloud.makeShared());
        condrem.filter (pcl_cloud);

        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZRGBNormal> ("normal_z", pcl::ComparisonOps::GT, -normal_thres_)));
        //range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        //      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem2 (range_cond2);
        condrem2.setInputCloud (pcl_cloud.makeShared());
        condrem2.filter (pcl_cloud);
        sw.end();
        cout<<"After normal filter "<<pcl_cloud.size()<<endl;
        sensor_msgs::PointCloud2 filtered_pts;
        pcl::toROSMsg(pcl_cloud, filtered_pts);
        filtered_pts.header.frame_id = target_frame_;
        filtered_pts.header.stamp = ros::Time::now();
        filtered_normal_pub_.publish(filtered_pts);
        if(pcl_cloud.size()==0) return pcl_cloud;

        //perform density based filtering
        sw.start("Density filtering");
        pcl::PointCloud<pcl::PointXYZRGBNormal> radius_filtered_pcl2;// = pcl_cloud;
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
        outrem.setInputCloud(pcl_cloud.makeShared());
        outrem.setRadiusSearch(density_radius_search_);
        outrem.setMinNeighborsInRadius(density_min_neighbors_);
        outrem.filter(radius_filtered_pcl2);
        sw.end();
        cout<<"Remaining clouds: "<<radius_filtered_pcl2.size()<<endl;
        return radius_filtered_pcl2;
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

    template<class T>
    void pcl_downsample(pcl::PointCloud<T> &point_cloud)
    {
    	pcl_downsample(point_cloud, downsample_size_, downsample_size_, downsample_size_);
    }

    void pcCallback(sensor_msgs::PointCloud2ConstPtr scan)
    {
        sensor_msgs::PointCloud2 pc2 = *scan;
        tf_->lookupTransform(target_frame_, scan->header.frame_id, scan->header.stamp, cur_sensor_trans_);
        laser_accumulate_->addData(pc2, *tf_);
        mainProcess(scan->header);
    }

    void scanCallback(sensor_msgs::LaserScanConstPtr scan)
    {
        sensor_msgs::LaserScan ls = *scan;
        tf_->lookupTransform(target_frame_, scan->header.frame_id, scan->header.stamp, cur_sensor_trans_);
        laser_accumulate_->addData(ls, *tf_);
        mainProcess(scan->header);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> getLabeledPointCloud( pcl::PointCloud<pcl::PointXYZRGBNormal> &input, unsigned int label)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal> single_pcl;
        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZRGBNormal> ("rgb", pcl::ComparisonOps::EQ, labelToRGB(label))));

        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem (range_cond);
        condrem.setInputCloud (input.makeShared());
        condrem.filter (single_pcl);
        return single_pcl;
    }
    vector<sensor_msgs::PointCloud> all_data_;
    void mainProcess(std_msgs::Header header)
    {
        fmutil::Stopwatch sw;
        sw.start("++++Total++++++: ");
        vector<sensor_msgs::PointCloud> data;
        if(laser_accumulate_->getLatestAccumulated(data))
        {
            //RGB used as the scan_no
            all_data_ = data;
            pcl::PointCloud<pcl::PointXYZRGB> pcl_labeled = labelScan(data, header);
            pcl::PointCloud<pcl::PointXYZRGBNormal> temp = normalEstimation(pcl_labeled);
            pcl::PointCloud<pcl::PointXYZRGBNormal> processed_pcl = filterNormal(temp);
            pcl::PointCloud<pcl::PointXYZRGBNormal> single_pcl = getLabeledPointCloud(processed_pcl, (unsigned int)(data.size()/2.0));


            //get the latest strip of pcl which can be considered as unstable but useful to ensure latest information is incorporated when reconstruction
            pcl::PointCloud<pcl::PointXYZRGBNormal> latest_pcl;
            for(size_t i= 0; i<(unsigned int)(data.size()/2.0); i++)
            {
                latest_pcl += getLabeledPointCloud(processed_pcl, i);
            }
            cout<<"Injecting latest normal with "<<latest_pcl.size()<<" pts"<<endl;
            //rebuild normals with old and new normals
            accumulated_normals_.insert(accumulated_normals_.begin(), single_pcl);
            if(accumulated_normals_.size()>accumulate_size_) accumulated_normals_.resize(accumulate_size_);
            cout<<"Accumulated with "<<accumulated_normals_.size()<<" normal points"<<endl;
            pcl::PointCloud<pcl::PointXYZRGBNormal> rebuild_normal;
            for(size_t i=0; i<accumulated_normals_.size(); i++)
            {
                rebuild_normal+=accumulated_normals_[i];
            }
            rebuild_normal+=latest_pcl;
            //include a pointcloud that has a frame parrallel to the odom frame but move with base_link
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(rebuild_normal, out);
            out.header = header;
            out.header.frame_id = target_frame_;
            final_pc2_pub_.publish(out);
            sensor_msgs::PointCloud out_pc_legacy;
            cout<<"before 2d compress: "<<rebuild_normal.size()<<endl;
            for(size_t i=0; i<rebuild_normal.size(); i++)
              rebuild_normal[i].z = 0.0;
            pcl_downsample(rebuild_normal, 0.05, 0.05, 0.05);
            cout<<"After 2d compress: "<<rebuild_normal.size()<<endl;
            pcl::toROSMsg(rebuild_normal, out);
            out.header = header;
            out.header.frame_id = target_frame_;
            sensor_msgs::convertPointCloud2ToPointCloud(out, out_pc_legacy);
            try
            {
            	tf_->transformPointCloud("/odom_baselink", out_pc_legacy, out_pc_legacy);
            }
            catch (tf::ExtrapolationException ex)
            {
            	cout<<ex.what()<<endl;
            }
            final_pc_pub_.publish(out_pc_legacy);
            //just to check
            assert(rebuild_normal.points.size() == out_pc_legacy.points.size());
	          tf::StampedTransform baselink_transform;
            tf_->lookupTransform(out_pc_legacy.header.frame_id, target_frame_, out_pc_legacy.header.stamp, baselink_transform);
	          Eigen::Quaternionf bl_rotation(baselink_transform.getRotation().w(), baselink_transform.getRotation().x(),baselink_transform.getRotation().y(),baselink_transform.getRotation().z()); 
            Eigen::Vector3f bl_trans(baselink_transform.getOrigin().x(),baselink_transform.getOrigin().y(),baselink_transform.getOrigin().z());
//         btQuaternion bt_bl_qt = baselink_transform.getRotation(); btVector3 bt_bl_origin = baselink_transform.getOrigin();
            //copy only the normals and transformed xyz
            pcl::PointCloud<pcl::PointXYZ> rebuild_normal_pcl;
            pcl::PointCloud<pcl::PointNormal> rebuild_point_normal;
            pcl::PointCloud<pcl::Normal> rebuild_normal_only;
            pcl::copyPointCloud<pcl::PointXYZRGBNormal, pcl::PointNormal>(rebuild_normal, rebuild_point_normal);
	          pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(rebuild_point_normal, rebuild_point_normal, bl_trans, bl_rotation);
            sensor_msgs::PointCloud2 rebuild_point_normal_pc2;
            pcl::toROSMsg(rebuild_point_normal, rebuild_point_normal_pc2);
            rebuild_point_normal_pc2.header = out_pc_legacy.header;
            final_pcl_pub_.publish(rebuild_point_normal_pc2);
            sensor_msgs::LaserScan out_laser;
            this->pointcloudsToLaser(out_pc_legacy, out_laser);
            laser_pub_.publish(out_laser);

            pcl::toROSMsg(latest_pcl, out);
            out.header = header;
            out.header.frame_id = target_frame_;
            latest_normal_pub_.publish(out);
            sw.end();
        }

        cout<<endl;
    }

    void pointcloudsToLaser(sensor_msgs::PointCloud& cloud, sensor_msgs::LaserScan& output)
    {
        //adapted from turtlebot's cloud_to_scan.cpp
        //cannot exceed 0.7 for compatibility of flirtlib
        output.header = cloud.header;
        output.angle_min = -M_PI*0.9;
        output.angle_max = M_PI*0.9;
        output.angle_increment = M_PI/180.0/5.0;
        output.time_increment = 0.0;
        output.scan_time = 1.0/30.0;
        output.range_min = 0.1;
        output.range_max = 80.0;

        uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
        output.ranges.assign(ranges_size, output.range_max + 1.0);

        for (size_t it = 0; it < cloud.points.size(); ++it)
        {
            const float &x = cloud.points[it].x;
            const float &y = cloud.points[it].y;
            const float &z = cloud.points[it].z;

            if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
            {
                ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
                continue;
            }

            double range_sq = y*y+x*x;
            /*if (range_sq < 0.) {
                ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
                continue;
            }*/

            double angle = -atan2(-y, x);
            if (angle < output.angle_min || angle > output.angle_max)
            {
                ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
                continue;
            }
            int index = (angle - output.angle_min) / output.angle_increment;


            //added to output the max range available from the point cloud
            //todo: more advanced processing of the points should use, right now just the min/max is sufficient

            if (output.ranges[index] == output.range_max + 1.0) output.ranges[index] = sqrt(range_sq);
            bool update_range;
            if(min_pc2laser_) update_range = output.ranges[index] * output.ranges[index] > range_sq;
            else update_range = output.ranges[index] * output.ranges[index] < range_sq;
            if (update_range)
                output.ranges[index] = sqrt(range_sq);
        }
    }


public:
    NormVirtualSensor(): norm_radius_search_(0.2), min_move_dist_(0.02)
    {
        ros::NodeHandle n;

        tf_ = new tf::TransformListener();

        target_frame_ = "/odom";

        laser_accumulate_ = new AccumulateData(target_frame_, min_move_dist_, norm_radius_search_/min_move_dist_+accummulate_buffer_);

        laser_scan_sub_.subscribe(n, "scan_in", 10);
        laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
        laser_scan_filter_->registerCallback(boost::bind(&NormVirtualSensor::scanCallback, this, _1));

        pointcloud_sub_.subscribe(n, "laser_surface_removed", 10);
        pointcloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pointcloud_sub_, *tf_, target_frame_, 10);
        pointcloud_filter_->registerCallback(boost::bind(&NormVirtualSensor::pcCallback, this, _1));

        accumulated_pub_ = n.advertise<sensor_msgs::PointCloud>("accumulated_pts", 10);
        final_pc2_pub_ = n.advertise<sensor_msgs::PointCloud2>("single_pcl", 10);
        final_pc_pub_ = n.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 10);
        final_pcl_pub_ = n.advertise<sensor_msgs::PointCloud2>("pcl_pointnormal_out", 10);
        latest_normal_pub_ = n.advertise<sensor_msgs::PointCloud2>("latest_normal", 10);
        filtered_normal_pub_ = n.advertise<sensor_msgs::PointCloud2>("filtered_normal", 10);
        laser_pub_ = n.advertise<sensor_msgs::LaserScan>("laser_out", 10);
    	normals_poses_pub_ = n.advertise<geometry_msgs::PoseArray>("normals_array", 10);
    	normal_pc2_pub_ = n.advertise<sensor_msgs::PointCloud2>("normal_pc2", 10);
    	dynamic_server_cb_ = boost::bind(&NormVirtualSensor::dynamicCallback, this, _1, _2);
    	dynamic_server_.setCallback(dynamic_server_cb_);

        ros::spin();
    }

};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "norm_virtual_sensor");
    NormVirtualSensor nvs;
    return 0;
}


