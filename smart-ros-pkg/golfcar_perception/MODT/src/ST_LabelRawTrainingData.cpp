#ifndef ST_LABEL_RAW_TRAINING_DATA
#define ST_LABEL_RAW_TRAINING_DATA

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <vector>
#include <stdlib.h>
#include "DATMO_datatypes.h"

using namespace std;
using namespace cv;

class DATMO_labellingData
{
public:
	DATMO_labellingData();
	void main_loop();
private:

	void load_param_file();
	void read_data_batch(int batch_serial);
	void label_data_batch(bool perform_fast_labeling);
	void update_labelling_history();

	void fast_label(int batch_serial, int skipTo_to_batch);
	void visualize_labelling_results();
	void save_abstract_result();

	ros::NodeHandle private_nh_;
	int starting_serial_, end_serial_, interval_;
	int scan_range_size_;
	cv::Size visualization_image_size_;
	string param_file_path_;
	string input_path_;
	string output_path_;

	DATMO_RawTrainingData raw_training_data_;

	std::vector<sensor_msgs::PointCloud> prev_clusters_, prev_clusters_copy_;
	std::vector<geometry_msgs::Point32>  prev_centroids_, prev_centroids_copy_;
	std::vector<int> prev_cluster_labels_, prev_cluster_labels_copy_;

	std::vector<sensor_msgs::PointCloud> curr_clusters_;
	std::vector<int> curr_cluster_labels_;
	std::vector<geometry_msgs::Point32>  curr_centroids_;

	DATMO_abstractSummary AbstractLabelling_, AbstractLabelling_copy_;

	double													img_side_length_, img_resolution_;
	cv::Mat													local_mask_;
	cv::Point												LIDAR_pixel_coord_;
	void initialize_local_image();
	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point2f & imgPt);
	inline bool LocalPixelValid(Point2f & imgPt);

	void fast_labeling_save_a_copy();
	void fast_labeling_recover_from_copy();
};

DATMO_labellingData::DATMO_labellingData():private_nh_("~")
{
	private_nh_.param("param_file_path", param_file_path_, std::string("/home/baoxing/data/DATMO_Label_Parameters.yaml"));

	load_param_file();

	//pay special attention to the serial: "starting_serial_" is the serial name of the first batch file;
	//									   "end_serial_" is the serial name of the last batch file;
	//									   only keep those labels with "interval_-1" scans both before and after, to guarantee reliable result;
	AbstractLabelling_.labelled_scan_startSerial = starting_serial_;
	AbstractLabelling_.labelled_scan_endSerial = end_serial_ - interval_+1;
	AbstractLabelling_.process_scan_startSerial = starting_serial_-interval_+1;
	AbstractLabelling_.process_scan_endSerial = end_serial_;

	int total_scans_involved = end_serial_ - starting_serial_+interval_;

	std::vector<int> default_mask(scan_range_size_, 0);
	AbstractLabelling_.masks_under_processing.resize(total_scans_involved, default_mask);

	initialize_local_image();
}

void DATMO_labellingData::load_param_file()
{
	FileStorage fs_read(param_file_path_, FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("cannot find parameter file"); return;}

	starting_serial_ = (int)fs_read["starting_serial"];
	end_serial_ = (int)fs_read["end_serial"];
	interval_ = (int)fs_read["interval"];
	scan_range_size_ = (int)fs_read["scan_range_size"];

	visualization_image_size_.height = (int)fs_read["image_height"];
	visualization_image_size_.width = (int)fs_read["image_width"];
	fs_read["input_path"]>> input_path_;
	fs_read["output_path"]>> output_path_;
	img_side_length_ = (double)fs_read["img_side_length"];
	img_resolution_ = (double)fs_read["img_resolution"];

	AbstractLabelling_.type_masks.clear();
	fs_read.release();
}

void DATMO_labellingData::read_data_batch(int batch_serial)
{
	raw_training_data_.scan_ClusterLabel_vector.clear();
	raw_training_data_.scan_serials.clear();
	raw_training_data_.clusters_odom.clear();
	raw_training_data_.clusters_baselink.clear();

	stringstream  data_batch_string;
	data_batch_string<<input_path_.c_str()<<"/data/training_data"<< batch_serial <<".yml";

	cout<<data_batch_string.str().c_str()<<endl;

	FileStorage fs_read(data_batch_string.str().c_str(), FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("cannot find data batch file"); return;}

	FileNode scan_serials = fs_read["scan_serials"];
	scan_serials >> raw_training_data_.scan_serials;

	raw_training_data_.scan_ClusterLabel_vector.resize(interval_);
	FileNode scan_ClusterLabel_vector = fs_read["scan_ClusterLabel_vector"];
	FileNodeIterator scan_ClusterLabel_vector_it = scan_ClusterLabel_vector.begin(), scan_ClusterLabel_vector_it_end = scan_ClusterLabel_vector.end();
	for(int idx=0; scan_ClusterLabel_vector_it != scan_ClusterLabel_vector_it_end; scan_ClusterLabel_vector_it++, idx++)
	{
		(*scan_ClusterLabel_vector_it)>>raw_training_data_.scan_ClusterLabel_vector[idx];
	}

	cout<<"odom_points"<<endl;

	FileNode odom_points = fs_read["odom_points"];
	FileNodeIterator odom_points_it = odom_points.begin(), odom_points_it_end = odom_points.end();
	for(int cluster_idx=0; odom_points_it != odom_points_it_end; odom_points_it++, cluster_idx++)
	{
		sensor_msgs::PointCloud cluster_tmp;
		FileNode cluster_odom_points = (*odom_points_it);
		FileNodeIterator cluster_odom_points_it = cluster_odom_points.begin(), cluster_odom_points_it_end = cluster_odom_points.end();
		for(int point_idx=0; cluster_odom_points_it != cluster_odom_points_it_end; cluster_odom_points_it++, point_idx++)
		{
			geometry_msgs::Point32 odom_point;
			odom_point.x = (*cluster_odom_points_it)["x"];
			odom_point.y = (*cluster_odom_points_it)["y"];
			cluster_tmp.points.push_back(odom_point);
		}
		raw_training_data_.clusters_odom.push_back(cluster_tmp);
	}

	cout<<"baselink_points"<<endl;

	FileNode baselink_points = fs_read["baselink_points"];
	FileNodeIterator baselink_points_it = baselink_points.begin(), baselink_points_it_end = baselink_points.end();
	for(int cluster_idx=0; baselink_points_it != baselink_points_it_end; baselink_points_it++, cluster_idx++)
	{
		sensor_msgs::PointCloud cluster_tmp;
		FileNode cluster_baselink_points = (*baselink_points_it);
		FileNodeIterator cluster_baselink_points_it = cluster_baselink_points.begin(), cluster_baselink_points_it_end = cluster_baselink_points.end();
		for(int point_idx=0; cluster_baselink_points_it != cluster_baselink_points_it_end; cluster_baselink_points_it++, point_idx++)
		{
			geometry_msgs::Point32 baselink_point;
			baselink_point.x = (*cluster_baselink_points_it)["x"];
			baselink_point.y = (*cluster_baselink_points_it)["y"];
			cluster_tmp.points.push_back(baselink_point);
		}
		raw_training_data_.clusters_baselink.push_back(cluster_tmp);
	}

	//cout<<"finish loading data"<<endl;
	fs_read.release();
}

void DATMO_labellingData::label_data_batch(bool perform_fast_labeling)
{
	stringstream  image_visual_string;
	image_visual_string<<input_path_.c_str()<<"/vis_image/image_"<< raw_training_data_.scan_serials.back()<<".jpg";
	Mat contour_visual_img = imread(image_visual_string.str().c_str(), CV_LOAD_IMAGE_COLOR);
	imshow("contour_visual_img",contour_visual_img);
	waitKey(10);

	curr_clusters_ = raw_training_data_.clusters_odom;
	curr_cluster_labels_.clear();
	curr_cluster_labels_.resize(raw_training_data_.clusters_baselink.size(), 0);
	curr_centroids_.clear();

	for(size_t i=0; i<curr_clusters_.size(); i++)
	{
		geometry_msgs::Point32 centroid_point;
		centroid_point.x = 0.0;
		centroid_point.y = 0.0;
		centroid_point.z = 0.0;

		for(size_t j=0; j<curr_clusters_[i].points.size(); j++)
		{
			centroid_point.x = centroid_point.x + curr_clusters_[i].points[j].x;
			centroid_point.y = centroid_point.y + curr_clusters_[i].points[j].y;
		}
		centroid_point.x = centroid_point.x/(float)curr_clusters_[i].points.size();
		centroid_point.y = centroid_point.y/(float)curr_clusters_[i].points.size();
		curr_centroids_.push_back(centroid_point);
	}

	std::vector<int> default_classtype (scan_range_size_, 0);
	raw_training_data_.scan_ClassType_vector.clear();
	raw_training_data_.scan_ClassType_vector.resize(raw_training_data_.scan_ClusterLabel_vector.size(), default_classtype);

	if(!perform_fast_labeling)
	{
		for(size_t i=0; i<raw_training_data_.clusters_baselink.size(); i++)
		{
			Vec3b cluster_color;
			cluster_color.val[0] = 0;
			cluster_color.val[1] = 255;
			cluster_color.val[2] = 0;

			for(size_t j=0; j<raw_training_data_.clusters_baselink[i].points.size(); j++)
			{
				Point2f imgpt_tmp;
				spacePt2ImgP(raw_training_data_.clusters_baselink[i].points[j], imgpt_tmp);

				if(LocalPixelValid(imgpt_tmp))
				{
					contour_visual_img.at<Vec3b>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = cluster_color;
				}
			}

			imshow("contour_visual_img",contour_visual_img);
			waitKey(100);

			int class_type = 0;
			printf("please key in the object type of this contour\n");
			scanf("%d", &class_type);
			curr_cluster_labels_[i] = class_type;
		}
	}
	else
	{
		cout<<"fast labelling"<<endl;
		cout<<"current cluster size, prev_clusters size: "<<curr_centroids_.size()<<","<<prev_centroids_.size()<<endl;

		//try to inherit current cluster labels from previous cluster, who has overlap between each other;
		double attach_threshold = 1.0;

		for(size_t i=0; i<curr_centroids_.size(); i++)
		{
			geometry_msgs::Point32 &probe_point = curr_centroids_[i];
			bool attach_prev_cluster = false;
			int attach_cluster = 0;

			for(size_t j=0; j<prev_centroids_.size(); j++)
			{
				geometry_msgs::Point32 &check_point = prev_centroids_[j];
				double distance_undercheck = sqrt((check_point.x- probe_point.x)*(check_point.x- probe_point.x)+(check_point.y- probe_point.y)*(check_point.y- probe_point.y));
				if(distance_undercheck<attach_threshold)
				{
					attach_prev_cluster =true;
					attach_cluster = int(j);
					break;
				}
			}

			if(attach_prev_cluster)
			{
				curr_cluster_labels_[i]=prev_cluster_labels_[attach_cluster];
			}
			else
			{
				//if cannot find, then treat it as "background";
				//so when choose fast-labelling, make sure there is no new unlabelled object appear;
				curr_cluster_labels_[i]=0;
			}
		}
	}

	//import the cluster class label into scan label;
	for(size_t i=0; i<raw_training_data_.scan_ClusterLabel_vector.size(); i++)
	{
		for(size_t j=0; j<raw_training_data_.scan_ClusterLabel_vector[i].size(); j++)
		{
			int &cluster_label_tmp 	= raw_training_data_.scan_ClusterLabel_vector[i][j];
			int &class_label_tmp 	= raw_training_data_.scan_ClassType_vector[i][j];

			if(cluster_label_tmp == -1) class_label_tmp=0;
			else
			{
				assert(cluster_label_tmp>=0 && cluster_label_tmp  < (int)curr_cluster_labels_.size());
				{
					class_label_tmp = curr_cluster_labels_[cluster_label_tmp];
					//cout<<cluster_label_tmp<<","<<class_label_tmp<<"\t";
				}
			}
		}
	}

	prev_clusters_ = curr_clusters_;
	prev_cluster_labels_ = curr_cluster_labels_;
	prev_centroids_ = curr_centroids_;

	//visualize the final results;
	for(size_t i=0; i<raw_training_data_.clusters_baselink.size(); i++)
	{
		Vec3b vehicle_color;
		vehicle_color.val[0] = 0;
		vehicle_color.val[1] = 0;
		vehicle_color.val[2] = 255;

		int labelled_class_type = curr_cluster_labels_[i];
		if(labelled_class_type == 1)
		{
			for(size_t j=0; j<raw_training_data_.clusters_baselink[i].points.size(); j++)
			{
				Point2f imgpt_tmp;
				spacePt2ImgP(raw_training_data_.clusters_baselink[i].points[j], imgpt_tmp);

				if(LocalPixelValid(imgpt_tmp))
				{
					contour_visual_img.at<Vec3b>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = vehicle_color;
				}
			}

			imshow("contour_visual_img",contour_visual_img);
			waitKey(100);
		}
	}


}


void DATMO_labellingData::save_abstract_result()
{
	ROS_INFO("save abstract results");

	for(int i = AbstractLabelling_.labelled_scan_startSerial; i<= AbstractLabelling_.labelled_scan_endSerial; i=i+1)
	{
		int serial_in_underprocessing = i - AbstractLabelling_.process_scan_startSerial;

		std::vector<int> mask_tmp = AbstractLabelling_.masks_under_processing[serial_in_underprocessing];
		AbstractLabelling_.type_masks.push_back(mask_tmp);
	}

	stringstream  abstract_summary;
	abstract_summary<<output_path_.c_str()<<"/abstract_summary.yml";
	cout<<"save abstract results"<<abstract_summary.str().c_str()<<endl;

	FileStorage fs(abstract_summary.str().c_str(), FileStorage::WRITE);
	if(!fs.isOpened()){ROS_ERROR("cannot write abstract_summary file"); return;}

	fs <<"labelled_scan_startSerial"<<AbstractLabelling_.labelled_scan_startSerial;
	fs <<"labelled_scan_endSerial"<<AbstractLabelling_.labelled_scan_endSerial;
	fs << "type_masks" << "[";
	for(size_t i=0; i<AbstractLabelling_.type_masks.size(); i++)
	{
		fs<<"[:";
		for(size_t j=0; j<AbstractLabelling_.type_masks[i].size(); j++)
		{
			fs<<AbstractLabelling_.type_masks[i][j];
		}
		fs << "]" ;
	}
	fs <<"]";

	fs.release();
}

void DATMO_labellingData::fast_label(int batch_serial, int skipTo_to_batch)
{
	ROS_INFO("do fast labelling for batches between (but not included) %d and %d", batch_serial, skipTo_to_batch);
	for(int i=batch_serial+1; i<skipTo_to_batch; i=i+1)
	{
		read_data_batch(i);
		label_data_batch(true);
		update_labelling_history();
	}
}

void DATMO_labellingData::fast_labeling_save_a_copy()
{
	prev_clusters_copy_ = prev_clusters_;
	prev_centroids_copy_ = prev_centroids_;
	prev_cluster_labels_copy_ = prev_cluster_labels_;
	AbstractLabelling_copy_ = AbstractLabelling_;
}
void DATMO_labellingData::fast_labeling_recover_from_copy()
{
	prev_clusters_ = prev_clusters_copy_;
	prev_centroids_ = prev_centroids_copy_;
	prev_cluster_labels_ = prev_cluster_labels_copy_;
	AbstractLabelling_ = AbstractLabelling_copy_;
}

void DATMO_labellingData::update_labelling_history()
{
	int current_head_serial = raw_training_data_.scan_serials.front();
	int current_tail_serial = raw_training_data_.scan_serials.back();
	assert(current_tail_serial = current_head_serial+interval_);

	size_t head_in_history = current_head_serial - AbstractLabelling_.process_scan_startSerial;

	for(size_t i=0; i<raw_training_data_.scan_ClassType_vector.size(); i++)
	{
		std::vector<int> &labelled_scan_mask 	= raw_training_data_.scan_ClassType_vector[i];
		std::vector<int> &history_scan_mask		= AbstractLabelling_.masks_under_processing[i+head_in_history];
		assert(labelled_scan_mask.size() == history_scan_mask.size());
		for(size_t j=0; j<labelled_scan_mask.size(); j++)
		{
			if(history_scan_mask[j]==0 && labelled_scan_mask[j]!=0) history_scan_mask[j] = labelled_scan_mask[j];
		}
	}

}

//interactive data labelling: use terminal or GUI interface;
void DATMO_labellingData::main_loop()
{
	ROS_INFO("begin to label data");

	namedWindow( "contour_visual_img", 1 );

	for(int batch_serial = starting_serial_; batch_serial <= end_serial_; )
	{
		ROS_INFO("---------------------------------------------------------------");
		ROS_INFO("batch serial %d", batch_serial);

		read_data_batch(batch_serial);
		label_data_batch(false);

        int use_the_result = 0;
        printf("@@@@@@@@@@@@@@@@@@ use your labelled result? Press 0 to redo it, Press 1 to accept @@@@@@@@@@@@@@ \n");
        scanf("%d", &use_the_result);

        if(use_the_result == 0)
        {
        	printf("redo the labelling of this batch\n");
        }
        else if(use_the_result == 1)
        {
        	printf("labelling of this batch is accepted, fuse it with the history.\n");
        	update_labelling_history();

        	int skipTo_to_batch = 0;
    		printf("****************skip to certain batch? Please enter the number**************\n");
    		scanf("%d", &skipTo_to_batch);

    		if(skipTo_to_batch > batch_serial+1)
			{
    			printf("jump to batch %d, will do fast labelling for scans in between\n", skipTo_to_batch);
    			fast_labeling_save_a_copy();
    			fast_label(batch_serial, skipTo_to_batch);
		        int use_the_fast_result = 0;

		        printf("use the fast result? 0 to redo it, Press 1 to accept \n");
		        scanf("%d", &use_the_fast_result);

		        if(use_the_fast_result == 0)
		        {
		        	printf("cancel the fast result\n");
		        	fast_labeling_recover_from_copy();
		        }
		        else if(use_the_fast_result == 1)
		        {
		        	printf("use the fast result\n");
		        	batch_serial=skipTo_to_batch;
		        }
		        else
		        {
		        	printf("Only 0, 1 is valid.\n");
		        }
			}
    		else
			{
    			printf("illegal skip, will process next batch %d\n", batch_serial+1);
    			batch_serial=batch_serial+1;
			}
        }
        else
        {
        	printf("Only 0, 1 is valid.\n");
        }
	}

	save_abstract_result();
}

void DATMO_labellingData::initialize_local_image()
{
	local_mask_				= Mat((int)(img_side_length_/img_resolution_*2), (int)(img_side_length_/img_resolution_*2), CV_8UC1);
	local_mask_				= Scalar(0);
	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;

	vector<Point2f> img_roi;
	Point2f p0, p1, p2, p3, p4;
	p0.x = 0.0;
	p0.y = 0.0;
	p1.x = 0.0;
	p1.y = float(local_mask_.rows-1);
	p2.x = float(LIDAR_pixel_coord_.x);
	p2.y = float(LIDAR_pixel_coord_.y);
	p3.x = float(local_mask_.cols-1);
	p3.y = float(local_mask_.rows-1);
	p4.x = float(local_mask_.cols-1);
	p4.y = 0.0;

	img_roi.push_back(p0);
	img_roi.push_back(p1);
	img_roi.push_back(p2);
	img_roi.push_back(p3);
	img_roi.push_back(p4);

	for(int i=0; i<(int)local_mask_.cols; i++)
		for(int j=0; j<(int)local_mask_.rows; j++)
		{
			Point2f point_tmp;
			point_tmp.x = (float)i;
			point_tmp.y = (float)j;
			if(cv::pointPolygonTest(img_roi, point_tmp, false)>0)local_mask_.at<uchar>(j,i)=255;
		}
}

void DATMO_labellingData::spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point2f & imgPt)
{
	imgPt.x = LIDAR_pixel_coord_.x - (spacePt.y/img_resolution_);
	imgPt.y = LIDAR_pixel_coord_.y - (spacePt.x/img_resolution_);
}

inline bool DATMO_labellingData::LocalPixelValid(Point2f & imgPt)
{
	if((int)imgPt.x < local_mask_.cols && (int)imgPt.x >=0 && (int)imgPt.y < local_mask_.rows && (int)imgPt.y >=0) return true;
	else return false;
}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "DATMO_labellingData_node");
	DATMO_labellingData DATMO_labellingData_node;
	DATMO_labellingData_node.main_loop();
	return (0);
}

#endif
