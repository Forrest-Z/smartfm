#ifndef DATMO_LABELLING_DATA
#define DATMO_LABELLING_DATA

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
	cv::Size visualization_image_size_;
	string param_file_path_;
	string input_path_;
	string output_path_;

	std::vector<int> cluster_label_history_;

	DATMO_RawTrainingData raw_training_data_;
	std::vector<sensor_msgs::PointCloud> prev_clusters_;
	std::vector<int> prev_cluster_labels_;

	DATMO_abstractSummary AbstractLabelling_;
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

	save_abstract_result();
}

void DATMO_labellingData::load_param_file()
{
	FileStorage fs_read(param_file_path_, FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("cannot find parameter file"); return;}

	starting_serial_ = (int)fs_read["starting_serial"];
	end_serial_ = (int)fs_read["end_serial"];
	interval_ = (int)fs_read["interval_"];

	visualization_image_size_.height = (int)fs_read["image_height"];
	visualization_image_size_.width = (int)fs_read["image_width"];
	fs_read["input_path"]>> input_path_;
	fs_read["output_path"]>> output_path_;

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
	data_batch_string<<input_path_.c_str()<<"/training_data"<< batch_serial <<".yml";
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

	FileNode odom_points = fs_read["odom_points"];
	FileNodeIterator odom_points_it = odom_points.begin(), odom_points_it_end = odom_points.end();
	for(int cluster_idx=0; odom_points_it != odom_points_it_end; odom_points_it_end++, cluster_idx++)
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

	FileNode baselink_points = fs_read["baselink_points"];
	FileNodeIterator baselink_points_it = baselink_points.begin(), baselink_points_it_end = baselink_points.end();
	for(int cluster_idx=0; baselink_points_it != baselink_points_it_end; baselink_points_it_end++, cluster_idx++)
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

	fs_read.release();
}

void DATMO_labellingData::label_data_batch(bool perform_fast_labeling)
{

	batch_T_.clear();
	for(size_t i=0; i<training_data_vector_.size(); i++)
	{
		DATMO_labelledScan single_scan_tmp;
		single_scan_tmp.laser_scan = training_data_vector_[i].laser_scan;
		single_scan_tmp.poseInOdom = training_data_vector_[i].poseInOdom;
		//initialize the mask label to be all 0;
		if(!miss_clustering)single_scan_tmp.type_mask.resize(single_scan_tmp.laser_scan.ranges.size(), 0);
		else single_scan_tmp.type_mask.resize(single_scan_tmp.laser_scan.ranges.size(), 4);
		batch_T_.push_back(single_scan_tmp);
	}

	//1st: draw the contour on the visualization image, label the contour;
	//class types:
	//		0-background;
	//		1-vehicle;
	//		2-motorbike;
	//		3-pedestrian (not labelled yet);

	std::vector<int> contour_labels(contour_vector_.size());

	stringstream  image_visual_string;
	image_visual_string<<input_path_.c_str()<<"/image_"<< batch_T_.back().laser_scan.header.seq<<".jpg";
	Mat contour_visual_img = imread(image_visual_string.str().c_str(), CV_LOAD_IMAGE_COLOR);
	imshow("contour_visual_img",contour_visual_img);
	waitKey(100);

	if(!lazy_labelling && !miss_clustering)
	{
		for(size_t i=0; i<contour_vector_.size(); i++)
		{
			//contour_visual_img = Scalar(0);
			drawContours( contour_visual_img, contour_vector_, i, Scalar(255, 255, 255), -1, 8, vector<Vec4i>(), 0, Point() );
			imshow("contour_visual_img",contour_visual_img);
			waitKey(100);

			int class_type = 0;
			printf("please key in the object type of this contour\n");
			scanf("%d", &class_type);
			contour_labels[i] = class_type;
		}
	}
	else if(lazy_labelling)
	{
		for(size_t i=0; i<contour_vector_.size(); i++)
		{
			contour_labels[i] = 0;
		}
	}
	else if(!lazy_labelling && miss_clustering)
	{
		//special consideration: due to clustering problem, some vehicle lidars are not properly clustered together, leading to no contour to label;
		//in this case, label the whole batch of scans as background are inappropriate, and hence label it as under-determined, while will not be used as training data;
		//		4-under-determined;
		ROS_WARN("miss clustering is true, will label the batch as 4");
		for(size_t i=0; i<contour_vector_.size(); i++)
		{
			contour_labels[i] = 4;
		}
	}


	//2nd: label the LIDAR scans according to its associated contours;
	//Here need to PAY SPECIAL ATTENTION to those missing range data:
	//assuming the LIDAR points in one cluster are continuous, simply fill the missing readings with same class type as beginning and ending serials of a cluster;
	for(size_t i=0; i<training_data_vector_.size(); i++)
	{
		DATMO_labelledScan &labelledScan_tmp=batch_T_[i];

		for(size_t j=0; j<training_data_vector_[i].movingObjectClusters.size(); j++)
		{
			std::pair<std::vector<int>, int> &pair_tmp = training_data_vector_[i].movingObjectClusters[j];
			assert(pair_tmp.second < (int)contour_labels.size());

			if(contour_labels[pair_tmp.second]!=0)
			{
				int smallest_serial = labelledScan_tmp.laser_scan.ranges.size();
				int biggest_serial = 0;
				for(int k=0; k<(int)pair_tmp.first.size(); k++)
				{
					if(pair_tmp.first[k]<smallest_serial)smallest_serial=pair_tmp.first[k];
					if(pair_tmp.first[k]>biggest_serial)biggest_serial=pair_tmp.first[k];
				}
				for(int k=smallest_serial; k<=biggest_serial; k++)
				{
					labelledScan_tmp.type_mask[k]=contour_labels[pair_tmp.second];
				}
			}
		}
	}

	//3rd: merge the labelling from the 2nd-half of "batch_Tminus1_" and the 1st-half of "batch_T_", save results;
	//it should be clarified that the 1st-half in the first batch, and the 2nd-half in the last batch will not be labelled, and hence not stored in the labelled data;

	if(batch_Tminus1_.size()==0)
	{
		batch_Tminus1_ = batch_T_;
		ROS_INFO("initializing batch_Tminus1_");
		return;
	}

	for(size_t i=0; i<(size_t)interval_; i++)
	{
		DATMO_labelledScan &labelledScan_T =batch_T_[i];
		DATMO_labelledScan &labelledScan_Tminus1 =batch_Tminus1_[(i+interval_)];
		for(size_t j=0; j<labelledScan_T.type_mask.size(); j++)
		{
			if(
					((labelledScan_T.type_mask[j]==0)&&(labelledScan_Tminus1.type_mask[j]!=4))
					||(labelledScan_T.type_mask[j]==4)
			  )
			{
				labelledScan_T.type_mask[j] = labelledScan_Tminus1.type_mask[j];
			}
		}
	}

	//if(AbstractLabelling_.type_masks.size()==0)
	//{
	//	AbstractLabelling_.labelled_scan_startSerial = (int)batch_T_[0].laser_scan.header.seq;
	//}
	//AbstractLabelling_.labelled_scan_endSerial = (int)batch_T_[interval_-1].laser_scan.header.seq;
	//for(size_t i=0; i<(size_t)interval_; i++) AbstractLabelling_.type_masks.push_back(batch_T_[i].type_mask);

	save_labelled_batch();
	batch_backup_ = batch_Tminus1_;
	batch_Tminus1_ = batch_T_;
}

void DATMO_labellingData::save_abstract_result()
{
	ROS_INFO("save abstract results");

	for(int i= AbstractLabelling_.labelled_scan_startSerial+interval_-1; i<= AbstractLabelling_.labelled_scan_endSerial; i=i+interval_)
	{
		stringstream  data_batch_string;
		data_batch_string<<output_path_.c_str()<<"/labelled_data"<< i<<".yml";
		FileStorage fs_read(data_batch_string.str().c_str(), FileStorage::READ);
		if(!fs_read.isOpened()){ROS_ERROR("cannot find labelled data file %u", i); return;}

		FileNode masks = fs_read["type_masks"];
		FileNodeIterator mask_it = masks.begin(), mask_it_end = masks.end();
		for(; mask_it!=mask_it_end; mask_it++)
		{
			vector<int> mask_tmp;
			(*mask_it)>>mask_tmp;
			AbstractLabelling_.type_masks.push_back(mask_tmp);
		}
		fs_read.release();
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

//lazy_labelling function is for all "background scan" batches;
void DATMO_labellingData::lazy_labelling_background(int batch_serial, int skipTo_to_batch)
{
	ROS_INFO("do lazy labelling for batches between (but not included) %d and %d", batch_serial, skipTo_to_batch);
	for(int i=batch_serial+interval_; i<skipTo_to_batch; i=i+interval_)
	{
		read_data_batch(i);
		label_data_batch(true);
	}
}

void DATMO_labellingData::update_labelling_history()
{

}

//interactive data labelling: use terminal or GUI interface;
void DATMO_labellingData::main_loop()
{
	ROS_INFO("begin to label data");
	namedWindow( "contour_visual_img", 1 );

	for(int batch_serial = starting_serial_; batch_serial<= end_serial_; )
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
        	if(batch_backup_.size()==0)
        	{
        		batch_Tminus1_.clear();
        	}
        	else
        	{
        		batch_Tminus1_ = batch_backup_;
        	}
        }
        else if(use_the_result == 1)
        {
        	printf("labelling of this batch is accepted.\n");

        	int skipTo_to_batch = 0;
    		printf("****************skip to certain batch? Please enter the number**************\n");
    		scanf("%d", &skipTo_to_batch);

    		if(skipTo_to_batch >= batch_serial+interval_ && skipTo_to_batch%interval_==0)
			{
    			printf("jump to batch %d, will do lazy labelling for scans in between", skipTo_to_batch);
    			lazy_labelling_background(batch_serial, skipTo_to_batch);
				batch_serial=skipTo_to_batch;
			}
    		else
			{
    			printf("illegal skip, will process next batch %d\n", batch_serial+interval_);
    			batch_serial=batch_serial+interval_;
			}
        }
        else
        {
        	if(batch_backup_.size()==0)
        	{
        		batch_Tminus1_.clear();
        	}
        	else
        	{
        		batch_Tminus1_ = batch_backup_;
        	}
        	printf("Only 0, 1, 2 is valid.\n");
        }
	}
	save_abstract_result();
}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "DATMO_labellingData_node");
	DATMO_labellingData DATMO_labellingData_node;
	DATMO_labellingData_node.main_loop();
	return (0);
}

#endif
