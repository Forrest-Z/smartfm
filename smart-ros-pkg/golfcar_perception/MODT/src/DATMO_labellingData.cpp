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
	void read_data_batch(int &batch_serial);
	void label_data_batch(bool lazy_labelling, bool miss_clustering);
	//this save the labelled scan
	void save_labelled_batch();
	void save_abstract_result();
	void lazy_labelling_background(int batch_serial, int skipTo_to_batch);

	ros::NodeHandle private_nh_;
	int starting_serial_, end_serial_, interval_;
	cv::Size visualization_image_size_;
	string param_file_path_;
	string input_path_;
	string output_path_;

	std::vector<DATMO_TrainingScan> training_data_vector_;
	std::vector<vector<Point> >contour_vector_;

	std::vector<DATMO_labelledScan> 	batch_T_;
	std::vector<DATMO_labelledScan> 	batch_Tminus1_;
	std::vector<DATMO_labelledScan> 	batch_backup_;

	DATMO_abstractSummary AbstractLabelling_;
};

DATMO_labellingData::DATMO_labellingData():private_nh_("~")
{
	private_nh_.param("param_file_path", param_file_path_, std::string("/home/baoxing/data/DATMO_Label_Parameters.yaml"));

	load_param_file();

	//AbstractLabelling_.labelled_scan_startSerial = starting_serial_ - interval_+1;
	//AbstractLabelling_.labelled_scan_endSerial = end_serial_ - interval_;
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

//to reconstruct the data structure from the collected data;
// "training_data_vector_" + "contour_vector_";
void DATMO_labellingData::read_data_batch(int &batch_serial)
{
	training_data_vector_.clear();
	contour_vector_.clear();

	stringstream  data_batch_string;
	data_batch_string<<input_path_.c_str()<<"/training_data"<< batch_serial<<".yml";
	cout<<data_batch_string.str().c_str()<<endl;
	FileStorage fs_read(data_batch_string.str().c_str(), FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("cannot find data batch file"); return;}

	FileNode scans = fs_read["scan"];
	FileNodeIterator scan_it = scans.begin(), scan_it_end = scans.end();
	FileNode odoms = fs_read["odom"];
	FileNodeIterator odom_it = odoms.begin(), odom_it_end = odoms.end();
	FileNode traing_data = fs_read["training_data_vector"];
	FileNodeIterator traing_data_it = traing_data.begin(), traing_data_it_end = traing_data.end();
	FileNode contours = fs_read["contours"];
	FileNodeIterator contour_it = contours.begin(), contour_it_end = contours.end();

	training_data_vector_.resize(2*interval_);
	int idx=0;
	for( ; scan_it != scan_it_end; scan_it++, idx++)
	{
		training_data_vector_[idx].laser_scan.header.seq = size_t((int)(*scan_it)["serial"]);
		double time_second_tmp = (double)(*scan_it)["time"];
		training_data_vector_[idx].laser_scan.header.stamp = ros::Time(time_second_tmp);
		training_data_vector_[idx].laser_scan.angle_min = (float)(*scan_it)["angle_min"];
		training_data_vector_[idx].laser_scan.angle_max = (float)(*scan_it)["angle_max"];
		training_data_vector_[idx].laser_scan.angle_increment = (float)(*scan_it)["angle_increment"];
		training_data_vector_[idx].laser_scan.time_increment = (float)(*scan_it)["time_increment"];
		training_data_vector_[idx].laser_scan.scan_time = (float)(*scan_it)["scan_time"];
		training_data_vector_[idx].laser_scan.range_min = (float)(*scan_it)["range_min"];
		training_data_vector_[idx].laser_scan.range_max = (float)(*scan_it)["range_max"];
		(*scan_it)["ranges"]>>training_data_vector_[idx].laser_scan.ranges;
		(*scan_it)["intensities"]>>training_data_vector_[idx].laser_scan.intensities;
	}

	idx=0;
	for( ; odom_it != odom_it_end; odom_it++, idx++)
	{
		training_data_vector_[idx].poseInOdom.pose.position.x = (double)(*odom_it)["x"];
		training_data_vector_[idx].poseInOdom.pose.position.y = (double)(*odom_it)["y"];
		training_data_vector_[idx].poseInOdom.pose.position.z = (double)(*odom_it)["z"];
		double roll_tmp = (double)(*odom_it)["roll"];
		double pitch_tmp = (double)(*odom_it)["pitch"];
		double yaw_tmp = (double)(*odom_it)["yaw"];
		training_data_vector_[idx].poseInOdom.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_tmp, pitch_tmp, yaw_tmp);
	}

	idx=0;
	for( ; traing_data_it != traing_data_it_end; traing_data_it++, idx++)
	{
		FileNodeIterator clusters_it = ((*traing_data_it)["movingObjectClusters"]).begin(), clusters_it_end = ((*traing_data_it)["movingObjectClusters"]).end();
		for( ; clusters_it != clusters_it_end; clusters_it++)
		{
			std::pair<std::vector<int>, int> pair_tmp;
			(*clusters_it)["serial_in_cluster"]>>pair_tmp.first;
			pair_tmp.second = (int)(*clusters_it)["corresponding_contour_serial"];
			training_data_vector_[idx].movingObjectClusters.push_back(pair_tmp);
		}
	}
	//ROS_INFO("idx %d", idx);

	for( ; contour_it != contour_it_end; contour_it++)
	{
		vector<Point> contour_tmp;
		FileNodeIterator point_it = (*contour_it).begin(), point_it_end = (*contour_it).end();
		for(; point_it!=point_it_end;point_it++)
		{
			cv::Point pointtmp((int)(*point_it)["x"], (int)(*point_it)["y"]);
			//cout<<"(x, y):"<<pointtmp.x<<" "<<pointtmp.y<<"\t";
			contour_tmp.push_back(pointtmp);
		}
		contour_vector_.push_back(contour_tmp);
	}

	fs_read.release();
}

void DATMO_labellingData::label_data_batch(bool lazy_labelling, bool miss_clustering)
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
	//		3-pedestrian;

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

	if(AbstractLabelling_.type_masks.size()==0)
	{
		AbstractLabelling_.labelled_scan_startSerial = (int)batch_T_[0].laser_scan.header.seq;
	}
	AbstractLabelling_.labelled_scan_endSerial = (int)batch_T_[interval_-1].laser_scan.header.seq;
	//for(size_t i=0; i<(size_t)interval_; i++) AbstractLabelling_.type_masks.push_back(batch_T_[i].type_mask);


	save_labelled_batch();
	batch_backup_ = batch_Tminus1_;
	batch_Tminus1_ = batch_T_;
}

//save the results of the 1st-half in "batch_T_" after merging the 2nd-half of "batch_Tminus1_" and the 1st-half of "batch_T_";
void DATMO_labellingData::save_labelled_batch()
{
	size_t lidarbatch_serial = batch_T_[interval_-1].laser_scan.header.seq;
	stringstream  data_batch_string;
	data_batch_string<<output_path_.c_str()<<"/labelled_data"<< lidarbatch_serial<<".yml";
	cout<<data_batch_string.str().c_str()<<endl;
	FileStorage fs(data_batch_string.str().c_str(), FileStorage::WRITE);

	fs << "scan" << "[";
	for(size_t i=0; i<(size_t)interval_; i++)
	{
		DATMO_labelledScan &labelledScan_T =batch_T_[i];
		fs<<"{:"<<"serial"<<(int)labelledScan_T.laser_scan.header.seq
		   <<"time"<<labelledScan_T.laser_scan.header.stamp.toSec()
		   <<"angle_min"<<labelledScan_T.laser_scan.angle_min
		   <<"angle_max"<<labelledScan_T.laser_scan.angle_increment
		   <<"time_increment"<<labelledScan_T.laser_scan.time_increment
		   <<"scan_time"<<labelledScan_T.laser_scan.scan_time
		   <<"range_min"<<labelledScan_T.laser_scan.range_min
		   <<"range_max"<<labelledScan_T.laser_scan.range_max
		   <<"ranges"<<"[:";

		for(size_t j=0; j<labelledScan_T.laser_scan.ranges.size(); j++)
		{
			fs << labelledScan_T.laser_scan.ranges[j];
		}
		fs << "]" << "intensities"<<"[:";

		for(size_t j=0; j<labelledScan_T.laser_scan.intensities.size(); j++)
		{
			fs << labelledScan_T.laser_scan.intensities[j];
		}
		fs << "]" << "}";
	}
	fs<<"]";

	fs << "odom" << "[";
	for(size_t i=0; i<(size_t)interval_; i++)
	{
		DATMO_labelledScan &labelledScan_T =batch_T_[i];
		tf::Quaternion q(labelledScan_T.poseInOdom.pose.orientation.x, labelledScan_T.poseInOdom.pose.orientation.y, labelledScan_T.poseInOdom.pose.orientation.z, labelledScan_T.poseInOdom.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		fs<<"{:"<<"x"<<labelledScan_T.poseInOdom.pose.position.x
			    <<"y"<<labelledScan_T.poseInOdom.pose.position.y
			    <<"z"<<labelledScan_T.poseInOdom.pose.position.z
			    <<"roll"<<roll
			    <<"pitch"<<pitch
			    <<"yaw"<<yaw<<"}";
	}
	fs <<"]";

	fs << "type_masks" << "[";
	for(size_t i=0; i<(size_t)interval_; i++)
	{
		DATMO_labelledScan &labelledScan_T =batch_T_[i];
		fs<<"[:";
		for(size_t j=0; j<labelledScan_T.type_mask.size(); j++)
		{
			fs<<labelledScan_T.type_mask[j];
		}
		fs << "]" ;
	}
	fs <<"]";

	fs.release();
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
		label_data_batch(true, false);
	}
}

//interactive data labelling: use terminal or GUI interface;
void DATMO_labellingData::main_loop()
{
	ROS_INFO("begin to label data");
	namedWindow( "contour_visual_img", 1 );

	bool miss_clustering = false;

	for(int batch_serial = starting_serial_; batch_serial<= end_serial_; )
	{
		ROS_INFO("---------------------------------------------------------------");
		ROS_INFO("batch serial %d", batch_serial);

		read_data_batch(batch_serial);
		label_data_batch(false, miss_clustering);
		miss_clustering = false;

        int use_the_result = 0;
        printf("@@@@@@@@@@@@@@@@@@ use your labelled result? Press 0 to redo it, Press 1 to accept, Press 2 to take care of miss_clustering  @@@@@@@@@@@@@@ \n");
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
        else  if(use_the_result == 2)
        {
        	miss_clustering=true;
        	if(batch_backup_.size()==0)
        	{
        		batch_Tminus1_.clear();
        	}
        	else
        	{
        		batch_Tminus1_ = batch_backup_;
        	}
        	printf("!!!!!!!!!!missing_clustering is true, redo the labelling of this batch as missing_clustering!!!!!!!!\n");
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
