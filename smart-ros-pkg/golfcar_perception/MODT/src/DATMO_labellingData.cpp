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
	void label_data_batch();
	//this save the labelled scan
	void save_labelled_batch();
	void save_abstract_result();

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
};

DATMO_labellingData::DATMO_labellingData():private_nh_("~")
{
	private_nh_.param("param_file_path", param_file_path_, std::string("/home/baoxing/data/DATMO_Label_Parameters.yaml"));
	load_param_file();
}

void DATMO_labellingData::load_param_file()
{
	FileStorage fs_read(param_file_path_, FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("ped_semantics cannot find parameter file"); return;}

	starting_serial_ = (int)fs_read["starting_serial"];
	end_serial_ = (int)fs_read["end_serial"];
	interval_ = (int)fs_read["interval_"];
	visualization_image_size_.height = (int)fs_read["image_height"];
	visualization_image_size_.width = (int)fs_read["image_width"];
	fs_read["input_path"]>> input_path_;
	fs_read["output_path"]>> output_path_;

	fs_read.release();
}

//to reconstruct the data structure from the collected data;
// "training_data_vector_" + "contour_vector_";
void DATMO_labellingData::read_data_batch(int &batch_serial)
{
	training_data_vector_.clear();
	contour_vector_.clear();

	stringstream  data_batch_string;
	data_batch_string<<input_path_.c_str()<<"/raw_data/training_data"<< batch_serial<<".yaml";
	FileStorage fs_read(data_batch_string.str().c_str(), FileStorage::READ);

	FileNode scans = fs_read["scan"];
	FileNodeIterator scan_it = scans.begin(), scan_it_end = scans.end();
	for( ; it != it_end; ++it, idx++ )
	{

	}

	FileNode odoms = fs_read["odom"];
	FileNodeIterator odom_it = odoms.begin(), odom_it_end = odoms.end();

	FileNode traing_data = fs_read["training_data_vector"];
	FileNodeIterator traing_data_it = traing_data.begin(), traing_data_it_end = traing_data.end();

	FileNode contours = fs_read["contours"];
	FileNodeIterator contour_it = contours.begin(), contour_it_end = contours.end();

	fs_read.release();
}

void DATMO_labellingData::label_data_batch()
{

}

void DATMO_labellingData::save_labelled_batch()
{

}

void DATMO_labellingData::save_abstract_result()
{

}

//interactive data labelling: use terminal or GUI interface;
void DATMO_labellingData::main_loop()
{
	ROS_INFO("begin to label data");

	for(int batch_serial = starting_serial_; batch_serial<= end_serial_; )
	{
		ROS_INFO("---------------------------------------------------------------");
		ROS_INFO("batch serial %d", batch_serial);

		read_data_batch(batch_serial);
		label_data_batch();

        int use_the_result = 0;
        printf("use your labelled result? Press 1 to use, 0 to redo it \n");
        scanf("%d", &use_the_result);

        if(use_the_result == 0){printf("redo the labelling of this batch\n");}
        else
        {
        	int skip_batch_num = 0;
    		printf("skip some batches? Please enter the number\n");
    		scanf("%d", &skip_batch_num);
    		if(skip_batch_num!=0)batch_serial=batch_serial+skip_batch_num;
    		else batch_serial=batch_serial+interval_;
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
