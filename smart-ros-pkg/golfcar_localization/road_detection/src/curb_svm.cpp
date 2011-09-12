#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <svm.h>
#include <road_detection/curbPointCloudID.h>
#include <tf/transform_listener.h>
//for svm file output
#include <iostream>
#include <fstream>
#include <ctime>
#include <cerrno>

#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

using namespace std;

namespace curb_svm{

	class curb_svm{

	public:
		curb_svm();
		~curb_svm();
		void restore_scalefile(string filename,double* &feature_min, double* &feature_max, int &feature_index);
		void rawCurbCallback (const sensor_msgs::PointCloud::ConstPtr& pc);
		int classify(sensor_msgs::PointCloud pc);
		void leftCurbIDCallback(const road_detection::curbPointCloudID::ConstPtr& cpcid);
		void rightCurbIDCallback(const road_detection::curbPointCloudID::ConstPtr& cpcid);
		int svmLeftCurbFeatures(sensor_msgs::PointCloud &laser_cloud, int PID_begin, int PID_end);
		int svmRightCurbFeatures(sensor_msgs::PointCloud &laser_cloud, int PID_begin, int PID_end);
		void publishClassifiedCurb(sensor_msgs::PointCloud &pc, int predicted_class);
		//void rightFeatureCallback(sensor_msgs::PointCloud pc);
		ros::NodeHandle private_nh_;
		vector<ros::Publisher> svm_curb_pubs_;
		ros::Publisher svm_leftFeature_pub_;
		ros::Publisher svm_rightFeature_pub_;
		ros::Publisher svm_leftclassified_pub_;
		ros::Publisher svm_rightclassified_pub_;
		ros::Subscriber svm_left_sub_, svm_right_sub_;
		string svm_scale_file_;
		void output_target(double value);
		float output(int index, float value);
		char* readline(FILE *input);

		double* feature_max_;
		double* feature_min_;
		int feature_index_;
		double lower_, upper_;
		bool write_file_;
		struct svm_model *svm_model_;
		tf::TransformListener sick_to_baselink_;
     	ofstream myfileleftx_;
     	ofstream myfilerightx_;
	};

};

namespace curb_svm
{
	curb_svm::curb_svm():private_nh_("~")
	{
		ros::NodeHandle nh;
		string svm_model_file;


		private_nh_.param("svm_scale_file",svm_scale_file_,string());
		private_nh_.param("svm_model_file",svm_model_file,string());
		private_nh_.param("svm_write_file", write_file_, false);

		if(write_file_)
		{
			cout<<"Write file start!"<<endl;
			time_t t1 = std::time(0);
			if (t1 == time_t(-1))
			{
				cerr<<"Bad time\n";
				exit(1);
			}

			stringstream ss;
			ss<<t1<<"_leftx.t";
			myfileleftx_.open(ss.str().c_str());
			ss.str(std::string());
			ss<<t1<<"_rightx.t";
			myfilerightx_.open(ss.str().c_str());

		}
		else
		{
			restore_scalefile(svm_scale_file_, feature_min_, feature_max_, feature_index_);
			svm_model_ = svm_load_model(svm_model_file.c_str());
			int nr_class= svm_model_->nr_class;
			cout<<"Number of classes detected:"<<nr_class<<endl;

			//setup ROS messages for final classified curbs
			if(nr_class>2)
			{
				for(int i=0;i<nr_class;i++)
				{
					ros::Publisher pub;
					stringstream ss;
					cout<<svm_model_->label[i]<<endl;
					if(svm_model_->label[i]==-1)ss<<"svm_curb_neg";
					else ss<<"svm_curb_"<<svm_model_->label[i];
					pub = nh.advertise<sensor_msgs::PointCloud>(ss.str().c_str(),1);
					svm_curb_pubs_.push_back(pub);
				}
			}
			//handle the case where only 2 classes is being categorized or using one class svm
			else
			{
				ros::Publisher pub1,pub2;
				pub1 = nh.advertise<sensor_msgs::PointCloud>("svm_curb_neg",1);
				svm_curb_pubs_.push_back(pub1);
				pub2 = nh.advertise<sensor_msgs::PointCloud>("svm_curb_1",1);
				svm_curb_pubs_.push_back(pub2);
			}

		}
		svm_right_sub_ = nh.subscribe("svm_rightCurbPoint_id", 1000, &curb_svm::rightCurbIDCallback, this);
		svm_left_sub_ = nh.subscribe("svm_leftCurbPoint_id", 1000, &curb_svm::leftCurbIDCallback, this);
		svm_leftFeature_pub_ = nh.advertise<sensor_msgs::PointCloud>("svm_left_feature",2);
		svm_rightFeature_pub_ = nh.advertise<sensor_msgs::PointCloud>("svm_right_feature",2);
		svm_leftclassified_pub_=nh.advertise<sensor_msgs::PointCloud>("svm_leftcurbline",2);
		svm_rightclassified_pub_=nh.advertise<sensor_msgs::PointCloud>("svm_rightcurbline",2);

	}
	void curb_svm::leftCurbIDCallback(const road_detection::curbPointCloudID::ConstPtr &cpcID)
	{
		sensor_msgs::PointCloud curb_segment;
		curb_segment.header = cpcID->pc.header;
		if(cpcID->pc.points.size()>0)
		{

			curb_segment.points.push_back(cpcID->pc.points[(cpcID->id_start+cpcID->id_end)/2]);
			sensor_msgs::PointCloud pc = cpcID->pc;
			int id_start = cpcID->id_start;
			int id_end = cpcID->id_end;
			int predicted_feature= svmLeftCurbFeatures(pc, id_start, id_end);

			if(!write_file_) publishClassifiedCurb(curb_segment,predicted_feature);
			//signal for the noisy input to the temporal filter
			if(predicted_feature!=1)
			{
				curb_segment.points.clear();
				for(int i=cpcID->id_start;i<cpcID->id_end;i++)
				{
					curb_segment.points.push_back(cpcID->pc.points[i]);
				}
				curb_segment.points.front().z=0;
				curb_segment.points.back().z=1;
			}
		}

		svm_leftclassified_pub_.publish(curb_segment);

	}
	void curb_svm::rightCurbIDCallback(const road_detection::curbPointCloudID::ConstPtr &cpcID)
	{
		sensor_msgs::PointCloud curb_segment;
		curb_segment.header = cpcID->pc.header;
		if(cpcID->pc.points.size()>0)
		{
			curb_segment.header = cpcID->pc.header;
			curb_segment.points.push_back(cpcID->pc.points[(cpcID->id_start+cpcID->id_end)/2]);
			sensor_msgs::PointCloud pc = cpcID->pc;
			int id_start = cpcID->id_start;
			int id_end = cpcID->id_end;
			int predicted_feature = svmRightCurbFeatures(pc, id_start, id_end);
			//cout<<"right"<<predicted_feature<<endl;
			if(!write_file_) publishClassifiedCurb(curb_segment,predicted_feature);

			if(predicted_feature!=2)
			{
				curb_segment.points.clear();
				for(int i=cpcID->id_start;i<cpcID->id_end;i++)
				{
					curb_segment.points.push_back(cpcID->pc.points[i]);
				}
				curb_segment.points.front().z=0;
				curb_segment.points.back().z=1;
			}
		}

		svm_rightclassified_pub_.publish(curb_segment);
	}

	void curb_svm::publishClassifiedCurb(sensor_msgs::PointCloud &pc, int predicted_class)
	{
		//search through the list of labels exhaustively, not efficient but it get the job done
		int i=0;
		if(svm_model_->nr_class>2)
		{
			for(i; i<svm_model_->nr_class;i++)
			{
				if(svm_model_->label[i]==predicted_class) {
					svm_curb_pubs_[i].publish(pc);
					break;
				}
			}
		}
		else
		{
			if(predicted_class == -1) svm_curb_pubs_[0].publish(pc);
			else svm_curb_pubs_[1].publish(pc);
		}

	}
	curb_svm::~curb_svm()
	{
		if(write_file_)
		{
			myfileleftx_.close();
			myfilerightx_.close();
		}
	}
	int curb_svm::classify(sensor_msgs::PointCloud pc)
	{
		//scaling
		//cout<<"scaling"<<endl;

		for(int i=0;i<pc.points.size();i++)
		{
			int idx = (3*i)+1;
			pc.points[i].x = output(idx,pc.points[i].x);
			pc.points[i].y = output(++idx,pc.points[i].y);
			pc.points[i].z = output(++idx,pc.points[i].z);

		}
		//cout<<"svm_start"<<endl;
		//svm
		struct svm_node *x;
		int max_nr_attr = 64;
		x = (struct svm_node *) realloc(x,max_nr_attr*sizeof(struct svm_node));
		//cout<<"after allocate"<<endl;

		//reduce number of features for svm, for 16 features originally, reducing 2 (2*2 left and right) become only 12 features
		int skip_features = 0;
		for(int i=0;i<pc.points.size()-2*skip_features;i++)
		{
			x[(3*i)].index=(3*i)+1+(3*skip_features);
			x[(3*i+1)].index=(3*i)+2+(3*skip_features);
			x[(3*i+2)].index=(3*i)+3+(3*skip_features);
			x[(3*i)].value = pc.points[i+skip_features].x;
			x[(3*i)+1].value = pc.points[i+skip_features].y;
			x[(3*i)+2].value = pc.points[i+skip_features].z;
		}
		x[3*(pc.points.size()-2*skip_features)].index=-1;
		return svm_predict(svm_model_,x);
		//if(svm_predict(svm_model_,x)>0)
		//classified_curb_pub_.publish(pc_ori);
		//cout<<"final predict"<<endl;
		//cout<<svm_predict(svm_model_,x)<<endl;
	}

	void curb_svm::restore_scalefile(string filename, double* &feature_min, double* &feature_max, int &feature_index)
	{
		//double *feature_max;
		//double *feature_min;

		int idx, c;
		FILE *fp_restore;
		const char *restore_filename = filename.c_str();
		double y_max = -DBL_MAX;
		double y_min = DBL_MAX;
		double y_lower,y_upper;
		int max_index=0;

		fp_restore = fopen(restore_filename,"r");
		if(fp_restore==NULL)
		{
			fprintf(stderr,"can't open file %s\n", restore_filename);
			exit(1);
		}
		cout<<"File opened"<<endl;
		c = fgetc(fp_restore);
		if(c == 'y')
		{
			readline(fp_restore);
			readline(fp_restore);
			readline(fp_restore);
		}
		cout<<readline(fp_restore)<<endl;
		cout<<readline(fp_restore)<<endl;
		cout<<"Retrieving maximum index"<<endl;
		while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
			max_index = max(idx,max_index);

		rewind(fp_restore);
		cout<<"Max index retrieved "<<max_index<<endl;
		feature_max = (double *)malloc((max_index+1)* sizeof(double));
		feature_min = (double *)malloc((max_index+1)* sizeof(double));

		double fmin, fmax;
		int y_scaling = 0;
		if((c = fgetc(fp_restore)) == 'y')
		{
			fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper);
			fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max);
			y_scaling = 1;
		}
		else
			ungetc(c, fp_restore);

		if (fgetc(fp_restore) == 'x') {
			cout<<"got x"<<endl;
			fscanf(fp_restore, "%lf %lf\n", &lower_, &upper_);
			while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
			{
				if(idx<=max_index)
				{
					feature_min[idx] = fmin;
					feature_max[idx] = fmax;
					//f_min.push_back(fmin);
					//f_max.push_back(fmax);
				}
			}
		}
		feature_index = max_index;
		fclose(fp_restore);
		cout<<"File closed"<<endl;
	}

	char* curb_svm::readline(FILE *input)
	{
		int len;
		char *line = NULL;
		int max_line_len = 1024;
		line = (char *) malloc(max_line_len*sizeof(char));
		if(fgets(line,max_line_len,input) == NULL)
			return NULL;

		while(strrchr(line,'\n') == NULL)
		{
			max_line_len *= 2;
			line = (char *) realloc(line, max_line_len);
			len = (int) strlen(line);
			if(fgets(line+len,max_line_len-len,input) == NULL)
				break;
		}
		return line;
	}

	float curb_svm::output(int index, float value)
	{
		/* skip single-valued attribute */
		if(feature_max_[index] == feature_min_[index])
			return value;

		if(value == feature_min_[index])
			value = lower_;
		else if(value == feature_max_[index])
			value = upper_;
		else
			value = lower_ + (upper_-lower_) *
				(value-feature_min_[index])/
				(feature_max_[index]-feature_min_[index]);

		if(value != 0)
		{
			//printf("%d:%g ",index, value);
		}
		return value;
	}

	int curb_svm::svmLeftCurbFeatures(sensor_msgs::PointCloud &laser_cloud, int PID_begin, int PID_end)
	{
		ROS_DEBUG("svnLeftCurb start");
		//push data set with an amount of features
		const unsigned int svm_features = 16;
		int svm_curb_center = (PID_begin + PID_end)/2;
		sensor_msgs::PointCloud svm_curb_data;
		svm_curb_data.header = laser_cloud.header;
		svm_curb_data.header.stamp = ros::Time(0);
		int svm_curb_left = svm_curb_center-svm_features/2;
		//take care the case where the middle point of the curb has less than half the number of specified svm features
		ROS_DEBUG("svnLeftCurb 1");
		if(svm_curb_left<0)
		{
			//simply take the point starting from most left
			for(unsigned int i=0; i<svm_features;i++)
			{
				svm_curb_data.points.push_back(laser_cloud.points[i]);
			}
			ROS_DEBUG("svnLeftCurb 2");
		}
		else
		{
			if(svm_features+svm_curb_left>laser_cloud.points.size()-1)
				svm_curb_left = laser_cloud.points.size() - svm_curb_left - 1;
			for(unsigned int i=svm_curb_left;i<svm_features+svm_curb_left;i++ )
			{
				svm_curb_data.points.push_back(laser_cloud.points[i]);
			}
			ROS_DEBUG("svnLeftCurb 3");
		}
		sick_to_baselink_.transformPointCloud("base_link", svm_curb_data, svm_curb_data);
		ROS_DEBUG("svnLeftCurb 4");
		//normalised everything to start from zero to +ve so that it is invariant to x and y axis
		double y_point = svm_curb_data.points[0].y;
		double x_point = svm_curb_data.points[0].x;
		if(write_file_)	myfileleftx_<<"1 ";
		//myfilelefty_<<"1 ";
		//myfileleftz_<<"1 ";
		for(int i=0; i<svm_curb_data.points.size();i++)
		{
			//assumed that the left curb will always to the left of the vehicle's x-axis
			svm_curb_data.points[i].y-=y_point;
			svm_curb_data.points[i].x-=x_point;

			if(write_file_)
			myfileleftx_<<(3*i)+1<<":"<<svm_curb_data.points[i].x<<' '
					<<(3*i)+2<<":"<<svm_curb_data.points[i].y<<' '
					<<(3*i)+3<<":"<<svm_curb_data.points[i].z<<' ';

		}
		ROS_DEBUG("svnLeftCurb 5");
		svm_leftFeature_pub_.publish(svm_curb_data);
		ROS_DEBUG("svnLeftCurb end");
		if(write_file_)
		{
			myfileleftx_<<endl;
			return -1;
		}

		return classify(svm_curb_data);

	}

	int curb_svm::svmRightCurbFeatures(sensor_msgs::PointCloud &laser_cloud, int PID_begin, int PID_end)
	{
		ROS_DEBUG("svnRightCurb start");
		//push data set with an amount of features
		const unsigned int svm_features = 16;
		int svm_curb_center = (PID_begin + PID_end)/2;

		sensor_msgs::PointCloud svm_curb_data;
		svm_curb_data.header = laser_cloud.header;
		svm_curb_data.header.stamp = ros::Time(0);
		//right curb feature
		int svm_curb_right = svm_curb_center+svm_features/2;
		//take care the case where the middle point of the curb has less than half the number of specified svm features

		if(svm_curb_right>=laser_cloud.points.size())
		{
			ROS_DEBUG("svm_curb_right>=laser_cloud.points.size()");
			//simply take the point ending with right most point
			for(unsigned int i=laser_cloud.points.size()-1-svm_features; i<laser_cloud.points.size()-1;i++)
			{
				svm_curb_data.points.push_back(laser_cloud.points[i]);
			}
		}
		else
		{


			//there are possibilities that right curb given is too much towards the left scan
			svm_curb_right=svm_curb_right-svm_features;
			if(svm_curb_right<0) svm_curb_right = 0;
			ROS_DEBUG("else.. %d",svm_curb_right);
			for(unsigned int i=svm_curb_right;i<svm_curb_right+svm_features;i++ )
			{
				svm_curb_data.points.push_back(laser_cloud.points[i]);
			}
		}
		sick_to_baselink_.transformPointCloud("base_link", svm_curb_data, svm_curb_data);
		//normalised everything to start from zero to -ve so that it is invariant to x and y axis

		double y_point = svm_curb_data.points[svm_curb_data.points.size()-1].y;
		double x_point = svm_curb_data.points[svm_curb_data.points.size()-1].x;

		if(write_file_)myfilerightx_<<"1 ";
		//myfilerighty_<<"1 ";
		//myfilerightz_<<"1 ";
		for(int i=0; i<svm_curb_data.points.size();i++)
		{
			//assumed that the right curb will always to the right of the vehicle's x-axis
			svm_curb_data.points[i].y-=y_point;
			svm_curb_data.points[i].x-=x_point;

			if(write_file_)
			myfilerightx_<<(3*i)+1<<":"<<svm_curb_data.points[i].x<<' '
					<<(3*i)+2<<":"<<svm_curb_data.points[i].y<<' '
					<<(3*i)+3<<":"<<svm_curb_data.points[i].z<<' ';

		}
		ROS_DEBUG("svnRightCurb end");
		svm_rightFeature_pub_.publish(svm_curb_data);
		if(write_file_)
		{
			myfilerightx_<<endl;
			return -1;
		}

		return classify(svm_curb_data);

	}
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "curb_svm");

	 curb_svm::curb_svm* svm_node;
	 svm_node = new curb_svm::curb_svm();

	 ros::spin();
}
