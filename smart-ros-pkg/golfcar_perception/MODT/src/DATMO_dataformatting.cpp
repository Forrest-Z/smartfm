#include <cstdio>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>

using namespace cv;
int main(int argc, char** argv) 
{
	std::string parameter_file = "./data/parameter_file.yaml";
	FileStorage fs_read(parameter_file.c_str(), FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("cannot find data batch file"); return 0;}

	std::string input_file, output_file;
	int training_sample_number, batch_number, feature_number_PerBatch;

	fs_read["input_file"]>> input_file;
	fs_read["output_file"]>> output_file;
	training_sample_number = (int)fs_read["training_sample_number"];
	batch_number = (int)fs_read["batch_number"];
	feature_number_PerBatch = (int)fs_read["feature_number_PerBatch"];
	fs_read.release();

	int total_feature_number = batch_number*feature_number_PerBatch;

    FILE *fp_input, *fp_output;
    if((fp_input=fopen(input_file.c_str(), "r"))==NULL){printf("cannot open input_file\n");return 0;}
    if((fp_output=fopen(output_file.c_str(), "w"))==NULL){printf("cannot open output_file\n");return 0;}
    
    for(int i=1; i<=training_sample_number; i++)
    {
    	int class_type;
    	double features[total_feature_number];

        fscanf(fp_input,  "%d\t", &class_type);
        for(int j=0; j<total_feature_number; j++) fscanf(fp_input, "%lf\t", &features[j]);
        fscanf(fp_input, "\n");
        
        fprintf(fp_output,  "%d\t", class_type);
        for(int j=0; j<total_feature_number; j++) fprintf(fp_output, "%d:%lf\t", (j+1), features[j]);
        fprintf(fp_output, "\n");
    }
    
    fclose(fp_input);
    fclose(fp_output);
    
    return 0;
}
