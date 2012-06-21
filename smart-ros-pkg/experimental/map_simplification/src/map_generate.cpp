/*
 * map_generate.cpp
 *
 *  Created on: Jun 15, 2012
 *      Author: demian
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argcv)
{
	ifstream file;
	//string folder("/home/golfcar/smartfm/smart-ros-pkg/experimental/map_simplification/maps");
	if(argc!=3)
	{
		cout<<"Usage: map_generate MAP_FOLDER input.txt"<<endl;
		return 1;
	}
	string folder(argcv[1]);
	file.open(argcv[2]);

	//load image into memory
	Vector< Mat > maps;
	stringstream background;
	background<<folder<<"/"<<"background.png";
	Mat background_img = imread(background.str());
	cvtColor(background_img, background_img, CV_RGB2GRAY);
	//convert to float 0-1 is necessary for simple multiply operation on Mat
	background_img.convertTo(background_img, CV_32F, 1./255);
	cout<<"Background_img type "<<background_img.type()<<endl;
	for(int i=0; i< 19; i++)
	{
		stringstream filename;
		filename<<folder<<"/"<<i+1<<".png";
		Mat map = imread(filename.str(), 0);

		if(map.cols == 0) cout<<"Load image error on "<<filename.str()<<endl;
		cout<<"map "<<i<<" type "<<map.type()<<endl;
		map.convertTo(map, CV_32F, 1./255);
		maps.push_back(map);
	}

	vector<string> bin_strs;
	while(file.good())
	{
	        string bin_str;
		getline(file, bin_str);
		if(bin_str.size()>0)
		    bin_strs.push_back(bin_str);
	}
        #pragma omp parallel for
	for(size_t i = 0; i < 8795/*bin_strs.size()*/; i++)
	{

	    {
	        string bin_str = bin_strs[i];

	        Mat new_map;
	        new_map = background_img.clone();
	        for(size_t j = 0; j<bin_str.size(); j++)
	        {
	            char on_off =bin_str[j];
	            if(on_off == '1')
	                multiply(new_map, maps[j], new_map);
	        }
	        multiply(new_map, 255, new_map);
	        stringstream filename;
                #pragma omp critical
	        {
	            filename<<folder<<"/comb/"<<i<<".png";
	            cout<<"Saving: "<<filename.str()<<endl;
	            imwrite(filename.str(), new_map);
	        }
	    }
	}
	return 0;
}
