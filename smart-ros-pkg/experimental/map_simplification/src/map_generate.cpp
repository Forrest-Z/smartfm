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
	string folder("/home/demian/smartfm/smart-ros-pkg/experimental/map_simplification/maps");
	if(argc!=2)
	{
		cout<<"Usage: map_generate input.txt"<<endl;
		return 1;
	}
	file.open(argcv[1]);

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

	string bin_str;
	int file_no=0;
	while(file.good())
	{
		getline(file, bin_str);
		if(bin_str.size()>0)
		{
			int bit_no = 0;
			Mat new_map;
			new_map = background_img.clone();
			for(string::iterator it=bin_str.begin(); it!=bin_str.end(); it++)
			{
				char on_off =*it;
				if(on_off == '1')
				{
					cout<<"O";
					multiply(new_map, maps[bit_no], new_map);
				}
				else cout<<" ";
				bit_no++;
			}
			cout<<endl;
			multiply(new_map, 255, new_map);
			stringstream filename;
			filename<<folder<<"/comb/"<<file_no++<<".png";
			cout<<"Saving: "<<filename.str()<<endl;
			imwrite(filename.str(), new_map);
		}
	}

	return 0;
}
