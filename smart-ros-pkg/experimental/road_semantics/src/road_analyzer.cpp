#include <iostream>
#include <float.h>
#include <math.h>
#include "topo_extractor/datatypes.hh"
#include "topo_extractor/utils.hh"
#include "topo_extractor/TopoExtractor.h"
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

char* imagefile=NULL;
char* outfile=NULL;
float image_resolution;
unsigned int unknown_min=127;
unsigned int unknown_max=128;
bool pruning = true;
float distance_min=0.0;
float distance_max=FLT_MAX;

void parse_params(int argc, char *argv[]) {
  if (argc < 2) {
    cout << "Please run with -? for runtime options.\n";
    exit (0);
  }
  
  for (int i=1;i<argc;i++) {
    if (0==strcmp(argv[i],"-?")) {
      cout << "\nTo run : ./bin/road_analyzer -image-file FILENAME1 <options>\n\n";
      cout << "options (see README for details):\n";
      cout << "-output-file FILENAME2\n";
      cout << "-image-resolution m/pixel \n";
      cout << "-min-unknown N\n";
      cout << "-max-unknown M\n";
      cout << "-pruning [0|1]\n";
      cout << "-min-distance R\n";
      cout << "-max-distance S\n";
      exit (0);
    }
    else if (i+1 < argc)
    {
	  if (0==strcmp(argv[i],"-image-file"))
		  imagefile=argv[++i];
	  else if (0==strcmp(argv[i],"-output-file"))
		  outfile=argv[++i];
	  else if (0==strcmp(argv[i],"-image-resolution"))
		  image_resolution=atoi(argv[++i]);
	  else if (0==strcmp(argv[i],"-min-unknown"))
		  unknown_min=atoi(argv[++i]);
	  else if (0==strcmp(argv[i],"-max-unknown"))
		  unknown_max=atoi(argv[++i]);
	  else if (0==strcmp(argv[i],"-pruning"))
		  pruning=atoi(argv[++i]);
	  else if (0==strcmp(argv[i],"-min-distance"))
		  distance_min=fmaxf(0,atof(argv[++i]));
	  else if (0==strcmp(argv[i],"-max-distance"))
		  distance_max=atof(argv[++i]);
	  else {
			cout << "Incorrect parameter list.\n";
			cout << "Please run with -? for runtime options.\n";
			exit(0);
	  	   }
	}
	else {
	  cout << "Incorrect parameter lists.\n";
	  cout << "Please run with -? for runtime options.\n";
	  exit(0);
	}
  }
}

//further write it as a class to handle the image loading and processing;

grid_type image_process(const char* imagefile_para, float image_resolution_para, unsigned int unknown_min_para, unsigned int unknown_max_para)
{
	IplImage *img = 0;
	grid_type src_grid;

	if((img = cvLoadImage( imagefile_para, CV_LOAD_IMAGE_GRAYSCALE)) == 0)
	{
		cout<<"unable to load image"<<endl;
		return src_grid;
	}

	int img_height 		= img -> height;
	int img_width  		= img -> width;
	int img_step	 	= img -> widthStep/sizeof(uchar);
	uchar * img_data 	= (uchar*)img ->imageData;
	column_type tmp(img_height);
	src_grid.resize(img_width,tmp);

	for(int ih=0; ih < img_height; ih++)
	{
		for(int iw=0; iw < img_width; iw++)
		{
			if(img_data[ih*img_step+iw]>unknown_max_para)
			{
				//free means bright color;
				src_grid[iw][ih]=Free;
			}
			else
			{
				src_grid[iw][ih]=Occupied;
			}
		}
	}

	return src_grid;
}


int main(int argc, char *argv[]) {
  parse_params(argc,argv);

  if (imagefile==NULL) {
    cout << "No image given.\n";
    cout << "Please run with -? for runtime options.\n";
    exit (0);
  }

  if (unknown_min > unknown_max) {
    cout << "min_unknown value must be <= max_unknown value\n";
    cout << "Please run with -? for runtime options.\n";
    exit (0);
  }

  if (distance_min > distance_max) {
    cout << "min_distance value must be <= max_distance value\n";
    cout << "Please run with -? for runtime options.\n";
    exit (0);
  }

  if (unknown_min < 1 || unknown_max > 254) {
    cout << "unknown values can be between 1 and 254.\n";
    cout << "Please run with -? for runtime options.\n";
    exit (0);
  }

  //write the loading function use opencv;
  grid_type curr_grid=image_process(imagefile, image_resolution, unknown_min, unknown_max);

  if (curr_grid.empty() || curr_grid[0].empty()) {
    cout << "Read grid has no dimensions\n";
    exit(-1);
  }

  if (outfile==NULL) {
    int delim=strcspn(imagefile,".");
    outfile=new char[delim+14];
    strncpy(outfile,imagefile,delim);
    outfile[delim]='\0';
    strcat(outfile,"_skeleton.ppm");
  }

  cout << endl<< "Input file: " << imagefile << endl;
  cout << "Output file: " << outfile << endl;
  cout << "Image Resolution: " << image_resolution << endl;
  cout << "OCCUPIED grayscale range: 0-" << unknown_min-1 << endl;
  cout << "UNKNOWN grayscale range: " << unknown_min << "-" << unknown_max << endl;
  cout << "FREESPACE grayscale range: " << unknown_max+1 << "-255" << endl;
  cout << "Pruning: ";
  if (pruning)
    cout << "On\n";
  else cout << "Off\n";
  cout << "Minimum distance: " << distance_min << endl;
  cout << "Maximum distance: " << distance_max << endl;

  topo_extractor topo(curr_grid,distance_min,distance_max,pruning);

  Time t;
  topo.extract_topology();
  cout << "Skeleton created in "<<t.get_since()<<" seconds\n\n";

  return 1;
}
