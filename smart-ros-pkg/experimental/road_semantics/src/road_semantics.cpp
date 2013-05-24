# include "road_semantics.h"

namespace golfcar_semantics{

	road_semantics::road_semantics(string parameter_file)
	{
		parameter_file_ = parameter_file;
		parameter_init();

		grid_type curr_grid = 	road_semantics::image_loading(image_file_path_, image_scale_, unknown_min_, unknown_max_);
		topology_extractor_ 	= new topo_extractor(curr_grid, distance_min_,distance_max_, pruning_);
		Time t;
		topology_extractor_->extract_topology();
		cout << "Skeleton created in "<<t.get_since()<<" seconds\n\n";

		topo_semantic_analyzer_ = new topo_semantic(*topology_extractor_);
	}

	void road_semantics::parameter_init()
	{
		ROS_INFO("parameter initialization for road_semantics");
		FileStorage fs_read(parameter_file_, FileStorage::READ);
		if(!fs_read.isOpened())ROS_ERROR("road_semantics cannot find parameter file");
		fs_read["image_path"] >> image_file_path_;
		image_scale_ = (double) fs_read["image_scale"];
		unknown_min_ = size_t((int) fs_read["unknown_min"]);
		unknown_max_ = size_t((int) fs_read["unknown_max"]);

		distance_min_ = 0;
		distance_max_ = DBL_MAX;
		pruning_	  = true;
	}

	grid_type road_semantics::image_loading(string imagefile, double image_scale, unsigned int unknown_min, unsigned int unknown_max)
	{
		IplImage *img = 0;
		grid_type src_grid;

		if((img = cvLoadImage( imagefile.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == 0)
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
				if(img_data[ih*img_step+iw]>unknown_max)
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

	void road_semantics::network_semantics()
	{
		topo_semantic_analyzer_->analyze_semantic();
	}

	road_semantics::~road_semantics()
	{
		delete topology_extractor_;
		delete topo_semantic_analyzer_;
	}
};

