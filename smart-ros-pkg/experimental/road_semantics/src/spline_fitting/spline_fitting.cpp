# include "spline_fitting.h"

namespace golfcar_semantics{

	road_semantics::road_semantics(string parameter_file)
	{

	}

	void spline_fitting::parameter_init()
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

	void spline_fitting::network_semantics()
	{
		topo_semantic_analyzer_->analyze_semantic();
	}

	spline_fitting::~spline_fitting()
	{

	}
};

