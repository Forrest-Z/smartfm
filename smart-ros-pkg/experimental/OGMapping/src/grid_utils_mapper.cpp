#include "grid_utils_mapper.h"

namespace golfcar_perception{

	grid_utils_mapper::grid_utils_mapper():private_nh_("~")
	{

	}

	void grid_utils_mapper::scanCallback(const sm::LaserScan& scan)
	{

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_utils_mapper");
	ros::NodeHandle n;
	golfcar_perception::grid_utils_mapper grid_utils_mapper_node;
	ros::spin();
	return 0;
}
