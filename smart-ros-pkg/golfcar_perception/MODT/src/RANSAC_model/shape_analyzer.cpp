#include "shape_analyzer.h"

namespace mrpt{
	
	shape_analyzer::shape_analyzer()
	{
		printf("to analyze the shape");
	}
		
	void shape_analyzer::RANSAC_shape(sensor_msgs::PointCloud &input_cloud)
	{
		mrpt::dynamicsize_vector<double> xs, ys;
		for(size_t k=0; k<input_cloud.points.size(); k++)
		{
			xs.push_back( input_cloud.points[k].x);
			ys.push_back( input_cloud.points[k].y);
		}
		std::vector<std::pair<mrpt::vector_size_t, Lshape> >  Lshape_models;
		ransac_detect_Lshape(xs, ys, Lshape_models, 0.3, 5);
	}
};
