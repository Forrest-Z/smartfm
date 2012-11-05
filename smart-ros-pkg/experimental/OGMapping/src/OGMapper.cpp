#include "OGMapper.h"

namespace golfcar_perception{

OGMapper::OGMapper():
		private_nh_("~")
{
	map_saved_ = false;
	private_nh_.param("init_map", init_map_, true);
	private_nh_.param("map_size_x", map_size_x_, 4251); //8502
	private_nh_.param("map_size_y", map_size_y_, 4520); //9040
	private_nh_.param("map_file", map_file_, std::string("tra_map"));
	OGMapper::map_load(map_file_);

	map_frame_ = "map";
	tf_ = new tf::TransformListener();

	private_nh_.param("initial_occ", initial_occ_, 0.5);
	private_nh_.param("occ_over_occ", occ_over_occ_, 0.7);
	private_nh_.param("occ_over_free", occ_over_free_, 0.05);

	boundary_sub_.subscribe(nh_, "road_boundary_pts", 10);
	boundary_filter_ = new tf::MessageFilter<PointCloud>(boundary_sub_, *tf_, map_frame_, 10);
	boundary_filter_ ->registerCallback(boost::bind(& OGMapper::boundaryCallback, this, _1));
	boundary_filter_->setTolerance(ros::Duration(0.05));

	surface_sub_.subscribe(nh_, "road_surface_pts", 10);
	surface_filter_ = new tf::MessageFilter<PointCloud>(surface_sub_, *tf_, map_frame_, 10);
	surface_filter_ ->registerCallback(boost::bind(& OGMapper::surfaceCallback, this, _1));
	surface_filter_->setTolerance(ros::Duration(0.05));

	//to be continued...
	//mapSave_Callback()
	save_sub_ = nh_.subscribe("/save_Tmap", 1, &OGMapper::mapSave_Callback, this);
}

void OGMapper::boundaryCallback(const PointCloud::ConstPtr& boundary_in)
{
	boost::recursive_mutex::scoped_lock bcl(configuration_mutex_);
	PointCloud boundary_map_frame;
	pcl_ros::transformPointCloud(map_frame_, *boundary_in, boundary_map_frame, *tf_ );
	std::string boundary_type = "boundary";
	OGMapper::map_update(boundary_map_frame, boundary_type);
}

void OGMapper::surfaceCallback(const PointCloud::ConstPtr& surface_in)
{
	boost::recursive_mutex::scoped_lock scl(configuration_mutex_);
	PointCloud surface_map_frame;
	pcl_ros::transformPointCloud(map_frame_, *surface_in, surface_map_frame, *tf_ );
	std::string surface_type = "surface";
	OGMapper::map_update(surface_map_frame, surface_type);
}

void OGMapper::map_update(PointCloud& input_pcl, std::string& input_type)
{
	boost::recursive_mutex::scoped_lock mul(configuration_mutex_);

	//Occupancy Grid Mapping (OGM) - Probabilistic Robotics P284;
	// p(occ | occ) = 0.7; p(occ | free) = 0.1;
	double occ_conditioned_on_meas;
	if(input_type == "surface") occ_conditioned_on_meas = occ_over_free_;
	else occ_conditioned_on_meas = occ_over_occ_;
	double log_evidence = log10(occ_conditioned_on_meas/(1-occ_conditioned_on_meas));
	double log_original = log10(initial_occ_/(1-initial_occ_));
	for(size_t i=0; i<input_pcl.points.size(); i++)
	{
		unsigned int x_dim = MAP_GXWX(map_, input_pcl.points[i].x);
		unsigned int y_dim = MAP_GXWX(map_, input_pcl.points[i].y);
		if(!MAP_VALID(map_,x_dim, y_dim))continue;
		unsigned int cell_index = MAP_INDEX(map_, x_dim, y_dim);
		//binary bayes filter;
		double last_prob = map_->cells[cell_index].occupy_probility;
		double l_t_minus_one = log10(last_prob/(1-last_prob));
		double l_t = log_evidence - log_original + l_t_minus_one;

		map_->cells[cell_index].occupy_probility = 1.0 -1.0 /(1.0 +exp(l_t));
		map_->cells[cell_index].update_times ++;
	}
}

void OGMapper::map_load(const std::string& fname)
{
	boost::recursive_mutex::scoped_lock ml(configuration_mutex_);
	map_ = map_alloc();
	ROS_ASSERT(map_);

	std::string mapfname = "";
	double origin[3];
	int negate;
	double res;
	double occ_th, free_th;
	std::string frame_id = "map";

	std::string file_path =fname + ".yaml";
	//mapfname = fname + ".pgm";
	//std::ifstream fin((fname + ".yaml").c_str());
	std::ifstream fin(file_path.c_str());
	if (fin.fail()) {
	  ROS_ERROR("Map_server could not open %s.", file_path.c_str());
	  exit(-1);
	}
	//read the parameters from YAML document;
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	try {
	  doc["resolution"] >> res;
	} catch (YAML::InvalidScalar) {
	  ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
	  exit(-1);
	}
	try {
	  doc["negate"] >> negate;
	} catch (YAML::InvalidScalar) {
	  ROS_ERROR("The map does not contain a negate tag or it is invalid.");
	  exit(-1);
	}
	try {
	  doc["occupied_thresh"] >> occ_th;
	} catch (YAML::InvalidScalar) {
	  ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
	  exit(-1);
	}
	try {
	  doc["free_thresh"] >> free_th;
	} catch (YAML::InvalidScalar) {
	  ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
	  exit(-1);
	}
	try {
	  doc["origin"][0] >> origin[0];
	  doc["origin"][1] >> origin[1];
	  doc["origin"][2] >> origin[2];
	} catch (YAML::InvalidScalar) {
	  ROS_ERROR("The map does not contain an origin tag or it is invalid.");
	  exit(-1);
	}
	try {
	  doc["image"] >> mapfname;
	  // TODO: make this path-handling more robust
	  if(mapfname.size() == 0)
	  {
		ROS_ERROR("The image tag cannot be an empty string.");
		exit(-1);
	  }
	  if(mapfname[0] != '/')
	  {
		// dirname can modify what you pass it
		char* fname_copy = strdup(file_path.c_str());
		mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
		free(fname_copy);
	  }
	} catch (YAML::InvalidScalar) {
	  ROS_ERROR("The map does not contain an image tag or it is invalid.");
	  exit(-1);
	}

	ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
	if(!init_map_)OGMapper::loadMapFromFile(map_, mapfname.c_str(), res,negate, occ_th, free_th, origin);
	else{
		ROS_INFO("init known maps");
		OGMapper::unknownMap(map_, res, (unsigned int) map_size_x_, (unsigned int) map_size_y_, origin);
		OGMapper::mapsave();
	}
	ROS_INFO("Yep, map loaded!");
}

void
OGMapper::unknownMap(map_t* map, double res,
                unsigned int size_x, unsigned int size_y, double* origin)
{
	boost::recursive_mutex::scoped_lock uml(configuration_mutex_);

	unsigned int i,j;
	map->size_x = size_x;
	map->size_y = size_y;
	map->scale	= res;
	map->origin_x = *(origin);
	map->origin_y = *(origin+1);
	map->yaw = *(origin+2);
	// Convert to player format
	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
	ROS_ASSERT(map->cells);

	for(j = 0; j < (unsigned int) map->size_y; j++)
	{
		for (i = 0; i < (unsigned int) map->size_x; i++)
		{
		 	map->cells[MAP_INDEX(map, i, map->size_y -j-1)].occupy_probility = 0.50;
		 	map->cells[MAP_INDEX(map, i, map->size_y -j-1)].update_times = 0;
		}
	}
}

void
OGMapper::loadMapFromFile(map_t* map,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin)
{
	boost::recursive_mutex::scoped_lock mlf(configuration_mutex_);
	SDL_Surface* img;

	unsigned char* pixels;
	unsigned char* p;
	int rowstride, n_channels;
	unsigned int i,j;
	int k;
	double occ;
	int color_sum;
	double color_avg;

	// Load the image using SDL.  If we get NULL back, the image load failed.
	if(!(img = IMG_Load(fname)))
	{
		std::string errmsg = std::string("failed to open image file \"") +
				std::string(fname) + std::string("\"");
		throw std::runtime_error(errmsg);
	}

	map->size_x = img->w;
	map->size_y = img->h;
	map->scale	= res;
	map->origin_x = *(origin);
	map->origin_y = *(origin+1);
	map->yaw = *(origin+2);
	// Convert to player format
	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
	ROS_ASSERT(map->cells);

	// Get values that we'll need to iterate through the pixels
	rowstride = img->pitch;
	n_channels = img->format->BytesPerPixel;

	// Copy pixel data into the map structure
	pixels = (unsigned char*)(img->pixels);
	for(j = 0; j < (unsigned int) map->size_y; j++)
	{
		for (i = 0; i < (unsigned int) map->size_x; i++)
		{
			// Compute mean of RGB for this pixel
			p = pixels + j*rowstride + i*n_channels;
			color_sum = 0;
			for(k=0;k<n_channels;k++)
			color_sum += *(p + (k));
			color_avg = color_sum / (double)n_channels;

			// If negate is true, we consider blacker pixels free, and whiter
			// pixels free.  Otherwise, it's vice versa.
			if(negate)
			occ = color_avg / 255.0;
			else
			occ = (255 - color_avg) / 255.0;

			// Apply thresholds to RGB means to determine occupancy values for
			// map.  Note that we invert the graphics-ordering of the pixels to
			// produce a map with cell (0,0) in the lower-left corner.
		 	map->cells[MAP_INDEX(map, i, map->size_y -j-1)].occupy_probility = occ;

		 	if(occ >=0.4 && occ <=0.6) map->cells[MAP_INDEX(map, i, map->size_y -j-1)].update_times = 0;
		 	else map->cells[MAP_INDEX(map, i, map->size_y -j-1)].update_times = 3;
		}
	}
	SDL_FreeSurface(img);
}

void OGMapper::mapSave_Callback(const OGMapping::save_map::ConstPtr& save_pointer)
{
	boost::recursive_mutex::scoped_lock ms(configuration_mutex_);
	if(map_saved_) return;
	ROS_INFO("save traverse map");
	OGMapper::mapsave();
	map_saved_ = true;
}

void OGMapper::mapsave()
{
	ROS_INFO("Received a %d X %d map @ %.3f m/pix",
	map_->size_x,
	map_->size_y,
	map_->scale);

	std::string mapdatafile = map_file_ + ".pgm";
	ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
	FILE* out = fopen(mapdatafile.c_str(), "w");
	if (!out)
	{
		ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
		return;
	}

	fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
			map_->scale, map_->size_x, map_->size_y);

	for(unsigned int y = 0; y <  (unsigned int) map_->size_y; y++) {
		for(unsigned int x = 0; x <  (unsigned int) map_->size_x; x++) {
		unsigned int i = x + (map_->size_y - y - 1) * map_->size_x;

		//rewrite the mapping function from probability to gray scale;
		uint8_t pixel_value;
		OGMapper::probility_to_grayscale(map_->cells[i].occupy_probility, pixel_value);
		fputc(pixel_value, out);
		}
	}

	fclose(out);


	std::string mapmetadatafile = map_file_ + ".yaml";
	ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
	FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


	/*
	resolution: 0.100000
	origin: [0.000000, 0.000000, 0.000000]
	#
	negate: 0
	occupied_thresh: 0.65
	free_thresh: 0.196

	*/

	fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
		  mapdatafile.c_str(), map_->scale, map_->origin_x, map_->origin_y, map_->yaw);

	fclose(yaml);

	ROS_INFO("Done\n");
}

	//there is unnecessary to make a decision during map updating; only make this decision when yielding the final output;
	//probability ranges: clear [0, 0.4); unknown [0.4, 0.6]; occupied (0.6, 1.0];
	// clear floor(0.3999*256.0); unknown ceiling(0.4001*256), floor(0.5999*256); ceiling(0.6001*256), floor(1.0*256-0.00000001);
	//corresponding values: clear 255 - [0, 102]; unknown 255 - [103, 153]; occupied 255 - [154, 255];
	inline void OGMapper::grayscale_to_probility(uint8_t & pixel_value, uint8_t & occ_prob)
	{
		//inevitable reduced accuracy;
		occ_prob = 1.0/256.0 * double( 255 -pixel_value);
	}
	inline void OGMapper::probility_to_grayscale(double & occ_prob, uint8_t & pixel_value)
	{
		pixel_value = 255 - uint8_t(floor((occ_prob+DBL_MIN)*256.0));
	}

	OGMapper::~OGMapper()
	{
		if( map_ != NULL )
		{
			map_free( map_ );
			map_ = NULL;
		}
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "OGMapper");
	ros::NodeHandle n;
	golfcar_perception::OGMapper OGMapper_node;
	ros::spin();
	return 0;
}
