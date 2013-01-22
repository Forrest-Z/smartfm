/*
 * voro_field.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: liuwlz
 */

#include <voronoi_field.h>

bool debug = false;
DynamicVoronoi voronoi_field;

VoroField::VoroField()
{
	resolution = 0;
	height = 0;
	width = 0;
	//*gridCell[] = NULL;
	map_sub = nh.subscribe("local_map",1, &VoroField::map_CB, this);
	cost_map_pub = nh.advertise<sensor_msgs::PointCloud> ("local_map_pub", 1);
	//voronoi_field.DynamicVoronoi();
	//local_map_sub = nh.subscribe()
	//!Voronoi field params Initialise
	//DmaxActual = DMaxDefault / resolution;
	//AlphaActual = AlphaDefault / resolution;
}

VoroField::~VoroField(){
}

void VoroField::map_CB(const nav_msgs::OccupancyGrid map_grid)
{
	height = map_grid.info.height;
	width = map_grid.info.width;
	resolution = map_grid.info.resolution;

	local_map_pub.header = map_grid.header;
	local_map_pub.header.frame_id = "/base_link";
	//!Grid map initialise
	gridCell = new bool*[height];
	Time t;
	for (int x= 0; x< height; x++)
	{
		gridCell[x] = new bool[width];
	}
	for (int i=0; i< height;i++){
		for (int j = 0; j<width; j++){
			if(map_grid.data[i*width+j] != 0)
				gridCell[i][j] = false;
			else
				gridCell[i][j] = true;
		}
	}


	if (debug){
			cout<< "map received"<<width<<height<<endl;
		}
	voro_field_update();
	cout << "voro_field updated in "<<t.get_since()<<" seconds\n\n";

}

void VoroField::voro_field_update()
{
	if (debug)
		cout<< "Updating map"<<endl;
	voronoi_field.initializeMap(height, width, gridCell);
	voronoi_field.update();
	voronoi_field.prune();
	voronoi_field.updateVoro();
	//voronoi_field.visualize();
	cost_map_publish();
}

void VoroField::cost_map_publish()
{
	std::vector<geometry_msgs::Point32> cost;
	voronoi_field.updatePathCost(cost, resolution);
	local_map_pub.points = cost;
	cost_map_pub.publish(local_map_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voro_field_node");

    VoroField voroField;

    ros::spin();
    return 0;
};





