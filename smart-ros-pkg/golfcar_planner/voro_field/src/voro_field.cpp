/*
 * voro_field.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: liuwlz
 */

#include <voronoi_field.h>

bool debug = true;

VoroField::VoroField()
{
	resolution = 0;
	height = 0;
	width = 0;
	//*gridCell[] = NULL;
	map_sub = nh.subscribe("map",1, &VoroField::map_CB, this);
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
	//!Grid map initialise
	gridCell = new bool*[height];
	for (int x= 0; x< height; x++)
	{
		gridCell[x] = new bool[width];
	}
	for (int i=0; i< height;i++){
		for (int j = 0; j<width; j++){
			if(map_grid.data[i*width+j] != 0)
				gridCell[i][j] = true;
			else
				gridCell[i][j] = false;
		}
	}
	if (debug){
			cout<< "map received"<<width<<height<<endl;
		}
	voro_field_update();
}

void VoroField::voro_field_update()
{
	if (debug)
		cout<< "Updating map"<<endl;
	DynamicVoronoi voronoi_field;
	voronoi_field.initializeMap(height, width, gridCell);
	voronoi_field.update();
	voronoi_field.prune();
	voronoi_field.updateVoro();
	voronoi_field.visualize();
}

/*
void VoroField::cost_map_publish()
{
	map_pub.header = map.header;
	map_pub.info = map.info;
	map_pub.data = grid[NumRows*NumColumns];
	voro_field_pub.publish(map_pub);
}
*/



int main(int argc, char **argv)
{
    ros::init(argc, argv, "voro_field_node");

    VoroField voroField;

    ros::spin();
    return 0;
};





