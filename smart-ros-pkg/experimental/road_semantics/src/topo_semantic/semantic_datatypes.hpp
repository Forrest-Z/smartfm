#ifndef SEMANTIC_DATATYPES_HPP
#define SEMANTIC_DATATYPES_HPP

#include <vector>
#include <opencv/cv.h>

using namespace std;

class semantic_assembly
{
	public:

	class intersection
	{
		public:
		int nodeClusterID;
		int way_number;
		double x, y;
		vector <CvPoint2D32f> polygon;
	};

	class roundabout
	{
		public:
		vector<int> nodeClusterIDs;	//containing nodeClusters;
		vector<int> edgeIDs;	//connected edges;
		double x, y, radius;
		vector <CvPoint2D32f> polygon;
	};

	class roadlink
	{
		public:
		int	edgeID;
		double width;
	};

	vector<intersection> intersections;
	vector<roundabout> roundabouts;
	vector<roadlink> roadlinks;
};  

#endif
