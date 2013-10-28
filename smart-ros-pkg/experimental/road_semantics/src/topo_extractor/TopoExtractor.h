#ifndef TOPO_EXTRACTOR_H
#define TOPO_EXTRACTOR_H

#include "datatypes.hh"
#include "heap.hh"
#include "utils.hh"
#include "topo_graph.hpp"

#include <iostream>
#include <float.h>
#include <vector>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fmutil/fm_math.h>

#define EDGE_LENGTH_THRESHOLD 30.0

using namespace std;
using namespace cv;

//for nodeCluster clustering using OpenCV function;
int is_equal( const void* _a, const void* _b, void* userdata )
{
	size_t a = *(const size_t*)_a;
	size_t b = *(const size_t*)_b;
	topo_graph  *graph_tmp = (topo_graph*) userdata;

	CvPoint2D32f center_a = graph_tmp->nodeClusters[a].cluster_center;
	CvPoint2D32f center_b = graph_tmp->nodeClusters[b].cluster_center;

	double radius_a = graph_tmp->nodeClusters[a].intersection_radius;
	double radius_b = graph_tmp->nodeClusters[b].intersection_radius;
	double radius_big = radius_a>radius_b?radius_a:radius_b;
	double center_distance = sqrt((center_a.x-center_b.x)*(center_a.x-center_b.x)+(center_a.y-center_b.y)*(center_a.y-center_b.y));
	return (center_distance <= radius_big);
}


class topo_extractor {
public:
  //the output from "topo_extractor";
  topo_graph road_graph_;
  int grid_size_x,grid_size_y;

  IplImage *binary_image_;

  topo_extractor(const grid_type&, float, float, bool);

  //extract nodes and edges from skeleton image, and organize them in "road_graph_";
  void extract_topology();

private:
  grid_type original_grid;
  float coastal_dist, prune_dist;
  bool prune;

  // Keeps track of distance to closest obstacle.
  class dist_cell {
  public:
    int x,y;
    float distance;
    friend bool operator> (const dist_cell& d1, const dist_cell& d2)  {
      return d1.distance>d2.distance;
    }
  };

  typedef vector <dist_cell> dist_col;
  typedef vector <dist_col> dist_grid;
  dist_grid distance_grid;
  
  void calculate_distances();
  typedef enum {occupied, free, skel, processing, processed, unknown} State;
  typedef vector < vector <State> > gridtype;  
  gridtype _step1_grid;
  gridtype _step2_grid;
  bool on_grid(int,int);

  void find_skel();
  void initialize();
  void thin();
  
  class edge {
  public:
    edge(int i, int j) {
      x=i;
      y=j;
    }
    int x,y;
  };
  
  typedef deque<edge> queuetype;
  
  queuetype _step1_queue;  //!< holds cells to process in step1
  queuetype _step2_queue;  //!< holds cells to process in step2
  
  State step(gridtype&, edge&, bool);
  void find_skel_edge();

  //to further thining the skeleton to single pixel;
  void skel_thining();
  bool remove_center_pixel(int x, int y);

  void extract_node_edge();
  void Graph_Extraction(CvMat *pSrc);
  void build_topology();
  void printf_topology();
  inline bool check_8connectivity(CvPoint pt1, CvPoint pt2);
  void topo_filtering();

  void learn_metric_properties();
  void road_spline_fitting();
  void distance_properties();

  void merge_intersections();

  void visualize_TopoMetric_graph();
};  

#endif //evg_thin_hh 

