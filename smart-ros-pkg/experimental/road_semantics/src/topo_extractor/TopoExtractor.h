
#ifndef TOPO_EXTRACTOR_H
#define TOPO_EXTRACTOR_H

#include "datatypes.hh"
#include "heap.hh"
#include "utils.hh"
#include "../topo_graph.hpp"

#include <iostream>
#include <float.h>
#include <vector>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fmutil/fm_math.h>

using namespace std;

class topo_extractor {
public:
  //the output from "topo_extractor";
  topo_graph road_graph_;

  topo_extractor(const grid_type&, float, float, bool);

  //extract nodes and edges from skeleton image, and organize them in "road_graph_";
  void extract_topology();

private:
  grid_type original_grid;
  int grid_size_x,grid_size_y;
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
  void Graph_Extraction(CvMat *pSrc, CvMat *pDst, CvMat *pDst2);
  void build_topoloty();
  inline bool check_8connectivity(CvPoint pt1, CvPoint pt2);
  void topo_filtering();
};  

#endif //evg_thin_hh 

