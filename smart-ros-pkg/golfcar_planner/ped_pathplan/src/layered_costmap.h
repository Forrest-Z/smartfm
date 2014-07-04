#ifndef LAYERED_COSTMAP_H_
#define LAYERED_COSTMAP_H_

#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

class PedBeliefCostmap;

class LayeredCostmap {
    LayeredCostmap(costmap_2d::Costmap2DROS* cm_ros);
    void update();

    costmap_2d::Costmap2DROS* cm_static;
    BeliefCostmap cm_peds;

    std::vector<float> costarr;
    int nx, ny;
};

class CrowdCostmap {
    CrowdCostmap(costmap_2d::Costmap2DROS* cm_ros);
    std::vector<PedCostmap*> peds;
    std::vector<float> costarr;
    int nx, ny;
};

class PedCostmap {
    PedCostmap(costmap_2d::Costmap2DROS* cm_ros);
    costmap_2d::Costmap2D cm_inflat;
    const unsigned char* get();
};

#endif

