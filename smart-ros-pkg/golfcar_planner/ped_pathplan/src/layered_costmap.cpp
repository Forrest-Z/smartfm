#include<algorithm>
#include"layered_costmap.h"

PedCostmap::PedCostmap(costmap_2d::Costmap2DROS* cm_ros) {
    cm_ros->getCostmapCopy(cm_inflat);
    cm_inflat.resetMaps();
}

const unsigned char* PedCostmap::get() {
    return cm_inflat.getCharMap();
}

CrowdCostmap::CrowdCostmap(costmap_2d::Costmap2DROS* cmros)
    :nx(cmros->getSizeInCellsX()), ny(cmros->getSizeInCellsY()),
    costarr(nx*ny, 0)
{
}

CrowdCostmap::~CrowdCostmap() {
}

CrowdCostmap::update() {
    fill(costarr.begin(), costarr.end(), 1.0);
    for(auto& p: peds) {
        
    }
}
