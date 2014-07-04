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
{
    costarr = new float[nx*ny];

}

CrowdCostmap::~CrowdCostmap() {
    delete[] costarr;
}

CrowdCostmap::update() {
    for()
}
