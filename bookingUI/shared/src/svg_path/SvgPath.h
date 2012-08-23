#include <iostream>

#include <tinyxml.h>

#include "StationPath.h"

class SvgPath
{
public:
    SvgPath();
    SvgPath(const char* pFilename, double resolution);
    void loadFile(const char* pFilename, double resolution);
    void getSlowZones(std::vector<SlowZone>* slowZones);


    StationPath getPath(std::string id);
    std::vector<SlowZone> getSlowZone();
    void GetDoubleAttribute(TiXmlElement* childElement, const char* attributeName, double* value);

private:
    TiXmlDocument svgDoc_;
    double res_;
    PathPoint size_;
    unsigned int foundPath, foundSlowZone;

    StationPath StringToPath(std::string value);

    void convert_to_meter(StationPath* pose);

    PathPoint GetTransform(TiXmlElement* childElement);

    PathPoint getSize();
};
