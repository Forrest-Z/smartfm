#include <iostream>

#include <tinyxml.h>

#include "StationPath.h"

class SvgPath
{
public:
    SvgPath(const char* pFilename);
    SvgPath(const char* pFilename, StationPath* pose, PathPoint *size, const char* id);
    void getSlowZones(std::vector<SlowZone>* slowZones);
    void convert_to_meter(StationPath* pose, PathPoint &size, double resolution);

private:


    TiXmlElement* svgElement_;
    unsigned int foundPath, foundSlowZone;

    void findPathElements(TiXmlNode* pParent, StationPath* pose, unsigned int *npoints, const char* id);

    std::string string_error(std::string description, std::string details);

    int find_size_attributes(TiXmlElement* pElement, PathPoint *width_height);

    int find_path_attributes(TiXmlElement* pElement, StationPath* pose, const char* id);

    int find_path_details(TiXmlElement* pElement, StationPath* pose);
    bool match_path_id(TiXmlElement* pElement, const char* id);

    void findSize(TiXmlNode* pParent, PathPoint* size);
};
