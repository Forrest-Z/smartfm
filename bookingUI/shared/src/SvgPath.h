#include <tinyxml.h>
#include <iostream>
#include <sstream>
using namespace std;
#include <stdio.h>
#include <StationPath.h>

class SvgPath
{
    
public:
    
    SvgPath(const char* pFilename, StationPath* pose, PathPoint *size, const char* id);
    void convert_to_meter(StationPath* pose, PathPoint &size, double resolution);
    
private:
    unsigned int foundPath;
    void findPathElements(StationPath* pose, PathPoint* size, unsigned int *npoints, TiXmlNode* pParent, const char* id);
   
    string string_error(string description, string details);
      
    int find_size_attributes(TiXmlElement* pElement, PathPoint *width_height);
   
    int find_path_attributes(TiXmlElement* pElement, StationPath* pose, const char* id);
    

    
};
