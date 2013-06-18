#ifndef SVGBOUNDARY_H_
#define SVGBOUNDARY_H_

#include <stdexcept>
#include <vector>
#include <string>

#include <tinyxml.h>

#include <geometry_msgs/Point32.h>

// this reads a path from the SVG file.
// It's actually quite similar to the SvgPath class, but Demian can't recall
// why we re-implemented from scratch...

class SvgBoundary
{
public:
    /// Constructs an empty object. loadFile must be called.
    SvgBoundary();

    /// Constructs and loads the file.
    /// @param f the name of the file to load (assumed to be a SVG map)
    /// @param resolution the resolution of the map (pixel to meter conversion)
    SvgBoundary(std::string f, double resolution);

    /// Loads the data from the file
    /// @param f the name of the file to load (assumed to be a SVG map)
    /// @param resolution the resolution of the map (pixel to meter conversion)
    void loadFile(std::string f, double resolution);

    /// Returns the selected path
    std::vector<geometry_msgs::Point32> getPath(std::string id);

private:
    TiXmlDocument svgDoc_;

    double height_; // height of the map
    double width_;  // width of the map
    double resolution_; // resolution of the map

    void convert_to_meter(std::vector<geometry_msgs::Point32>& pose);
    void getSize();
    std::vector<geometry_msgs::Point32> StringToPath(std::string data);
};

#endif /* SVGBOUNDARY_H_ */
