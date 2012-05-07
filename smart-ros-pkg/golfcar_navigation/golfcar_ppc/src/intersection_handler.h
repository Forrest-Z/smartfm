#ifndef INTERSECTIONHANDLER_H_
#define INTERSECTIONHANDLER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>

/** A class to handle going through intersections.
 *
 * Some intersections are equipped with an infrastructure sensor that tells the
 * car whether it is safe to go through. For some other intersections, the
 * passenger is required to give the clear to go signal by clicking on rviz.
 *
 * For now there is only one intersection with an infrastructure sensor. This
 * is currently identified by location (i.e. by comparing the global position
 * with some hard-coded value). Since this is still experimental, we keep the
 * manual signal in parallel: the passenger can click on rviz to give the clear
 * to go signal and override the infrastructure sensor signal.
 */
class IntersectionHandler
{
public:
    double last_ints_dist_;

    IntersectionHandler();

    /** Updates the handler with the position of the next intersection location.
    *
    * Creates a new marker on the map when the next intersection is different
    * from the current intersection.
    */
    void update_int(const geometry_msgs::Point &);

    bool is_clear_to_go() const;

private:
    /// Position of the current intersection we are dealing with
    geometry_msgs::Point int_point_;

    bool marker_clicked_;

    /// Marker server
    interactive_markers::InteractiveMarkerServer marker_server_;

    /// Puts a marker on rviz at the given location
    void add_marker(const geometry_msgs::Point &);

    /// Called when the marker is clickeddd
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);
};



#endif /* INTERSECTIONHANDLER_H_ */
