#ifndef INTERSECTIONHANDLER_H_
#define INTERSECTIONHANDLER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>
#include <pnc_msgs/move_status.h>

#include <boost/thread/thread.hpp>


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
    IntersectionHandler();

    /** Updates the handler with the position of the next intersection location.
    *
    * Creates a new marker on the map when the next intersection is different
    * from the current intersection.
    */
    void update(const pnc_msgs::move_status &);

    /// Returns whether it is safe to go through the intersection
    bool is_clear_to_go() const;

    /// Returns the distance to the intersection point
    double dist_to_int() const;


private:
    bool initialised_;

    /// Position of the current intersection we are dealing with
    geometry_msgs::Point int_point_;

    /// Distance to the intersection, updated by update()
    double dist_to_int_; // previously last_ints_dist_

    /// Whether the marker has been clicked
    bool marker_clicked_;

    /// Whether we want to monitor the infrastructure sensor
    bool monitoring_;

    /// The state of the infrastructure sensor
    bool infra_clear_;

    ros::NodeHandle nh_;
    ros::ServiceClient client_;

    // It is not wise to use a timer to call the service as the call mechanism is
    // darn slow. Since the timer is in the same queue as other messages, it
    // will slow down the whole speed_advisor. Use a thread instead
    ros::Timer infra_timer_;
    boost::thread infra_srv_thread_;

    /// Marker server
    interactive_markers::InteractiveMarkerServer marker_server_;

    /// Puts a marker on rviz at int_point_
    void add_marker();

    /// Called when the marker is clickeddd
    void process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

    /// check the infrastructure sensor monitoring
    void infra_thread_fun();
};



#endif /* INTERSECTIONHANDLER_H_ */
