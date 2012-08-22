#ifndef INTERSECTIONHANDLER_H_
#define INTERSECTIONHANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pnc_msgs/move_status.h>

#include "IntersectionPolicy.h"
#include "InterarctiveMarkerPolicy.h"
#include "InfrastructureSensorPolicy.h"


/** A class to handle going through intersections.
 *
 * At each intersection point corresponds a specific behaviour. For instance
 * at the t-junction the car can check the infrastructure sensor if it's up
 * and running, and it also uses the interactive marker on rviz for human
 * feedback. At EA and E3A, it uses the onboard cameras, etc.
 *
 * Each behavior is implemented by an IntersectionPolicy object. The
 * IntersectionHandler is in charge of launching the relevant policies and
 * taking the final decision.
 */
class IntersectionHandler
{
public:
    IntersectionHandler();

    /** Updates the handler and creates a new set of policies when receiving a
     * new intersection.
     * @param move_status contains the position of the next intersection.
     */
    void update(const pnc_msgs::move_status &move_status);

    /// Returns whether it is safe to go through the intersection.
    bool is_clear_to_go() const;

    /// Returns the distance to the intersection point
    double dist_to_int() const;

private:
    bool initialised_;

    /// Position of the current intersection we are dealing with
    geometry_msgs::Point int_point_;

    /// Distance to the intersection, updated by update()
    double dist_to_int_;

    ros::NodeHandle nh_;

    /// active policy
    IntersectionPolicy * policy_;
};



#endif /* INTERSECTIONHANDLER_H_ */
