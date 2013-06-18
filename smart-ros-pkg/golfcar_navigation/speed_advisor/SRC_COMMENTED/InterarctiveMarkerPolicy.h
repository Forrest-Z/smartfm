#ifndef INTERARCTIVEMARKERPOLICY_H_
#define INTERARCTIVEMARKERPOLICY_H_

#include "IntersectionPolicy.h"
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>


/** An IntersectionPolicy based on interactive markers. */
class InteractiveMarkerPolicy : public IntersectionPolicy
{
public:
    InteractiveMarkerPolicy(geometry_msgs::Point);

    /// Returns whether it is safe to go through the intersection
    bool is_clear_to_go();


private:
    ///Marker server
    interactive_markers::InteractiveMarkerServer marker_server_;

    /// Whether the marker has been clicked
    bool marker_clicked_;

    /// Called when the marker is clicked
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);
};

#endif /* INTERARCTIVEMARKERPOLICY_H_ */
