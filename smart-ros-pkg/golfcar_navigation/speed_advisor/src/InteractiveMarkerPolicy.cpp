#include "InterarctiveMarkerPolicy.h"


InteractiveMarkerPolicy::InteractiveMarkerPolicy(geometry_msgs::Point int_pt)
: marker_server_("intersection"), marker_clicked_(false)
{
    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 1;
    box_marker.scale.y = 1;
    box_marker.color.a = 1;
    box_marker.color.r = 0.5;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.markers.push_back( box_marker );
    box_control.always_visible = true;
    box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "Int";
    int_marker.description = "Intersection";
    int_marker.controls.push_back( box_control );
    int_marker.pose.position = int_pt;
    int_marker.pose.orientation.w = 1;

    marker_server_.insert(int_marker,
            boost::bind(&InteractiveMarkerPolicy::processFeedback, this, _1));
    marker_server_.applyChanges();
}

bool InteractiveMarkerPolicy::is_clear_to_go()
{
    return marker_clicked_;
}

void InteractiveMarkerPolicy::processFeedback
    (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(feedback->event_type==visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
       marker_clicked_ = true;
}
