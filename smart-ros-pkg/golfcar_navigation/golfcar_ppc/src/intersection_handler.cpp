#include "intersection_handler.h"



IntersectionHandler::IntersectionHandler()
: marker_server_ ("intersection")
{
    marker_clicked_ = true;
}

bool IntersectionHandler::is_clear_to_go() const
{
    return marker_clicked_;
}

void IntersectionHandler::update_int(const geometry_msgs::Point & int_pt)
{
    if( int_point_.x!=int_pt.x && int_point_.y!=int_pt.y )
    {
        marker_clicked_ = false;
        int_point_ = int_pt;
        add_marker(int_pt);
        marker_server_.applyChanges();
    }
}

void IntersectionHandler::add_marker(const geometry_msgs::Point & p)
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
    int_marker.pose.position = p;
    int_marker.pose.orientation.w = 1;

    marker_server_.insert(int_marker,
            boost::bind(&IntersectionHandler::processFeedback, this, _1));
}

void IntersectionHandler::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if(feedback->event_type==visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
       marker_clicked_ = true;
    }
}
