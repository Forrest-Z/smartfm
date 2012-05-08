#include "intersection_handler.h"

#include <fmutil/fm_math.h>
#include <infrastructure_road_monitoring/InfrastructureQuery.h>

using infrastructure_road_monitoring::InfrastructureQuery;


IntersectionHandler::IntersectionHandler()
: initialised_(false), marker_server_ ("intersection"), dist_to_int_(10000)
{
    client_ = nh_.serviceClient<InfrastructureQuery>("infrastructure_query");
    infra_timer_ = nh_.createTimer(ros::Duration(1),
            &IntersectionHandler::infra_timer_callback, this);
}

bool IntersectionHandler::is_clear_to_go() const
{
    return !initialised_ || marker_clicked_ || (monitoring_ && infra_clear_);
}

double IntersectionHandler::dist_to_int() const
{
    return dist_to_int_;
}

void IntersectionHandler::update(const pnc_msgs::move_status & status)
{
    bool new_int = ( int_point_.x!=status.int_point.x && int_point_.y!=status.int_point.y );

    /* If not initialised yet, then initialise with the new intersection info.
     * If receiving a new intersection, it means the vehicle has passed the
     * intersection point. This could be due to the vehicle overshooting the
     * scheduled stop point, hence we only proceed to the next intersection
     * if this intersection has been cleared.
     */
    if( !initialised_ || (new_int && is_clear_to_go()) )
    {
        ROS_DEBUG("Dealing with new intersection");
        initialised_ = true;
        marker_clicked_ = false;
        infra_clear_ = false;
        int_point_ = status.int_point;
        dist_to_int_ = status.dist_to_ints;

        add_marker();

        // Special case: if the next point is the t-junction (identified
        // by position), then launch the infrastructure sensor monitoring system.
        monitoring_ = (fmutil::distance(int_point_.x, int_point_.y, 32, 120) < 5);
    }
    else
    {
        //update dist_to_int_:
        // - only decrease
        // - zero if we passed the intersection point (new_int is true)
        if( new_int )
            dist_to_int_ = 0;
        else if( status.dist_to_ints < dist_to_int_)
            dist_to_int_ = status.dist_to_ints;
    }
}

void IntersectionHandler::infra_timer_callback(const ros::TimerEvent & dummy)
{
    if( ! monitoring_ )
        return;

    InfrastructureQuery srv;
    srv.request.id = "tjunc";
    if( client_.call(srv) )
    {
        infra_clear_ = srv.response.clear_to_go;
        ROS_DEBUG("Service infrastructure_query returned %s", (infra_clear_ ? "true" : "false"));
    }
    else
    {
        ROS_WARN("Failed to call service infrastructure_query");
        infra_clear_ = false;
    }
}

void IntersectionHandler::add_marker()
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
    int_marker.pose.position = int_point_;
    int_marker.pose.orientation.w = 1;

    marker_server_.insert(int_marker,
            boost::bind(&IntersectionHandler::process_feedback, this, _1));
    marker_server_.applyChanges();
}

void IntersectionHandler::process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if(feedback->event_type==visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
       marker_clicked_ = true;
       monitoring_ = false;
    }
}
