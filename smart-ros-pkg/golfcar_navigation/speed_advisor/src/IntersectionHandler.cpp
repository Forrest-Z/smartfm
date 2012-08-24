#include "IntersectionHandler.h"
#include "InterarctiveMarkerPolicy.h"
#include "InfrastructureSensorPolicy.h"
#include "LaserAreaPolicy.h"

#include <fmutil/fm_math.h>


class TJunctionPolicy : public IntersectionPolicy
{
    InfrastructureSensorPolicy infra;
    InteractiveMarkerPolicy marker;

public:
    TJunctionPolicy(geometry_msgs::Point p) : infra("tjunc"), marker(p) { }
    bool is_clear_to_go() { return infra.is_clear_to_go() || marker.is_clear_to_go(); }
};


class EAPedCrossingPolicy : public IntersectionPolicy
{
    LaserAreaPolicy area;
    InteractiveMarkerPolicy marker;

public:
    EAPedCrossingPolicy(geometry_msgs::Point p) : area("crossing_boundary"), marker(p) { }
    bool is_clear_to_go() { return area.is_clear_to_go() || marker.is_clear_to_go(); }
};




IntersectionHandler::IntersectionHandler()
: dist_to_int_(10000), policy_(0)
{

}

bool IntersectionHandler::is_clear_to_go() const
{
    return policy_==0 || policy_->is_clear_to_go();
}

double IntersectionHandler::dist_to_int() const
{
    return dist_to_int_;
}

void IntersectionHandler::update(const pnc_msgs::move_status & status)
{
    if( status.int_point.x==0 && status.int_point.y==0 )
    {
        // does this really happen ?

        ROS_DEBUG_THROTTLE(1, "int_point==(0,0)");
        if( policy_ )
        {
            delete policy_;
            policy_ = 0;
        }
        dist_to_int_ = 100000;
        return;
    }

    bool new_int = ( int_point_.x!=status.int_point.x && int_point_.y!=status.int_point.y );

    /* If not initialised yet, then initialise with the new intersection info.
     * If receiving a new intersection, it means the vehicle has passed the
     * intersection point. This could be due to the vehicle overshooting the
     * scheduled stop point, hence we only proceed to the next intersection
     * if this intersection has been cleared.
     */
    if( policy_==0 || (new_int && policy_->is_clear_to_go()) )
    {
        ROS_DEBUG_STREAM("Dealing with new intersection. " << status);
        int_point_ = status.int_point;
        dist_to_int_ = status.dist_to_ints;

        if( policy_ ) delete policy_;

        // Special case: if the next point is the t-junction (identified
        // by position), then launch the infrastructure sensor monitoring system.
        if( fmutil::distance(int_point_.x, int_point_.y, 32, 120) < 5 )
            policy_ = new TJunctionPolicy(int_point_);
        else if( fmutil::distance(int_point_.x, int_point_.y, 63, 300) < 20 )
            policy_ = new EAPedCrossingPolicy(int_point_);
        else
            policy_ = new InteractiveMarkerPolicy(int_point_);
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
        if( policy_ ) policy_->update_dist(dist_to_int_);
    }
}
