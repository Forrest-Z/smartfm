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

    bool is_clear_to_go()
    { return infra.is_clear_to_go() || marker.is_clear_to_go(); }

    void update_dist(double d)
    { infra.update_dist(d); marker.update_dist(d); }
};


class EAPedCrossingPolicy : public IntersectionPolicy
{
    LaserAreaPolicy *area;
    InteractiveMarkerPolicy marker;

public:
    EAPedCrossingPolicy(geometry_msgs::Point p) : area(0), marker(p)
    {
        ROS_DEBUG_NAMED("intersection", "Instantiating an EAPedCrossingPolicy");
        try
        {
            area = new LaserAreaPolicy("crossing_boundary");
        }
        catch( std::runtime_error & e )
        {
            ROS_WARN_NAMED("intersection", "Problem with the LaserAreaPolicy: %s", e.what());
            area = 0;
        }
    }

    ~EAPedCrossingPolicy() { if(area) delete area; }

    bool is_clear_to_go()
    { return (area!=0 && area->is_clear_to_go()) || marker.is_clear_to_go(); }

    void update_dist(double d)
    { if(area) area->update_dist(d); marker.update_dist(d); }
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
        // this happens when there is no intersection points ahead
       if( policy_ )
        {
            delete policy_;
            policy_ = 0;
        }
        dist_to_int_ = 100000;
        int_point_ = status.int_point;
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
        ROS_DEBUG_STREAM_NAMED("intersection", "Dealing with new intersection. " << status);
        int_point_ = status.int_point;
        dist_to_int_ = status.dist_to_ints;

        if( policy_ ) delete policy_;

        // Special cases: identified by position
        geometry_msgs::Point tjunc, ped;
        //tjunc.x = 32; tjunc.y = 120; //curb map
        //ped.x = 63; ped.y = 300; //curb map
        tjunc.x = 196; tjunc.y = 199; //dense map
        ped.x = 52; ped.y = 229; //dense map


        if( fmutil::distance<geometry_msgs::Point>(int_point_, tjunc) < 15 )
        {
            ROS_DEBUG_NAMED("intersection", "T Junction");
            policy_ = new TJunctionPolicy(int_point_);
        }
        else if( fmutil::distance<geometry_msgs::Point>(int_point_, ped) < 15 )
        {
            ROS_DEBUG_NAMED("intersection", "Pedestrian crossing");
            policy_ = new EAPedCrossingPolicy(int_point_);
        }
        else
        {
            ROS_DEBUG_NAMED("intersection", "Normal intersection (interactive)");
            policy_ = new InteractiveMarkerPolicy(int_point_);
        }
        policy_->update_dist(dist_to_int_);
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
        ROS_DEBUG_NAMED("intersection", "dist_to_int_=%f", dist_to_int_);

        assert( policy_ );
        policy_->update_dist(dist_to_int_);
    }
}
