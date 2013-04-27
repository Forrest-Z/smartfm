#include "IntersectionPolicy.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
/** An IntersectionPolicy based on interactive markers. */
class SimpleBoolPolicy : public IntersectionPolicy
{
public:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool clear_;
    SimpleBoolPolicy() : clear_(false){
        sub_ = nh_.subscribe("intersection_clear", 10, &SimpleBoolPolicy::Callback, this); 
    };

    /// Returns whether it is safe to go through the intersection
    bool is_clear_to_go(){return clear_;};
private:
    void Callback(std_msgs::Bool clear) {
        clear_ = clear.data;
    }

};

