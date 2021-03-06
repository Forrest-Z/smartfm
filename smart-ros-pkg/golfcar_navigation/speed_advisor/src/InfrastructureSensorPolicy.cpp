#include "InfrastructureSensorPolicy.h"

#include <infrastructure_road_monitoring/InfrastructureQuery.h>
using infrastructure_road_monitoring::InfrastructureQuery;


InfrastructureSensorPolicy::InfrastructureSensorPolicy(std::string id)
: int_id_(id), infra_clear_(false), running_(true)
{
    // create the client
    client_ = nh_.serviceClient<InfrastructureQuery>("infrastructure_query");

    // create the thread
    ROS_DEBUG_NAMED("infrastructure", "Launching the thread");
    infra_srv_thread_ = boost::thread(
            boost::bind(&InfrastructureSensorPolicy::thread_func, this) );
}

InfrastructureSensorPolicy::~InfrastructureSensorPolicy()
{
    // stop the thread loop
    running_ = false;

    // wait for completion
    infra_srv_thread_.join();

    // NOTE: is it the best way to kill the thread?
}

bool InfrastructureSensorPolicy::is_clear_to_go()
{
    return infra_clear_;
}

void InfrastructureSensorPolicy::thread_func()
{
    ROS_DEBUG_NAMED("infrastructure", "Starting thread");
    while( running_ )
    {
        InfrastructureQuery srv;
        srv.request.id = int_id_;
        if( client_.call(srv) )
        {
            infra_clear_ = srv.response.clear_to_go;
            ROS_DEBUG_NAMED("infrastructure",
                    "Service infrastructure_query returned %s clear=%s",
                    int_id_.c_str(), (infra_clear_ ? "true" : "false"));
        }
        else
        {
            ROS_WARN_THROTTLE_NAMED(3, "infrastructure", "Failed to call service infrastructure_query");
            infra_clear_ = false;
        }
    }
    ROS_DEBUG_NAMED("infrastructure", "Thread exiting");
}
