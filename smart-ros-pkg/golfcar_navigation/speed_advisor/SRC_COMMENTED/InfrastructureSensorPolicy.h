#ifndef INFRASTRUCTURESENSORPOLICY_H_
#define INFRASTRUCTURESENSORPOLICY_H_

#include "IntersectionPolicy.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <boost/thread/thread.hpp>


class InfrastructureSensorPolicy : public IntersectionPolicy
{
public:
    /// Creates an InfrastructureSensorPolicy that will monitor the state of
    /// the infrastructure sensor server for the intersection referred by the
    /// given id.
    InfrastructureSensorPolicy(std::string id);


    /// Stops the monitoring thread and waits for completion
    ~InfrastructureSensorPolicy();

    bool is_clear_to_go();

private:
    /// intersection id to be used when querying the infrastructure sensor server
    std::string int_id_;

    /// The state of the infrastructure sensor
    bool infra_clear_;

    /// The running state of the thread (used by the destructor to kill it).
    bool running_;

    ros::NodeHandle nh_;
    ros::ServiceClient client_;

    boost::thread infra_srv_thread_;
    void thread_func();
};

#endif /* INFRASTRUCTURESENSORPOLICY_H_ */
