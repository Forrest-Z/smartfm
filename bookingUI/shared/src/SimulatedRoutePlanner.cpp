#include <stdio.h>

#include "SimulatedRoutePlanner.h"
#include "GlobalClock.h"

SimulatedRoutePlanner::SimulatedRoutePlanner(const StationPaths & sp, float speed)
  : RoutePlanner(sp), vehicle_speed(speed), last_time(0.0), last_disp_time(0.0),
    verbose(false)
{
    
}

bool SimulatedRoutePlanner::goToDest()
{
    if( vehicle_speed > 0 )
    {
        if( last_time==0 ) {
            last_time = GlobalClock::time();
            last_disp_time = last_time;
            return false;
        }

        double current_time = GlobalClock::time(), dt = current_time-last_time;
        last_time = current_time;

        distance_travelled += vehicle_speed * dt;
        eta_ = (distance_to_travel-distance_travelled)/vehicle_speed;

        if( verbose && current_time > last_disp_time+1 )
        {
            last_disp_time = current_time;
            printf("Going to %s: travelled %dm out of %dm (%d%%). ETA=%d.\n",
                    destination_.c_str(), (int)distance_travelled,
                    (int)distance_to_travel,
                    (int)(distance_travelled/distance_to_travel*100),
                    (int)(eta_>=0?eta_:0) );
            fflush(stdout);
        }
    }

    if (vehicle_speed<=0 || distance_travelled > distance_to_travel) {
        printf("Reached %s.\n", destination_.c_str());
        fflush(stdout);
        eta_ = 0;
        return true;
    }
    return false;
}

void SimulatedRoutePlanner::initDest()
{
    distance_travelled = 0;
    distance_to_travel = sp_.getPath(currentStation_, destination_).length();
    last_time = 0;
    printf("Moving from %s to %s", currentStation_.c_str(), destination_.c_str());
}
