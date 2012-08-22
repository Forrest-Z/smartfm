#ifndef ONBOARDCAMPOLICY_H_
#define ONBOARDCAMPOLICY_H_

#include "IntersectionPolicy.h"
#include <vision_motion_detection/MotionExtractor.h>

class ExitingEALeftPolicy : public IntersectionPolicy
{
public:
    ExitingEALeftPolicy();
    ~ExitingEALeftPolicy();
    bool is_clear_to_go();
    void update_dist(double d);

private:
    MotionExtractor * right_cam_;
};


#endif /* ONBOARDCAMPOLICY_H_ */
