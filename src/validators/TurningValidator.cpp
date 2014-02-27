#include "TurningValidator.hpp"

#include <stdio.h>

#include <ompl/base/spaces/SE2StateSpace.h>

namespace global_path_planner
{
    
bool TurningValidator::checkMotion (const ompl::base::State *s1, 
        const ompl::base::State *s2) const {
    bool ret = ompl::base::DiscreteMotionValidator::checkMotion(s1, s2);
    
    return ret && isValidTurn(s1, s2);
}

bool TurningValidator::checkMotion (const ompl::base::State *s1, 
        const ompl::base::State *s2, 
        std::pair< ompl::base::State *, double > &lastValid) const {
    bool ret = ompl::base::DiscreteMotionValidator::checkMotion(s1, s2, lastValid);
    
    return ret && isValidTurn(s1, s2);
}

bool TurningValidator::isValidTurn(const ompl::base::State *s1, 
        const ompl::base::State *s2) const {
    
    // Calculates 0, dont know why.
    //ompl::base::SE2StateSpace* se2_ss = si_->getStateSpace()->as<ompl::base::SE2StateSpace>();
    //double dist = se2_ss->as<ompl::base::RealVectorStateSpace>(0)->distance(s1, s2);  
    //double angle = se2_ss->as<ompl::base::SO2StateSpace>(1)->distance(s1, s2);

    
    double s1_x = s1->as<ompl::base::SE2StateSpace::StateType>()->getX();
    double s1_y = s1->as<ompl::base::SE2StateSpace::StateType>()->getY();
    double s1_yaw = s1->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
    double s2_x = s2->as<ompl::base::SE2StateSpace::StateType>()->getX();
    double s2_y = s2->as<ompl::base::SE2StateSpace::StateType>()->getY();
    double s2_yaw = s2->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
    
    double dist = sqrt((s1_x - s2_x)*(s1_x - s2_x)+(s1_y - s2_y)*(s1_y - s2_y));
    double angle = fabs(s1_yaw - s2_yaw);
    angle = angle > M_PI ? 2.0 * M_PI - angle : angle;
    double time_dist = dist / mMaxSpeed;
    double max_angle = mMaxTurningSpeed * time_dist;
    
    //printf("(%4.2f, %4.2f, %4.2f) to (%4.2f, %4.2f, %4.2f): %4.2f pixel, %4.2f rad, %4.2f max turn\n",
    //        s1_x, s1_y, s1_yaw, s2_x, s2_y, s2_yaw, dist, angle, max_angle);
    
    // Valid if the angle between these states is smaller or equal than
    // the turning capabilities of the system.
    return angle <= max_angle;
}
    
} // end namespace global_path_planner
