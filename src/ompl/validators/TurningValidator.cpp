#include "TurningValidator.hpp"

#include <stdio.h>

#include <ompl/base/spaces/SE2StateSpace.h>

namespace motion_planning_libraries
{
    
bool TurningValidator::checkMotion (const ompl::base::State *s1, 
        const ompl::base::State *s2) const {
    //bool ret = ompl::base::DiscreteMotionValidator::checkMotion(s1, s2);
    
    return isValidTurn(s1, s2);
}

bool TurningValidator::checkMotion (const ompl::base::State *s1, 
        const ompl::base::State *s2, 
        std::pair< ompl::base::State *, double > &lastValid) const {
    //bool ret = ompl::base::DiscreteMotionValidator::checkMotion(s1, s2, lastValid);
    
    return isValidTurn(s1, s2);
}

bool is_similar(double n1, double n2) {
    double p = ((n1 + n2) / 2.0) * 0.1;
    return (n2 > (n1 - p) && n2 < (n1 + p)); 
}

bool TurningValidator::isValidTurn(const ompl::base::State *s1, 
        const ompl::base::State *s2) const {
    
    // Calculates 0, dont know why.
    //ompl::base::SE2StateSpace* se2_ss = si_->getStateSpace()->as<ompl::base::SE2StateSpace>();
    //double dist = se2_ss->as<ompl::base::RealVectorStateSpace>(0)->distance(s1, s2);  
    //double angle = se2_ss->as<ompl::base::SO2StateSpace>(1)->distance(s1, s2);

    double s1_x = 0.0;
    double s1_y = 0.0;
    double s1_yaw = 0.0;
    double s2_x = 0.0;
    double s2_y = 0.0;
    double s2_yaw = 0.0;
    
    switch(mEnvType) {
        case ENV_XY: {
            return true;
        }
        case ENV_XYTHETA: {
            double s1_x = s1->as<ompl::base::SE2StateSpace::StateType>()->getX();
            double s1_y = s1->as<ompl::base::SE2StateSpace::StateType>()->getY();
            double s1_yaw = s1->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
            double s2_x = s2->as<ompl::base::SE2StateSpace::StateType>()->getX();
            double s2_y = s2->as<ompl::base::SE2StateSpace::StateType>()->getY();
            double s2_yaw = s2->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
            break;
        }
    }
    
    
   
    double dist = sqrt((s1_x - s2_x)*(s1_x - s2_x)+(s1_y - s2_y)*(s1_y - s2_y));
/* 
    double angle = fabs(s1_yaw - s2_yaw);
    angle = angle > M_PI ? 2.0 * M_PI - angle : angle;
    double time_dist = dist / mMaxSpeed;
    double max_angle = mMaxTurningSpeed * time_dist;
    
     
    //printf("(%4.2f, %4.2f, %4.2f) to (%4.2f, %4.2f, %4.2f): %4.2f pixel, %4.2f rad, %4.2f max turn\n",
    //        s1_x, s1_y, s1_yaw, s2_x, s2_y, s2_yaw, dist, angle, max_angle);
    
    // Valid if the angle between these states is smaller or equal than
    // the turning capabilities of the system.
    return angle <= max_angle;
*/
    
    double car_length = 1.0;
    double max_dist = 0.2; // Waypoints must be close together to create a valid path.
    double max_steering_angle_rad = M_PI / 9.0;
    
    
    //printf("State 1 (%4.2f, %4.2f, %4.2f), State 2 (%4.2f, %4.2f, %4.2f)\n", 
    //        s1_x, s1_y, s1_yaw, s2_x, s2_y, s2_yaw);
    
    
    /*
    if(dist > max_dist) {
        printf("dist (%4.2f) > max_dist\n", dist);
        return false;
    }
    */
    
    // Transform s2 into s1
    double s1_x_t = 0;
    double s1_y_t = 0;
    double s1_yaw_t = 0;
    double s2_x_t = s2_x - s1_x;
    double s2_y_t = s2_y - s1_y;
    double s2_yaw_t = s2_yaw - s1_yaw;
    s2_x_t = s2_x_t * cos(-s1_yaw) - s2_y_t * sin(-s1_yaw);
    s2_y_t = s2_x_t * sin(-s1_yaw) + s2_y_t * cos(-s1_yaw);
    
    //printf("State 1_t (%4.2f, %4.2f, %4.2f), State 2_t (%4.2f, %4.2f, %4.2f)\n", 
    //    s1_x_t, s1_y_t, s1_yaw_t, s2_x_t, s2_y_t, s2_yaw_t);
   
    double steering_angle_rad = atan(car_length / (s1_x_t - s2_x_t));
    if(fabs(steering_angle_rad) > max_steering_angle_rad) {
        //printf("steering_angle (%4.2f) > max_steering_angle\n", steering_angle_rad);
        //return false;
    }

    // Next state is accessible by a straight line. 
    if(is_similar(s1_y_t, s2_y_t) && is_similar(s1_yaw_t, s2_yaw_t)) {
        printf("Next state can be reached via a straight line\n");
        return true;
    }
    
    // Next state is accessible by a valid, consistent steering angle.
    if(is_similar(abs(s2_x_t), abs(s2_y_t)) && is_similar(M_PI/2.0, abs(s2_yaw_t))) {
        printf("Next state can be reached via a constant turn\n");
        return true;
    }
    
    return false;
}
    
} // end namespace motion_planning_libraries
