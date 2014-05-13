#ifndef _TURNING_VALIDATOR_HPP_
#define _TURNING_VALIDATOR_HPP_

#include <vector>
#include <map>

#include <ompl/base/DiscreteMotionValidator.h>

namespace envire {
class TraversabilityGrid;
}

namespace motion_planning_libraries
{

/**
 * Checks if the path between the states does not run through an obstacle
 * and if the required turning angle does not exceed the maximum of the system. 
 */
class TurningValidator :  public ompl::base::DiscreteMotionValidator {
 
 private:
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    double mMaxSpeed;
    double mMaxSteeringAngleRad;
    
    bool isValidTurn(const ompl::base::State *s1, const ompl::base::State *s2) const;
    
 public:
    TurningValidator(const ompl::base::SpaceInformationPtr& si, 
            double max_speed, double max_steering_angle_rad) : 
            ompl::base::DiscreteMotionValidator(si),
            mMaxSpeed(max_speed), 
            mMaxSteeringAngleRad(max_steering_angle_rad) {
    }
    
    ~TurningValidator() {
    }
    
    bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const;

    bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2, 
            std::pair< ompl::base::State *, double > &lastValid) const;
    
    inline void setMaxSpeed(double max_speed) {
        mMaxSpeed = max_speed;
    }
    
    inline double getMaxSpeed() const {
        return mMaxSpeed;
    }
    
    inline void setMaxSteeringAngle(double max_steering_speed) {
        mMaxSteeringAngleRad = max_steering_speed;
    }
    
    inline double getMaxSteeringAngle() const {
        return mMaxSteeringAngleRad;
    }
};

} // end namespace motion_planning_libraries

#endif
