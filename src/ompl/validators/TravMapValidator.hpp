#ifndef _TRAV_MAP_VALIDATOR_HPP_
#define _TRAV_MAP_VALIDATOR_HPP_

#include <vector>
#include <map>

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <motion_planning_libraries/Config.hpp>

namespace envire {
class TraversabilityGrid;
}

namespace motion_planning_libraries
{
    
typedef envire::TraversabilityGrid::ArrayType TravData;

class TravMapValidator :  public ompl::base::StateValidityChecker {
 
 private:
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    size_t mGridWidth;
    size_t mGridHeight;
    boost::shared_ptr<TravData> mpTravData;
    enum EnvType mEnvType;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            size_t grid_width, 
            size_t grid_height,
            boost::shared_ptr<TravData> grid_data,
            enum EnvType env_type);
    
    ~TravMapValidator();
    
    bool isValid(const ompl::base::State* state) const;

    /** 
     * Report the distance to the nearest invalid state when starting from \e state. If the distance is
     * negative, the value of clearance is the penetration depth.
     */
    double clearance(const ompl::base::State* state) const;

    /** 
     * Report the distance to the nearest invalid state when starting from \e state, and if possible,
     * also specify a valid state \e validState in the direction that moves away from the colliding
     * state. The \e validStateAvailable flag is set to true if \e validState is updated. 
     */
    double clearance(const ompl::base::State *state, ompl::base::State* validState, bool &validStateAvailable) const;   
};

} // end namespace motion_planning_libraries

#endif
