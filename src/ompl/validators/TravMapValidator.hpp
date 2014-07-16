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
#include <motion_planning_libraries/Helpers.hpp>

namespace envire {
class TraversabilityGrid;
}

namespace motion_planning_libraries
{
    
typedef envire::TraversabilityGrid::ArrayType TravData;

class TravMapValidator:  public ompl::base::StateValidityChecker {
 
 private:
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    envire::TraversabilityGrid* mpTravGrid; // To request the driveability values.
    boost::shared_ptr<TravData> mpTravData;
    Config mConfig;
    mutable GridCalculations mGridCalc;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            Config config);
 
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data,
            Config config);
    
    ~TravMapValidator();
    
    void setTravGrid(envire::TraversabilityGrid* trav_grid, boost::shared_ptr<TravData> trav_data);
    
    bool isValid(const ompl::base::State* state) const;
};

} // end namespace motion_planning_libraries

#endif
