#ifndef _TRAV_MAP_VALIDATOR_HPP_
#define _TRAV_MAP_VALIDATOR_HPP_

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <base/samples/RigidBodyState.hpp>

namespace envire {
class TraversabilityGrid;
}

namespace global_path_planner
{

class TravMapValidator :  public ompl::base::StateValidityChecker {
 
 public:
    //std::vector<base::samples::RigidBodyState> mSamples;
 
 private:
    envire::TraversabilityGrid* mpTravGrid;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            envire::TraversabilityGrid* trav_grid) : 
            ompl::base::StateValidityChecker(si),
            mpTravGrid(trav_grid) {
    }
    
    bool isValid(const ompl::base::State* state) const;
};

} // end namespace global_path_planner

#endif
