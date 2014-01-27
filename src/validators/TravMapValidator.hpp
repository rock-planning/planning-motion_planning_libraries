#ifndef _TRAV_MAP_VALIDATOR_HPP_
#define _TRAV_MAP_VALIDATOR_HPP_

#include <envire/maps/TraversabilityGrid.hpp>

#include <ompl/base/StateValidityChecker.h>

namespace global_path_planner
{

class TravMapValidator :  public ompl::base::StateValidityChecker {
 
 private:
    envire::TraversabilityGrid mTravGrid;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si) : 
            ompl::base::StateValidityChecker(si) {
    }
    
    bool isValid(const ompl::base::State* state) const
    {
        // Check borders
        //state->getX getY getYaw
        
        
        return true;
    }
 
    inline void setTravGrid(envire::TraversabilityGrid& trav_grid) {
        mTravGrid = trav_grid;
    }
    
    bool setTravGrid(envire::Environment& env, std::string trav_map_id);
};

} // end namespace global_path_planner

#endif
