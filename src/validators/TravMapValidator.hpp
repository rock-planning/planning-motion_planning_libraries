#ifndef _TRAV_MAP_VALIDATOR_HPP_
#define _TRAV_MAP_VALIDATOR_HPP_

#include <envire/maps/TraversabilityGrid.hpp>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace global_path_planner
{

class TravMapValidator :  public ompl::base::StateValidityChecker {
 
 private:
    envire::TraversabilityGrid* mpTravGrid;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            envire::TraversabilityGrid* trav_grid) : 
            ompl::base::StateValidityChecker(si),
            mpTravGrid(trav_grid) {
    }
    
    bool isValid(const ompl::base::State* state) const
    {
        const ompl::base::SE2StateSpace::StateType* state_se2 = 
                state->as<ompl::base::SE2StateSpace::StateType>();
    
        if(mpTravGrid == NULL) {
            LOG_WARN("The traversability map has not been set yet, states are invalid");
            return false;
        }
    
        // Check borders.
        if(state_se2->getX() < 0 ||
                state_se2->getX() >= mpTravGrid->getCellSizeX() ||
                state_se2->getY() >= 0 ||
                state_se2->getY() >= mpTravGrid->getCellSizeY()) {
            return false;
        }   
        
        // Check obstacle.
       
        
        
        return true;
    }
};

} // end namespace global_path_planner

#endif
