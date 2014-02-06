#include "TravMapValidator.hpp"

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>

namespace global_path_planner
{

bool TravMapValidator::isValid(const ompl::base::State* state) const
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
            state_se2->getY() < 0 ||
            state_se2->getY() >= mpTravGrid->getCellSizeY()) {
        LOG_INFO("State (%4.2f,%4.2f) is invalid (not within the grid)",  
                state_se2->getX(), state_se2->getY());
        return false;
    }   
    
    // Check obstacle.
    envire::TraversabilityGrid::ArrayType &trav_data(
            mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    if(trav_data[state_se2->getY()][state_se2->getX()] == 
            envire::SimpleTraversability::CLASS_OBSTACLE) {
        LOG_INFO("State (%4.2f,%4.2f) is invalid (lies on an obstacle)", 
                state_se2->getX(), state_se2->getY());
        return false;
    }
    
    return true;
}

} // end namespace global_path_planner

