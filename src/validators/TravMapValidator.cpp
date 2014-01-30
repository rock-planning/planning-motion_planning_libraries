#include "TravMapValidator.hpp"

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
            state_se2->getY() >= 0 ||
            state_se2->getY() >= mpTravGrid->getCellSizeY()) {
        return false;
    }   
    
    // Check obstacle.
    envire::TraversabilityGrid::ArrayType &trav_data(
            mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    if(trav_data[state_se2->getY()][state_se2->getX()] == 
            envire::SimpleTraversability::CLASS_OBSTACLE) {
        return false;
    }
    
    return true;
}

} // end namespace global_path_planner

