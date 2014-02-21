#include "TravMapValidator.hpp"



#include <ompl/base/SpaceInformation.h>

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
    
    // Maps position to grid coordinates.
    int x_grid = (int)(state_se2->getX() + 0.5);
    int y_grid = (int)(state_se2->getY() + 0.5);

    // Check borders.
    if(     x_grid < 0 || x_grid >= (int)mpTravGrid->getCellSizeX() ||
            y_grid < 0 || y_grid >= (int)mpTravGrid->getCellSizeY()) {
        LOG_INFO("State (%d,%d) is invalid (not within the grid)", x_grid, y_grid);
        return false;
    }   
    
    // Check obstacle.
    //envire::TraversabilityGrid::ArrayType &trav_data(
    //        mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    if((*mpTravData)[y_grid][x_grid] == envire::SimpleTraversability::CLASS_OBSTACLE) {
        LOG_INFO("State (%d,%d) is invalid (lies on an obstacle)", x_grid, y_grid);
        return false;
    }
    
    /* method is const...
    base::samples::RigidBodyState rbs;
    rbs.position = base::Vector3d(state_se2->getX(), state_se2->getY(), 0);
    rbs.orientation = Eigen::AngleAxis<double>(state_se2->getYaw(), base::Vector3d(0,0,1));
    rbs.time = base::Time::now();
    mSamples.push_back(rbs);
    */
    
    return true;
}

double TravMapValidator::clearance(const ompl::base::State* state) const
{
    if(mpTravGrid == NULL) {
        LOG_WARN("The traversability map has not been set yet, clearance cannot be calculated");
        return 0.0;
    }

    bool state_start_valid = isValid(state);
    bool state_cur_valid = state_start_valid;
    
    // Copy state TODO Does it have to be so complicated?
    ompl::base::State* state_cur_mem = mpSpaceInformation->allocState();
    
    const ompl::base::SE2StateSpace::StateType* state_start = 
            state->as<ompl::base::SE2StateSpace::StateType>();
    ompl::base::SE2StateSpace::StateType* state_cur = 
            state_cur_mem->as<ompl::base::SE2StateSpace::StateType>();
            
    state_cur->setX(state_start->getX());
    state_cur->setY(state_start->getY());
    state_cur->setYaw(state_start->getYaw());
    
    // Circles around the start state until a state with different validity has been found
    // TODO If everything is invalid it will fail and return 0.
    // TODO Not very omptimized yet..
    int stop = std::max(mpTravGrid->getCellSizeX(),mpTravGrid->getCellSizeY());
    for(int i=1; i<stop; i++) {
        int dir = i%2 ? 1 : -1;
        
        for(int x=0; x < i; x++) {
            state_cur->setX(state_cur->getX()+dir);

            state_cur_valid = isValid(state_cur);
            if(state_cur_valid != state_start_valid) {
                double dist = sqrt(pow(state_start->getX()-state_cur->getX(), 2) + 
                        pow(state_start->getY()-state_cur->getY(), 2));
//                printf("Start (%s) (%4.2f, %4.2f) to (%4.2f, %4.2f), dist: %4.2f\n",
//state_start_valid ? "valid" : "not valid", state_start->getX(), state_start->getY(),
//state_cur->getX(), state_cur->getY(), dist);
                mpSpaceInformation->freeState(state_cur_mem);
                return state_start_valid ? dist : -dist; 
            }
        }
        
        for(int y=0; y < i ;y++) {
            state_cur->setY(state_cur->getY()+dir);

            state_cur_valid = isValid(state_cur);
            if(state_cur_valid != state_start_valid) {
                double dist = sqrt(pow(state_start->getX()-state_cur->getX(), 2) + 
                        pow(state_start->getY()-state_cur->getY(), 2));
                mpSpaceInformation->freeState(state_cur_mem);
                return state_start_valid ? dist : -dist;
            }
        }    
    }
    mpSpaceInformation->freeState(state_cur_mem);
    return 0.0;
}

/** 
    * Report the distance to the nearest invalid state when starting from \e state, and if possible,
    * also specify a valid state \e validState in the direction that moves away from the colliding
    * state. The \e validStateAvailable flag is set to true if \e validState is updated. 
    */
double TravMapValidator::clearance(const ompl::base::State *state, ompl::base::State*
validState, bool &validStateAvailable) const
{
    validStateAvailable = false;
    return clearance(state);
}

} // end namespace global_path_planner

