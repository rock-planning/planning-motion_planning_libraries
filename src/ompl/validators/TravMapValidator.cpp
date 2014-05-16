#include "TravMapValidator.hpp"

#include <ompl/base/SpaceInformation.h>
#include <base/Logging.hpp>

namespace motion_planning_libraries
{

TravMapValidator::TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            size_t grid_width, 
            size_t grid_height,
            boost::shared_ptr<TravData> grid_data,
            enum EnvType env_type) : 
            ompl::base::StateValidityChecker(si),
            mpSpaceInformation(si),
            mGridWidth(grid_width), 
            mGridHeight(grid_height),
            mpTravData(grid_data),
            mEnvType(env_type){
}

TravMapValidator::~TravMapValidator() {
}
    
bool TravMapValidator::isValid(const ompl::base::State* state) const
{
    
    int x_grid = 0;
    int y_grid = 0;
    
    switch(mEnvType) {
        case ENV_XY: {
            const ompl::base::RealVectorStateSpace::StateType* state_rv = 
                    state->as<ompl::base::RealVectorStateSpace::StateType>();
            x_grid = (int)state_rv->values[0];
            y_grid = (int)state_rv->values[1];
            break;
        }
        case ENV_XYTHETA: {
            const ompl::base::SE2StateSpace::StateType* state_se2 = 
                    state->as<ompl::base::SE2StateSpace::StateType>();
            x_grid = (int)state_se2->getX();
            y_grid = (int)state_se2->getY();
            break;
        }
    }
   
    // Check borders.
    if(     x_grid < 0 || x_grid >= (int)mGridWidth ||
            y_grid < 0 || y_grid >= (int)mGridHeight) {
        LOG_DEBUG("State (%d,%d) is invalid (not within the grid)", x_grid, y_grid);
        return false;
    }   
    
    // Check obstacle.
    if((*mpTravData)[y_grid][x_grid] == envire::SimpleTraversability::CLASS_OBSTACLE) {
        LOG_DEBUG("State (%d,%d) is invalid (lies on an obstacle)", x_grid, y_grid);
        return false;
    }
    
    return true;
}

double TravMapValidator::clearance(const ompl::base::State* state) const
{
    // Not used currently (and needs adaption regarding the different environments).
#if 0
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
    int stop = std::max(mGridWidth, mGridHeight);
    for(int i=1; i<stop; i++) {
        int dir = i%2 ? 1 : -1;
        
        for(int x=0; x < i; x++) {
            state_cur->setX(state_cur->getX()+dir);

            state_cur_valid = isValid(state_cur);
            if(state_cur_valid != state_start_valid) {
                double dist = sqrt(pow(state_start->getX()-state_cur->getX(), 2) + 
                        pow(state_start->getY()-state_cur->getY(), 2));
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
#endif
    return 0.0;
}

double TravMapValidator::clearance(const ompl::base::State *state, ompl::base::State* 
        validState, bool &validStateAvailable) const
{
    validStateAvailable = false;
    return clearance(state);
}

} // end namespace motion_planning_libraries

