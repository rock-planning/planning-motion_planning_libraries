#include "TravMapValidator.hpp"

#include <ompl/base/SpaceInformation.h>

#include <base/Logging.hpp>

#include <motion_planning_libraries/ompl/spaces/SherpaStateSpace.hpp>

namespace motion_planning_libraries
{

TravMapValidator::TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            Config config) : 
            ompl::base::StateValidityChecker(si),
            mpSpaceInformation(si),
            mConfig(config), 
            mGridCalc() {
}

TravMapValidator::TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data,
            Config config) : 
            ompl::base::StateValidityChecker(si),
            mpSpaceInformation(si),
            mpTravGrid(trav_grid),
            mpTravData(grid_data),
            mConfig(config), 
            mGridCalc() {
    mGridCalc.setTravGrid(trav_grid, grid_data);
}

TravMapValidator::~TravMapValidator() {
}

void TravMapValidator::setTravGrid(envire::TraversabilityGrid* trav_grid, 
        boost::shared_ptr<TravData> trav_data) {
    mpTravGrid = trav_grid;
    mpTravData = trav_data;
    mGridCalc.setTravGrid(trav_grid, trav_data);
}
    
bool TravMapValidator::isValid(const ompl::base::State* state) const
{  
    if(mpTravGrid == NULL) {
        throw std::runtime_error("TravMapValidator: No traversability grid available");
    }

    switch(mConfig.mEnvType) {
        case ENV_XY: {
            int x_grid = 0;
            int y_grid = 0;

            const ompl::base::RealVectorStateSpace::StateType* state_rv = 
                    state->as<ompl::base::RealVectorStateSpace::StateType>();
            x_grid = (int)state_rv->values[0];
            y_grid = (int)state_rv->values[1];

            // Check borders.
            if(     x_grid < 0 || x_grid >= (int)mpTravGrid->getCellSizeX() ||
                    y_grid < 0 || y_grid >= (int)mpTravGrid->getCellSizeY()) {
                LOG_DEBUG("State (%d,%d) is invalid (not within the grid)", x_grid, y_grid);
                return false;
            }   

            // Check obstacle.
            double class_value = (double)(*mpTravData)[y_grid][x_grid];
            double driveability = (mpTravGrid->getTraversabilityClass(class_value)).getDrivability();
                
            if(driveability == 0.0) {
                LOG_DEBUG("State (%d,%d) is invalid (lies on an obstacle)", x_grid, y_grid);
                return false;
            }

            return true;
        }
        case ENV_XYTHETA: {
            const ompl::base::SE2StateSpace::StateType* state_se2 = 
                    state->as<ompl::base::SE2StateSpace::StateType>();
                
            // Get current state XYTHETA.    
            double x_grid = state_se2->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
            double y_grid = state_se2->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
            double yaw_grid = state_se2->as<ompl::base::SO2StateSpace::StateType>(1)->value;
            
            mGridCalc.setFootprint(x_grid, y_grid, yaw_grid, 
                    ceil(std::max(mConfig.mRobotLengthMinMax.first, mConfig.mRobotLengthMinMax.second) / 
                            (double)mpTravGrid->getScaleX()), 
                    ceil(std::max(mConfig.mRobotWidthMinMax.first, mConfig.mRobotWidthMinMax.second) / 
                            (double)mpTravGrid->getScaleY()));
                    
            return mGridCalc.isValid();
        }
        case ENV_SHERPA: {
            const SherpaStateSpace::StateType* state_sherpa = state->as<SherpaStateSpace::StateType>();
                
            // Get current state XYTHETA.    
            double x_grid = state_sherpa->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
            double y_grid = state_sherpa->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
            double yaw_grid = state_sherpa->as<ompl::base::SO2StateSpace::StateType>(1)->value;
            double length_factor = state_sherpa->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0];
            double width_factor = state_sherpa->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1];            
            
            // Calculate the current width/length.
            double length = mConfig.mRobotLengthMinMax.first + 
                    fabs(mConfig.mRobotLengthMinMax.second - mConfig.mRobotLengthMinMax.first) * length_factor;
            double width = mConfig.mRobotWidthMinMax.first + 
                    fabs(mConfig.mRobotWidthMinMax.second - mConfig.mRobotWidthMinMax.first) * width_factor;
            
            mGridCalc.setFootprint(x_grid, y_grid, yaw_grid, length, width);
                    
            return mGridCalc.isValid();
            
            break;
        }
        default: {
            throw std::runtime_error("TravMapValidator received an unknown environment");
        }
    }
   
}

} // end namespace motion_planning_libraries

