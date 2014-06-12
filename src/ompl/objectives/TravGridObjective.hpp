#ifndef _OBJECTIVE_TRAV_GRID_HPP_
#define _OBJECTIVE_TRAV_GRID_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <envire/maps/TraversabilityGrid.hpp>
#include <base/Logging.hpp>

#include <motion_planning_libraries/Config.hpp>

namespace motion_planning_libraries
{
    
typedef envire::TraversabilityGrid::ArrayType TravData;

/**
 * Using the costs of the trav grid. 
 */
class TravGridObjective :  public ompl::base::StateCostIntegralObjective {

 public:   
     static const unsigned char OMPL_MAX_COST = 100;
    
 private:
     envire::TraversabilityGrid* mpTravGrid; // To request the driveability values.
     boost::shared_ptr<TravData> mpTravData;
     int mGridWidth;
     int mGridHeight;
     enum EnvType mEnvType;
        
 public:
    TravGridObjective(const ompl::base::SpaceInformationPtr& si, 
                        envire::TraversabilityGrid* trav_grid,
                        boost::shared_ptr<TravData> trav_data, 
                        int grid_width, int grid_height,
                        enum EnvType env_type) : 
                ompl::base::StateCostIntegralObjective(si, true), 
                mpTravGrid(trav_grid), 
                mpTravData(trav_data),
                mGridWidth(grid_width), mGridHeight(grid_height), 
                mEnvType(env_type) {
    }
    
    ~TravGridObjective() {
    }
    
    ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        double x = 0, y = 0;
        
        switch(mEnvType) {
            case ENV_XY: {
                const ompl::base::RealVectorStateSpace::StateType* state_rv = 
                        s->as<ompl::base::RealVectorStateSpace::StateType>();
                x = state_rv->values[0];
                y = state_rv->values[1];
                break;
            }
            case ENV_XYTHETA: {
                const ompl::base::SE2StateSpace::StateType* state_se2 = 
                        s->as<ompl::base::SE2StateSpace::StateType>();
                x = state_se2->getX();
                y = state_se2->getY();
                break;
            }
            default: {
                throw std::runtime_error("TravMapValidator received an unknown environment");
                break;
            }
        }
        
        // TODO Assuming: only valid states are passed?
        if(x < 0 || x >= mGridWidth || 
                y < 0 || y >= mGridHeight) {
            LOG_WARN("Invalid state (%4.2f, %4.2f) has been passed and will be ignored", 
                   x, y); 
            throw std::runtime_error("Invalid state received");
            //return ompl::base::Cost(0);
        }
    
        // Uses the driveability which creates costs from 0 to OMPL_MAX_COST.
        double class_value = (double)(*mpTravData)[y][x];
        double driveability = (mpTravGrid->getTraversabilityClass(class_value)).getDrivability();
        return ompl::base::Cost(OMPL_MAX_COST - (driveability * (double)OMPL_MAX_COST + 0.5));        
    }
};

} // end namespace motion_planning_libraries

#endif
