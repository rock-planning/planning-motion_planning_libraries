#ifndef _OBJECTIVE_TRAV_GRID_HPP_
#define _OBJECTIVE_TRAV_GRID_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <envire/maps/TraversabilityGrid.hpp>

namespace global_path_planner
{
    
typedef envire::TraversabilityGrid::ArrayType TravData;

/**
 * Using the costs of the trav grid. 
 */
class TravGridObjective :  public ompl::base::StateCostIntegralObjective {

 private:
     boost::shared_ptr<TravData> mpTravData;
     int mGridWidth;
     int mGridHeight;
        
 public:
    TravGridObjective(const ompl::base::SpaceInformationPtr& si, 
            boost::shared_ptr<TravData> trav_data, int grid_width, int grid_height) : 
            ompl::base::StateCostIntegralObjective(si, true), mpTravData(trav_data),
            mGridWidth(grid_width), mGridHeight(grid_height) {
    }
    
    ~TravGridObjective() {
    }
    
    ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        const ompl::base::SE2StateSpace::StateType* state_se2 = 
                s->as<ompl::base::SE2StateSpace::StateType>();
        
        // TODO Assuming: only valid states are passed?
        if(state_se2->getX() < 0 || state_se2->getX() >= mGridWidth || 
                state_se2->getY() < 0 || state_se2->getY() >= mGridHeight) {
            LOG_WARN("Invalid state (%4.2f, %4.2f) has been passed and will be ignored", 
                   state_se2->getX(), state_se2->getY()); 
            throw std::runtime_error("Invalid state received");
            //return ompl::base::Cost(0);
        }
    
        // TODO Use real costs and a terrain class configuration file.
        // Assuming 12 classes and using each of the classes as it costs: 13 - class
        // 0: unkown, 1: obstacle
        double grid_class = (double)(*mpTravData)[state_se2->getY()][state_se2->getX()];
        if(grid_class == 1) { // obstacle
            return ompl::base::Cost(600);
        }
        if(grid_class == 0) { // unknown
            return ompl::base::Cost(6.0);
        }
        return ompl::base::Cost(13 - grid_class);        
    }
};

} // end namespace global_path_planner

#endif
