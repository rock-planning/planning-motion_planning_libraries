#ifndef _OBJECTIVE_TRAV_GRID_HPP_
#define _OBJECTIVE_TRAV_GRID_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <envire/maps/TraversabilityGrid.hpp>

namespace global_path_planner
{

/**
 * Using the costs of the trav grid. 
 */
class TravGridObjective :  public ompl::base::StateCostIntegralObjective {

 private:
     envire::TraversabilityGrid* mpTravGrid;
     envire::TraversabilityGrid::ArrayType* mpTravData;
        
 public:
    TravGridObjective(const ompl::base::SpaceInformationPtr& si, 
            envire::TraversabilityGrid* trav_grid) : 
            ompl::base::StateCostIntegralObjective(si, true), mpTravGrid(trav_grid) {
        mpTravData = new envire::TraversabilityGrid::ArrayType(
                mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    }
    
    ~TravGridObjective() {
        delete mpTravData;
    }
    
    ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        // TODO Assuming: only valid states are passed?
        if(mpTravGrid == NULL) {
            LOG_WARN("No traversability grid available yet, cost 0 will be returned");
            return ompl::base::Cost(0.0);
        }
    
        // TODO Use real costs and a terrain class configuration file.
        // Assuming 12 classes and using each of the classes as it costs: 13 - class
        // 0: unkown, 1: obstacle
        const ompl::base::SE2StateSpace::StateType* state_se2 = 
            s->as<ompl::base::SE2StateSpace::StateType>();
        double grid_class = (double)(*mpTravData)[state_se2->getY()][state_se2->getY()];
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
