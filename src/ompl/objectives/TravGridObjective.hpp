#ifndef _OBJECTIVE_TRAV_GRID_HPP_
#define _OBJECTIVE_TRAV_GRID_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <envire/maps/TraversabilityGrid.hpp>
#include <base/Logging.hpp>

#include <motion_planning_libraries/Config.hpp>
#include <motion_planning_libraries/ompl/spaces/SherpaStateSpace.hpp>

namespace motion_planning_libraries
{
    
typedef envire::TraversabilityGrid::ArrayType TravData;

/**
 * Using the costs of the trav grid. 
 */
class TravGridObjective :  public ompl::base::StateCostIntegralObjective {

 public:   
     static const unsigned char OMPL_MAX_COST = 100;
     static const double TIME_TO_ADAPT_FOOTPRINT = 40; // Time to move from min to max in sec.
     static const double PENALTY_TO_ADAPT_FOOTPRINT = 20;
    
 private:
     envire::TraversabilityGrid* mpTravGrid; // To request the driveability values.
     boost::shared_ptr<TravData> mpTravData;
     Config mConfig;
        
 public:
    /**
     * \param enable_motion_cost_interpolation Defines if only start and end state
     * are used for cost calculations or smaller intermediate steps. By default false.
     * Not required for correct collision detection.
     * TODO Currently only the cost of the center of the robot is used.
     */
    TravGridObjective(const ompl::base::SpaceInformationPtr& si, 
                        bool enable_motion_cost_interpolation,
                        Config config) : 
                ompl::base::StateCostIntegralObjective(si, enable_motion_cost_interpolation), 
                mpTravGrid(NULL), 
                mpTravData(),
                mConfig(config) {
    }     
     
    TravGridObjective(const ompl::base::SpaceInformationPtr& si, 
                        bool enable_motion_cost_interpolation,
                        envire::TraversabilityGrid* trav_grid,
                        boost::shared_ptr<TravData> trav_data,
                        Config config) : 
                ompl::base::StateCostIntegralObjective(si, enable_motion_cost_interpolation), 
                mpTravGrid(trav_grid), 
                mpTravData(trav_data),
                mConfig(config) {
    }
    
    ~TravGridObjective() {
    }
    
    void setTravGrid(envire::TraversabilityGrid* trav_grid, boost::shared_ptr<TravData> trav_data) {
        mpTravGrid = trav_grid;
        mpTravData = trav_data;
    }
    
    ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        if(mpTravGrid == NULL) {
            throw std::runtime_error("TravGridObjective: No traversability grid available");
        }
    
        double x = 0, y = 0;
        int footprint_class = 0;
        
        switch(mConfig.mEnvType) {
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
            case ENV_SHERPA: {
                const SherpaStateSpace::StateType* state_sherpa = 
                        s->as<SherpaStateSpace::StateType>();
                x = state_sherpa->getX();
                y = state_sherpa->getY();
                footprint_class = state_sherpa->getFootprintClass();
                break;
            }
            default: {
                throw std::runtime_error("TravGridObjective received an unknown environment");
                break;
            }
        }
        
        // TODO Assuming: only valid states are passed?
        if(x < 0 || x >= mpTravGrid->getCellSizeX() || 
                y < 0 || y >= mpTravGrid->getCellSizeY()) {
            LOG_WARN("Invalid state (%4.2f, %4.2f) has been passed and will be ignored", 
                   x, y); 
            throw std::runtime_error("Invalid state received");
            //return ompl::base::Cost(0);
        }
    
        // Estimate time to traverse the cell using forward speed and driveability.
        double class_value = (double)(*mpTravData)[y][x];
        double driveability = (mpTravGrid->getTraversabilityClass(class_value)).getDrivability();
        double cost = 0;
        if(driveability == 0 || mConfig.mRobotForwardVelocity == 0) {
            cost = std::numeric_limits<double>::max();
        } else {
            // Calculate time to traverse the cell. Driveability of 1.0 means, that the cell can
            // be traversed with full speed.
            cost = (mpTravGrid->getScaleX() / mConfig.mRobotForwardVelocity) /  driveability;
            // Increases cost regarding the footprint. Max footprint means full speed,
            // min footprint increases the cost by the number of footprint classes.
            if(mConfig.mEnvType == ENV_SHERPA) {
                cost /= (footprint_class+1) / ((double)mConfig.mNumFootprintClasses+1);
            }
        }
        //return ompl::base::Cost(OMPL_MAX_COST - (driveability * (double)OMPL_MAX_COST + 0.5));        
        return ompl::base::Cost(cost);
    }
    
    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const {
        // Uses the base motionCost() to calculate the cost to traverse from s1 to s2 
        // (mean costs of s1 and s2 and the distance (x,y,theta/2.0) between the states,
        // uses the above stateCost() implementation).
        ompl::base::Cost cost = ompl::base::StateCostIntegralObjective::motionCost(s1, s2);
         
        switch(mConfig.mEnvType) {
                
            // Adds cost for changing the footprint.
            case ENV_SHERPA: {
                // Add high additional costs if the footprint of the system have been changed.
                const SherpaStateSpace::StateType* st_s1 = s1->as<SherpaStateSpace::StateType>();
                const SherpaStateSpace::StateType* st_s2 = s2->as<SherpaStateSpace::StateType>();
                
                double footprint_cost = (abs(st_s1->getFootprintClass() - st_s2->getFootprintClass()) / 
                        (double)mConfig.mNumFootprintClasses) * 
                        mConfig.mTimeToAdaptFootprint;
                if(footprint_cost > 0) {
                    footprint_cost += mConfig.mAdaptFootprintPenalty;
                    
                    //std::cout << "Moving from state (" << st_s1->getX() << "," << st_s1->getY() << ") fp class " << st_s1->getFootprintClass() << 
                    //        " to (" << st_s2->getX() << "," << st_s2->getY() << ") fp class " << st_s2->getFootprintClass() << 
                    //        " changes the footprint, increases cost from " << cost.v << " to " << cost.v + footprint_cost << std::endl;
                }
   
                cost.v += footprint_cost;

                break;
            }
            default: {
                break;
            }
        }
        return cost;
    }
};

} // end namespace motion_planning_libraries

#endif
