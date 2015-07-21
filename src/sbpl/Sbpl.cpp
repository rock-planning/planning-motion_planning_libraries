#include "Sbpl.hpp"

#include <exception>

#include <envire/operators/SimpleTraversability.hpp>

#include <sbpl/sbpl_exception.h>

namespace motion_planning_libraries
{

// PUBLIC
Sbpl::Sbpl(Config config) : AbstractMotionPlanningLibrary(config),
        mpSBPLEnv(),
        mpSBPLPlanner(),
        mSBPLWaypointIDs(),
        mpSBPLMapData(NULL),
        mSBPLNumElementsMap(0),
        mLastSolutionCost(0),
        mStartGrid(),
        mGoalGrid(),
        mEpsilon(0.0) {
            
    LOG_DEBUG("SBPL constructor");
}

bool Sbpl::solve(double time) {
    
    LOG_DEBUG("SBPL solve()");
    
    mSBPLWaypointIDs.clear();
    
    bool ret = false;
    try {
        // Current conclusion: Better not touch each planners epsilon.
        ret = mpSBPLPlanner->replan(time, &mSBPLWaypointIDs, &mLastSolutionCost);
        mPathCost = mLastSolutionCost;
        mEpsilon = mpSBPLPlanner->get_solution_eps();
    } catch (...) {
        LOG_ERROR("Replanning failed");
        return false;
    }
    
    if(ret) {
        LOG_INFO("Found solution contains %d waypoints", mSBPLWaypointIDs.size());
        return true;
    } else {
        return false;
    }
}

void Sbpl::createSBPLMap(envire::TraversabilityGrid* trav_grid,
        boost::shared_ptr<TravData> trav_data) {
    
    LOG_DEBUG("SBPL createSBPLMap");
    
    // Create a new sbpl map if it has not been created yet or the number 
    // of elements have changed.
    if(mpSBPLMapData != NULL) {
        if(trav_data->num_elements() != mSBPLNumElementsMap) {
            free(mpSBPLMapData);
            mpSBPLMapData = NULL;
            
            mpSBPLMapData = (unsigned char*)calloc(trav_data->num_elements(), sizeof(unsigned char));
            mSBPLNumElementsMap = trav_data->num_elements();
        } else {
            memset(mpSBPLMapData, 0, mSBPLNumElementsMap);
        }
    } else {
        mpSBPLMapData = (unsigned char*)calloc(trav_data->num_elements(), sizeof(unsigned char));
        mSBPLNumElementsMap = trav_data->num_elements();
    }
      
    // Adds the costs to the sbpl map using the driveability value of the traversability classes.
    double driveability = 0.0;
    unsigned char sbpl_cost = 0;
    unsigned char* sbpl_map_p = mpSBPLMapData;
    uint8_t* stop_p = trav_data->origin() + trav_data->num_elements();
    
    // The calculated costs of unknown areas (mean value of all grids)
    // is set to a close-to-obstacle-cost to prevent using unknown areas.
    std::vector <envire::TraversabilityClass > trav_classes = trav_grid->getTraversabilityClasses();
    double highest_driveability = 0.0;
    double last_highest_driveability = 0.0;
    for(unsigned int i=0; i<trav_classes.size(); i++) {
        double driveability = (trav_grid->getTraversabilityClass(i)).getDrivability();
        if(driveability > highest_driveability) {
            last_highest_driveability = highest_driveability; 
            highest_driveability = driveability;
        }
    }
    
    LOG_WARN("Driveability of unknown areas is increased to %4.2f in SBPL", last_highest_driveability);
    for(uint8_t* p = trav_data->origin(); p < stop_p; p++, sbpl_map_p++) {
        uint8_t class_value = *p;
        if(class_value == envire::SimpleTraversability::CLASS_UNKNOWN) {
            driveability = last_highest_driveability;
        } else {
            driveability = (trav_grid->getTraversabilityClass(class_value)).getDrivability();
        }
        sbpl_cost = SBPL_MAX_COST - (int)(driveability * (double)SBPL_MAX_COST + 0.5) + 1.0;
        *sbpl_map_p = sbpl_cost;
    }
}

// \todo "Check whether x == length and y == width are correct."
std::vector<sbpl_2Dpt_t> Sbpl::createFootprint(double robot_width, double robot_length) {

    LOG_DEBUG("SBPL createFootprint");
    
    double half_robot_width = robot_width / 2.0;
    double half_robot_length = robot_length / 2.0;
    std::vector<sbpl_2Dpt_t> footprint;
    
    sbpl_2Dpt_t pt_m;
    pt_m.y = -half_robot_width;
    pt_m.x = -half_robot_length;
    footprint.push_back(pt_m);
    pt_m.y = half_robot_width;
    pt_m.x = -half_robot_length;
    footprint.push_back(pt_m);
    pt_m.y = half_robot_width;
    pt_m.x = half_robot_length;
    footprint.push_back(pt_m);
    pt_m.y = -half_robot_width;
    pt_m.x = half_robot_length;
    footprint.push_back(pt_m);
    
    return footprint;
}

bool Sbpl::foundFinalSolution() {
    return (mEpsilon == 1.0);
}

} // namespace motion_planning_libraries
