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
        sbpl_cost = SBPL_MAX_COST - (int)(driveability * (double)SBPL_MAX_COST) + 1.0;
        *sbpl_map_p = sbpl_cost;
    }
    
    mTravInfos.setTravInfo(trav_grid);
}

bool Sbpl::sbplMapToTravGrid(envire::TraversabilityGrid** trav_grid, 
        envire::FrameNode** frame_node = NULL)
{
    
    if(!mTravInfos.isDataValid()) {
        LOG_WARN("Internal trav data is not valid, SBPL trav map cannot be created");
        return false;
    }
    
    // Initializing exploreMap that is going to be dumped.
    *trav_grid = new envire::TraversabilityGrid(
            mTravInfos.cellSizeX, mTravInfos.cellSizeY, 
            mTravInfos.scaleX, mTravInfos.scaleY, 
            mTravInfos.offsetX, mTravInfos.offsetY);
    (*trav_grid)->setTraversabilityClass(0, envire::TraversabilityClass (1.0)); // unknown
    (*trav_grid)->setTraversabilityClass(1, envire::TraversabilityClass (0.0)); // obstacle
    (*trav_grid)->setTraversabilityClass(2, envire::TraversabilityClass (0.5)); // robot cells
    
    envire::TraversabilityGrid::ArrayType& trav_array = (*trav_grid)->getGridData();
    
    // Copy SBPL map (just the obstacles) into the trav grid.
    for(size_t y = 0; y < mTravInfos.cellSizeY; y++){
        for (size_t x = 0; x < mTravInfos.cellSizeX; x++){
            (*trav_grid)->setProbability(1.0, x, y);
            if(mpSBPLMapData[x+y*mTravInfos.cellSizeX] == SBPL_MAX_COST + 1) {
                trav_array[y][x] = 1; // obstacle
            }
        } 
    }
    
    // Add the passed cells. Used to visualize the robot rectangle which 
    // is used for collision checking by SBPL.
    std::set< std::pair<int, int> >::iterator it = mAllCheckedCells.begin();
    int x_i=0, y_i=0;
    for(; it != mAllCheckedCells.end(); it++) {
        x_i = it->first;
        y_i = it->second;
        unsigned int pos = x_i + y_i * mTravInfos.cellSizeX;
        if(pos < mTravInfos.cellSizeX * mTravInfos.cellSizeY) {
            trav_array[y_i][x_i] = 2; // robot cells.
        } else {
            LOG_WARN("Cell position (%d,%d) lies outside of the map", x_i, y_i);
        }
    }
    
    // Creates copy of the frame node.
    *frame_node = new envire::FrameNode(*mTravInfos.frameNode);
    
    return true;
}

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
    LOG_INFO("Current epsilon is %4.2f", mEpsilon);
    return (mEpsilon == 1.0);
}

bool Sbpl::addLastCheckedCells() {
    boost::shared_ptr<EnvironmentNAVXYTHETALATTICE> env_sbpl =
        boost::dynamic_pointer_cast<EnvironmentNAVXYTHETALATTICE>(mpSBPLEnv);  
    
    if(env_sbpl == NULL) {
        LOG_WARN("No SBL EnvironmentNAVXYTHETALATTICE environment seems to be used, last cells could not be added");
        return false;
    }
        
    std::vector<sbpl_2Dcell_t> last_checked_cells = env_sbpl->GetCheckedCells();
    std::vector<sbpl_2Dcell_t>::iterator it = last_checked_cells.begin();
    for(; it != last_checked_cells.end(); it++) {
        mAllCheckedCells.insert(std::pair<int,int>(it->x, it->y));
    }
    return true;
}

} // namespace motion_planning_libraries
