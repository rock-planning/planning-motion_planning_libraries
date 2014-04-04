#include "Sbpl.hpp"

#include <sbpl/discrete_space_information/environment_navxythetamlevlat.h>

namespace global_path_planner
{

// PUBLIC
Sbpl::Sbpl() : GlobalPathPlanner() {
}

// PROTECTED
bool Sbpl::initialize() { 
    mpEnv = boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT>(new EnvironmentNAVXYTHETAMLEVLAT());    
    // TODO For SBPL the cells have to be quadrats.
    mpEnv->InitializeEnv(mpTravGrid->getCellSizeX(), mpTravGrid->getCellSizeY, 
            NULL, // initial map, use it 
            mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
            mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw(),
            0.1, 0.1, 0.1, // tolerance x,y,yaw, ignored
            createFootprint(mRobotWidth, mRobotHeight), mpTravGrid->getScaleX(),
            mRobotForwardVelocity,  mRobotRotationalVelocity,
            12, // cost threshold
            NULL); // motion priimitives file
    
    return true;
}

bool Sbpl::solve(double time) {
    return true;
}
    
bool Sbpl::fillPath(std::vector<base::samples::RigidBodyState>& path) {
    return true;
}

// PRIVATE
vector<sbpl_2Dpt_t> Sbpl::createFootprint(double robot_width, double robot_height) {
    half_robot_width = robot_width / 2.0;
    half_robot_height = robot_height / 2.0;
    vector<sbpl_2Dpt_t> footprint;
    
    sbpl_2Dpt_t pt_m;
    pt_m.x = -half_robot_width;
    pt_m.y = -half_robot_height;
    footprint.push_back(pt_m);
    pt_m.x = half_robot_width;
    pt_m.y = -half_robot_height;
    footprint.push_back(pt_m);
    pt_m.x = half_robot_width;
    pt_m.y = half_robot_height;
    footprint.push_back(pt_m);
    pt_m.x = -half_robot_width;
    pt_m.y = half_robot_height;
    footprint.push_back(pt_m);
    
    return footprint;
}

} // namespace global_path_planner
