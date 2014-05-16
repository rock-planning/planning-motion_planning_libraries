#include <stdlib.h>
#include <stdio.h>

#include <motion_planning_libraries/ompl/Ompl.hpp>
#include <motion_planning_libraries/sbpl/Sbpl.hpp>

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

int main(int argc, char** argv)
{
    using namespace motion_planning_libraries;
    
    // Create conf file.
    Config conf;
    std::string path_primitives(getenv ("AUTOPROJ_PROJECT_BASE"));
    path_primitives += "/external/sbpl/matlab/mprim/pr2_10cm.mprim";
    conf.mSBPLMotionPrimitivesFile = path_primitives;
    conf.mEnvType = ENV_XYTHETA;
    conf.mSearchUntilFirstSolution = false;
    
    // Create the trav map.
    envire::Environment* env = new  envire::Environment();
    envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(100, 100, 0.1, 0.1);
    trav->setTraversabilityClass(0, envire::TraversabilityClass(0.5)); // driveability of unknown
    trav->setUniqueId("/trav_map");
    env->attachItem(trav);
    envire::FrameNode* frame_node = new envire::FrameNode();
    env->getRootNode()->addChild(frame_node);
    trav->setFrameNode(frame_node);
 
    // Create start and goal
    base::samples::RigidBodyState rbs_start;
    rbs_start.setPose(base::Pose(base::Position(1,1,0), base::Orientation::Identity()));
    base::samples::RigidBodyState rbs_goal;
    rbs_goal.setPose(base::Pose(base::Position(9,9,0), base::Orientation::Identity()));
    
    // SBPL in ENV_XYTHETA
    /*
    std::cout << std::endl << "SBPL PLANNING" << std::endl;
    Sbpl sbpl(conf);
    sbpl.setTravGrid(env, "/trav_map");
    sbpl.setStartPoseInWorld(rbs_start);
    sbpl.setGoalPoseInWorld(rbs_goal);

    if(sbpl.plan(10)) {
        std::cout << "SBPL problem (ENV_XYTHETA) solved" << std::endl;
    } else {
        std::cout << "SBPL problem could not be solved" << std::endl;
    }
    */
    
    // OMPL in ENV_XY
    std::cout << std::endl << "OMPL PLANNING" << std::endl;
    //conf.mEnvType = ENV_XY;
    Ompl ompl(conf);
    ompl.setTravGrid(env, "/trav_map");
    ompl.setStartPoseInWorld(rbs_start);
    ompl.setGoalPoseInWorld(rbs_goal);
    
    if(ompl.plan(10)) {
        std::cout << "OMPL problem (ENV_XY) solved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be solved" << std::endl;
    }

    return 0;
}
