#include <stdlib.h>
#include <stdio.h>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

int main(int argc, char** argv)
{
    using namespace motion_planning_libraries;
    
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
    
    // Create conf file.
    Config conf;
    std::string path_primitives(getenv ("AUTOPROJ_PROJECT_BASE"));
    path_primitives += "/external/sbpl/matlab/mprim/pr2_10cm.mprim";
    conf.mSBPLMotionPrimitivesFile = path_primitives;
    conf.mSearchUntilFirstSolution = false;
    
    // SBPL
    std::cout << std::endl << "SBPL XYTHETA PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_SBPL;
    conf.mEnvType = ENV_XYTHETA;
    
    MotionPlanningLibraries sbpl(conf);
    sbpl.setTravGrid(env, "/trav_map");
    sbpl.setStartState(State(rbs_start));
    sbpl.setGoalState(State(rbs_goal));

    if(sbpl.plan(10)) {
        std::cout << "SBPL problem solved" << std::endl;
        sbpl.printPathInWorld();
    } else {
        std::cout << "SBPL problem could not be solved" << std::endl;
    }
     
    // OMPL
    std::cout << std::endl << "OMPL XY PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_OMPL;
    conf.mEnvType = ENV_XY;
    
    MotionPlanningLibraries ompl(conf);
    ompl.setTravGrid(env, "/trav_map");
    ompl.setStartState(rbs_start);
    ompl.setGoalState(rbs_goal);
    
    if(ompl.plan(10)) {
        std::cout << "OMPL problem solved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be solved" << std::endl;
    }
    
    std::cout << std::endl << "OMPL XY PLANNING IMPROVE" << std::endl;
    if(ompl.plan(10)) {
        std::cout << "OMPL problem improved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be improved" << std::endl;
    }
    
    // OMPL arm motion planning
    std::cout << std::endl << "OMPL ARM PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_OMPL;
    conf.mEnvType = ENV_ARM;
    conf.setJoints(4);
    
    MotionPlanningLibraries ompl_arm(conf);
    std::vector<double> joint_angles_start, joint_angles_goal;
    joint_angles_start.push_back(M_PI);
    joint_angles_goal.push_back(0.0);
    for(int i=0; i<3; ++i) {
        joint_angles_start.push_back(0.0);
        joint_angles_goal.push_back(0.0);
    }
    State arm_start(joint_angles_start);
    State arm_goal(joint_angles_goal);
     
    ompl_arm.setStartState(arm_start);
    ompl_arm.setGoalState(arm_goal);
    
    if(ompl_arm.plan(10)) {
        std::cout << "OMPL ARM problem solved" << std::endl;
        std::vector<struct State> states = ompl_arm.getPath();
        for(int i=0; i<states.size(); ++i) {
            std::cout << states[i].getString() << std::endl;
        }
    } else {
        std::cout << "OMPL ARM problem could not be solved" << std::endl;
    }

    return 0;
}
