#include <stdlib.h>
#include <stdio.h>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>
#include <motion_planning_libraries/Helpers.hpp>

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

int main(int argc, char** argv)
{
    using namespace motion_planning_libraries;
    
    // Create the trav map.
    envire::Environment* env = new  envire::Environment();
    envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(100, 100, 0.1, 0.1);
    boost::shared_ptr<TravData> trav_data = boost::shared_ptr<TravData>(new TravData(
            trav->getGridData(envire::TraversabilityGrid::TRAVERSABILITY)));
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
    conf.mSearchUntilFirstSolution = true;
    
    // Draw a rectangle in the center 
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    
    // x,y,theta,width,length
    calc.setFootprint(50, 50, 0, 10, 10);
    calc.setValue(1); // obstacle
    
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
     
    return 0;
}
