#include <stdlib.h>
#include <stdio.h>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>
#include <motion_planning_libraries/Helpers.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

int main(int argc, char** argv)
{
    using namespace motion_planning_libraries;
    
    struct MotionPrimitivesConfig config;
    config.mSpeeds.mSpeedForward = 1.0;
    config.mSpeeds.mSpeedBackward = 0.8;
    config.mSpeeds.mSpeedLateral = 0.0;
    config.mSpeeds.mSpeedTurn = 0.4;
    config.mSpeeds.mSpeedPointTurn = 0.0;
    
    config.mSpeeds.mMultiplierForward = 1;
    config.mSpeeds.mMultiplierBackward = 5;
    config.mSpeeds.mMultiplierLateral = 10;
    config.mSpeeds.mMultiplierTurn = 2;
    config.mSpeeds.mMultiplierPointTurn = 8;
    
    config.mNumPrimPartition = 2;
    config.mNumPosesPerPrim = 10;
    config.mNumAngles = 16;
    
    config.mMapWidth = 100;
    config.mMapHeight = 100;
    config.mGridSize = 0.1;
    
    SbplMotionPrimitives mprims(config);
    mprims.createPrimitives();
    mprims.storeToFile("test.mprim");
    
    return 0;
    
    // Create the trav map.
    envire::Environment* env = new  envire::Environment();
    envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(100, 100, 0.1, 0.1);
    boost::shared_ptr<TravData> trav_data = boost::shared_ptr<TravData>(new TravData(
            trav->getGridData(envire::TraversabilityGrid::TRAVERSABILITY)));
    trav->setTraversabilityClass(0, envire::TraversabilityClass(0.5)); // driveability of unknown
    trav->setTraversabilityClass(1, envire::TraversabilityClass(0.0)); // driveability of obstacles
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
    std::string path_primitives(getenv ("AUTOPROJ_CURRENT_ROOT"));
    path_primitives += "/external/sbpl/matlab/mprim/pr2_10cm.mprim";
    conf.mSBPLMotionPrimitivesFile = path_primitives;
    conf.mSearchUntilFirstSolution = true;
    
    // Draw a rectangle in the center 
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    
    // x,y,theta,width,length
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    

    calc.setFootprintPoseInGrid(50, 50, 0);
    std::cout << "Footprint 1 " << (calc.isValid() ? "valid" : "not valid") << std::endl;
    
    calc.setFootprintPoseInGrid(55, 55, 0);
    std::cout << "Footprint 2 " << (calc.isValid() ? "valid" : "not valid") << std::endl;
    
    calc.setFootprintPoseInGrid(60, 60, 0);
    std::cout << "Footprint 3 " << (calc.isValid() ? "valid" : "not valid") << std::endl;
    
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

    sbpl.getTrajectoryInWorld();
     
    return 0;
}
