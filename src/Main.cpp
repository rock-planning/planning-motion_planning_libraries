#include <motion_planning_libraries/ompl/Ompl.hpp>
#include <motion_planning_libraries/sbpl/Sbpl.hpp>

#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    //motion_planning_libraries::Ompl ompl_planner;
    
    using namespace motion_planning_libraries;
    
    ConfigSBPL conf;
    std::string path_env(getenv ("AUTOPROJ_PROJECT_BASE"));
    path_env += "/external/sbpl/env_examples/env2.cfg";
    conf.mEnvFile = path_env;
    
    std::string path_primitives(getenv ("AUTOPROJ_PROJECT_BASE"));
    path_primitives += "/external/sbpl/matlab/mprim";
    conf.mMotionPrimitivesFile = path_primitives;
    
    Sbpl sbpl(conf);
    
    return 0;
}
