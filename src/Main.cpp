#include <global_path_planner/ompl/Ompl.hpp>
#include <global_path_planner/sbpl/Sbpl.hpp>

#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    //global_path_planner::Ompl ompl_planner;
    
    using namespace global_path_planner;
    
    ConfigurationSBPL conf;
    std::string path_env(getenv ("AUTOPROJ_PROJECT_BASE"));
    path_env += "/external/sbpl/env_examples/env2.cfg";
    conf.mSBPLEnvFile = path_env;
    
    std::string path_primitives(getenv ("AUTOPROJ_PROJECT_BASE"));
    path_primitives += "/external/sbpl/matlab/mprim";
    conf.mSBPLMotionPrimitivesFile = path_primitives;
    
    Sbpl sbpl(conf);
    
    return 0;
}
