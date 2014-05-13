#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>

#include <base/Logging.hpp>
#include <motion_planning_libraries/MotionPlanningLibraries.hpp>

#include <sbpl/utils/utils.h>
#include <sbpl/config.h> // here #define DEBUG 0, causes a lot of trouble
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/environment_nav2D.h>
#include <sbpl/discrete_space_information/environment_navxythetamlevlat.h>
#include <sbpl/planners/planner.h>
#undef DEBUG

namespace motion_planning_libraries
{

enum SbplEnv {SBPL_XY, SBPL_XYTHETA};
    
struct ConfigurationSBPL : public ConfigurationBase {
    ConfigurationSBPL() :
            mSBPLEnvType(SBPL_XY),
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true), 
            mSBPLSearchUntilFirstSolution(false) {
    }
    
    SbplEnv mSBPLEnvType;
    std::string mSBPLEnvFile;
    std::string mSBPLMotionPrimitivesFile;
    bool mSBPLForwardSearch;
    bool mSBPLSearchUntilFirstSolution;
};
    
/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 */
class Sbpl : public MotionPlanningLibraries
{      
private:
    // Driveability 0.0 to 1.0 will be mapped to SBPL_MAX_COST to 0 
    // with obstacle threshold of SBPL_MAX_COST.
    static const unsigned char SBPL_MAX_COST = 100;
    
    ConfigurationSBPL mConfigSBPL;
    
    boost::shared_ptr<DiscreteSpaceInformation> mpSBPLEnv;
    boost::shared_ptr<SBPLPlanner> mpSBPLPlanner;
    std::vector<int> mSBPLWaypointIDs;
    unsigned char* mpSBPLMapData;
    size_t mSBPLNumElementsMap;
    // Required to set start/goal in SBPL_XYTHETA 
    // (grid coordinates have to be converted back to meters) 
    double mScaleX, mScaleY; 
    
 public: 
    Sbpl(ConfigurationSBPL config_sbpl = ConfigurationSBPL());
    void mpEnv();
 
 protected:
    /**
     * 
     */
    virtual bool initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * 
     */
    virtual bool setStartGoal(int start_x, int start_y, double start_yaw, 
            int goal_x, int goal_y, double goal_yaw);
    
    /**
     * 
     */
    virtual bool solve(double time);
        
    /**
     * 
     */
    virtual bool fillPath(std::vector<base::samples::RigidBodyState>& path);
    
 private:        
    /**
     * Converts the trav map to a sbpl map using the driveability value.
     * Driveability 0.0 to 1.0 is mapped to costs 100 to 0 with obstacle threshold 100.
     */
    void createSBPLMap(boost::shared_ptr<TravData> trav_data);
    
    std::vector<sbpl_2Dpt_t> createFootprint(double robot_width, double robot_height);
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_HPP_
