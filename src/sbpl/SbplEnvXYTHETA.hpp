#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_

#include "Sbpl.hpp"
#include "SbplMotionPrimitives.hpp"

namespace motion_planning_libraries
{
    
class SbplEnvXYTHETA : public Sbpl
{          
 protected:
    // Required to set start/goal in ENV_XYTHETA 
    // (grid coordinates have to be converted back to meters) 
    double mSBPLScaleX, mSBPLScaleY; 
    struct SbplMotionPrimitives* mPrims;
    // Used to store the local goal pose (x,y,theta) to add it to the end of the 
    // found intermediate path (last pose is not supported).
    base::Vector3d mGoalLocal;

 public: 
    SbplEnvXYTHETA(Config config = Config());
 
    /**
     * 
     */
    virtual bool initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * 
     */
    virtual bool setStartGoal(struct State start_state, struct State goal_state);
        
    virtual bool solve(double time);    
        
    /**
     * Currently just converts the found prim ids to x,y,theta.
     * It does not use ConvertStateIDPathintoXYThetaPath()
     * which would also add the intermediate points.
     */
    virtual bool fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid);
    
    inline struct SbplMotionPrimitives* getMotionPrimitives() {
        return mPrims;
    }
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_
