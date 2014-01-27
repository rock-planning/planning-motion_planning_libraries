#ifndef _GLOBAL_PATH_PLANNER_HPP_
#define _GLOBAL_PATH_PLANNER_HPP_

#include <base/samples/RigidBodyState.hpp>

#include <global_path_planner/validators/TravMapValidator.hpp>

namespace global_path_planner
{

class GlobalPathPlanner
{
 private:
    base::samples::RigidBodyState mStart, mGoal;
    TravMapValidator* mpTravMapValidator;
    
 public: 
    GlobalPathPlanner();
    ~GlobalPathPlanner();
    
    void init();
 
    inline void setStart(base::samples::RigidBodyState& start) {
        mStart = start;
    }
    
    inline base::samples::RigidBodyState getStart() const {
        return mStart;
    }
    
    inline void setGoal(base::samples::RigidBodyState& goal) {
        mGoal = goal;
    }
    
    inline base::samples::RigidBodyState getGoal() const {
        return mGoal;
    }  
    
    inline void setTravGrid(envire::TraversabilityGrid& trav_grid) {
        mpTravMapValidator->setTravGrid(trav_grid);
    }
    
    inline bool setTravGrid(envire::Environment& env, std::string trav_map_id) {
        return mpTravMapValidator->setTravGrid(env, trav_map_id);
    }
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_HPP_
