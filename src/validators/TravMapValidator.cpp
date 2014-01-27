#include "TravMapValidator.hpp"

namespace global_path_planner
{

bool TravMapValidator::setTravGrid(envire::Environment& env, std::string trav_map_id) {
    typedef envire::TraversabilityGrid e_trav;

    // Extract traversability map from evironment.
    e_trav* trav_map = env.getItem< e_trav >(trav_map_id).get();
    if(trav_map) {
        mTravGrid = *trav_map;
        return true;
    }
    
    LOG_INFO("No traversability map with id '%s' available, first trav map will be used", 
            trav_map_id.c_str());
          
    std::vector<e_trav*> maps = env.getItems<e_trav>();
    if(maps.size() < 1) {
        LOG_WARN("Environment does not contain any traversability grids");
        return false;
    } else {
        std::vector<e_trav*>::iterator it = maps.begin();
        mTravGrid = **it;
        LOG_INFO("Traversability map '%s' wil be used", (*it)->getUniqueId().c_str());
    }

    return true;
}

} // end namespace global_path_planner

