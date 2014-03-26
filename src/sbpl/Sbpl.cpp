#include "Sbpl.hpp"

namespace global_path_planner
{

Sbpl::Sbpl() : GlobalPathPlanner() {
}
 
bool Sbpl::initialize() { 
    return true;
}

bool Sbpl::solve(double time) {
    return true;
}
    
bool Sbpl::fillPath(std::vector<base::samples::RigidBodyState>& path) {
    return true;
}

} // namespace global_path_planner
