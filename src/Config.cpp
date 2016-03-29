#include "Config.hpp"

namespace motion_planning_libraries {

std::string MovementTypesString[] = {
    "Undefined", 
    "Forward", 
    "Backward", 
    "Forward Turn", 
    "Backward Turn", 
    "Pointturn", 
    "Lateral"
};  

std::string MplErrorsString[] = {
    "No error", 
    "Undefined error", 
    "Missing: Start", 
    "Missing: Goal", 
    "Missing: Trav-Map", 
    "Missing: Start, Goal", 
    "Missing: Start, Trav-Map", 
    "Missing: Goal, Trav-Map", 
    "Not used",
    "Missing: Start, Goal, Trav-Map", 
    "Start lies on an obstacle", 
    "Goal lies on an obstacle", 
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Start and Goal lie on an obstacle", 
    "Planning failed", 
    "Wrong state type (start/goal correct?)", 
    "Map initialization failed", 
    "Start/goal could not be set", 
    "Replanning not required"
}; 

} // end namespace motion_planning_libraries
