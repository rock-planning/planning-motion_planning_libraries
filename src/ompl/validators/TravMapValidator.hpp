#ifndef _TRAV_MAP_VALIDATOR_HPP_
#define _TRAV_MAP_VALIDATOR_HPP_

#include <vector>
#include <map>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>

namespace envire {
class TraversabilityGrid;
}

namespace global_path_planner
{

class TravMapValidator :  public ompl::base::StateValidityChecker {
 
 private:
    envire::TraversabilityGrid* mpTravGrid;
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    envire::TraversabilityGrid::ArrayType* mpTravData;
    mutable std::map<double, base::Waypoint> mSamples;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            envire::TraversabilityGrid* trav_grid) : 
            ompl::base::StateValidityChecker(si),
            mpTravGrid(trav_grid),
            mpSpaceInformation(si),
            mSamples() {
        mpTravData = new envire::TraversabilityGrid::ArrayType(mpTravGrid->getGridData
                (envire::TraversabilityGrid::TRAVERSABILITY));
    }
    
    ~TravMapValidator() {
        delete mpTravData;
    }
    
    bool isValid(const ompl::base::State* state) const;
    
    // TODO Implement clearance methods. 
    
    /** 
     * Report the distance to the nearest invalid state when starting from \e state. If the distance is
     * negative, the value of clearance is the penetration depth.
     */
    double clearance(const ompl::base::State* state) const;

    /** 
     * Report the distance to the nearest invalid state when starting from \e state, and if possible,
     * also specify a valid state \e validState in the direction that moves away from the colliding
     * state. The \e validStateAvailable flag is set to true if \e validState is updated. 
     */
    double clearance(const ompl::base::State *state, ompl::base::State* validState, bool &validStateAvailable) const;

    std::vector<base::Waypoint> getSamples() {
        std::vector<base::Waypoint> vec;
        std::map<double, base::Waypoint>::iterator it = mSamples.begin();
        for(;it != mSamples.end(); it++) {
            vec.push_back(it->second);
        }
        return vec;
    }    
};

} // end namespace global_path_planner

#endif
