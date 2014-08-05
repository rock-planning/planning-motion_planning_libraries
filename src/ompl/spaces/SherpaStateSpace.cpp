#include "SherpaStateSpace.hpp"

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>

namespace motion_planning_libraries {

ompl::base::State* SherpaStateSpace::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void SherpaStateSpace::freeState(ompl::base::State *state) const
{
    ompl::base::CompoundStateSpace::freeState(state);
}

void SherpaStateSpace::registerProjections(void)
{
    class SherpaDefaultProjection : public ompl::base::ProjectionEvaluator
    {
    public:

        SherpaDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        virtual unsigned int getDimension(void) const
        {
            return 2;
        }

        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(2);
            bounds_ = space_->as<ompl::base::SE2StateSpace>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
        {
            memcpy(&projection(0), state->as<ompl::base::SE2StateSpace::StateType>()->
                    as<ompl::base::RealVectorStateSpace::StateType>(0)->values, 2 * sizeof(double));
        }
    };

    registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(dynamic_cast<ompl::base::ProjectionEvaluator*>(new SherpaDefaultProjection(this))));
}

} // end namespace motion_planning_libraries
