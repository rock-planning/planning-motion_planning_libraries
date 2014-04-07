#include "DiscreteSE2StateSpace.hpp"

#include <cstring>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

namespace global_path_planner
{

ompl::base::State* DiscreteSE2StateSpace::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void DiscreteSE2StateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

void DiscreteSE2StateSpace::registerProjections(void)
{
    class DiscreteSE2DefaultProjection : public ProjectionEvaluator
    {
    public:

        DiscreteSE2DefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        virtual unsigned int getDimension(void) const
        {
            return 2;
        }

        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(2);
            bounds_ = space_->as<SE2StateSpace>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            memcpy(&projection(0), state->as<DiscreteSE2StateSpace::StateType>()->as<DiscreteStateSpace::StateType>(0)->values, 2 * sizeof(double));
        }
    };

    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new SE2DefaultProjection(this))));
}

} // end namespace global_path_planner
