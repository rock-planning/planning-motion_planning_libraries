#ifndef _DISCRETE_SE2_STATE_SPACE_HPP_
#define _DISCRETE_SE2_STATE_SPACE_HPP_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"

namespace global_path_planner
{
/** \brief A state space representing SE(2) */
class DiscreteSE2StateSpace : public CompoundStateSpace
{
public:

    /** \brief A state in SE(2): (x, y, yaw) */
    class StateType : public CompoundStateSpace::StateType
    {
    public:
        StateType(void) : CompoundStateSpace::StateType()
        {
        }

        /** \brief Get the X component of the state */
        int getX(void) const
        {
            return as<DiscreteStateSpace::StateType>(0)->value;
        }

        /** \brief Get the Y component of the state */
        int getY(void) const
        {
            return as<DiscreteStateSpace::StateType>(1)->value;
        }

        /** \brief Get the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        int getYaw(void) const
        {
            return as<DiscreteStateSpace::StateType>(2)->value;
        }

        /** \brief Set the X component of the state */
        void setX(int x)
        {
            as<DiscreteStateSpace::StateType>(0)->value = x;
        }

        /** \brief Set the Y component of the state */
        void setY(int y)
        {
            as<DiscreteStateSpace::StateType>(1)->value = y;
        }

        /** \brief Set the X and Y components of the state */
        void setXY(double x, double y)
        {
            setX(x);
            setY(y);
        }

        /** \brief Set the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        void setYaw(double yaw)
        {
            as<DiscreteStateSpace::StateType>(2)->value = yaw;
        }

    };


    DiscreteSE2StateSpace(int width, int height) : CompoundStateSpace()
    {
        setName("DiscreteSE2" + getName());
        type_ = STATE_SPACE_UNKNOWN;
        addSubspace(StateSpacePtr(new DiscreteStateSpace(0,width)), 1.0);
        addSubspace(StateSpacePtr(new DiscreteStateSpace(0,height)), 1.0);
        addSubspace(StateSpacePtr(new DiscreteStateSpace(0,360)), 0.5);
        lock();
    }

    virtual ~DiscreteSE2StateSpace(void)
    {
    }

    /** \copydoc RealVectorStateSpace::setBounds() */
    void setBounds(const RealVectorBounds &bounds)
    {
        as<RealVectorStateSpace>(0)->setBounds(bounds);
    }

    /** \copydoc RealVectorStateSpace::getBounds() */
    const RealVectorBounds& getBounds(void) const
    {
        return as<RealVectorStateSpace>(0)->getBounds();
    }

    virtual State* allocState(void) const;
    virtual void freeState(State *state) const;

    virtual void registerProjections(void);

};

} // end namespace global_path_planner

#endif
