#ifndef _SHERPA_STATE_SPACE_HPP_
#define _SHERPA_STATE_SPACE_HPP_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace motion_planning_libraries {

class SherpaStateSpace : public ompl::base::CompoundStateSpace
{
public:

    /** \brief A state in SherpaStateSpace: (x, y, yaw, length, width) */
    class StateType : public ompl::base::CompoundStateSpace::StateType
    {
    public:
        StateType(void) : ompl::base::CompoundStateSpace::StateType()
        {
        }

        /** \brief Get the X component of the state */
        double getX(void) const
        {
            return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        }

        /** \brief Get the Y component of the state */
        double getY(void) const
        {
            return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        }

        /** \brief Get the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        double getYaw(void) const
        {
            return as<ompl::base::SO2StateSpace::StateType>(1)->value;
        }
        
        double getLength(void) const
        {
            return as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0];
        }
        
        double getWidth(void) const
        {
            return as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1];
        }

        /** \brief Set the X component of the state */
        void setX(double x)
        {
            as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = x;
        }

        /** \brief Set the Y component of the state */
        void setY(double y)
        {
            as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = y;
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
            as<ompl::base::SO2StateSpace::StateType>(1)->value = yaw;
        }
        
        void setLength(double length)
        {
            as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0] = length;
        }
        
        void setWidth(double width)
        {
            as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1] = width;
        }

        void setFootprint(double width, double length)
        {
            setWidth(width);
            setLength(length);
        }
    };

    /**
     * TODO Set longest valid freaction?
     */
    SherpaStateSpace(void) : ompl::base::CompoundStateSpace()
    {
        setName("Sherpa" + getName());
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(2)), 1.0);
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 0.5);
        // Width and length, which should not be regarded in the distance calculation.
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(2)), 0.0);
        // 0 means min length/width, 1.0 max.
        ompl::base::RealVectorBounds bound(2);
        bound.setLow (0.0);
        bound.setHigh(1.0);
        as<ompl::base::RealVectorStateSpace>(2)->setBounds(bound);
        lock();
    }

    virtual ~SherpaStateSpace(void)
    {
    }

    /** \copydoc RealVectorStateSpace::setBounds() */
    void setBounds(const ompl::base::RealVectorBounds &bounds)
    {
        as<ompl::base::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    /** \copydoc RealVectorStateSpace::getBounds() */
    const ompl::base::RealVectorBounds& getBounds(void) const
    {
        return as<ompl::base::RealVectorStateSpace>(0)->getBounds();
    }

    virtual ompl::base::State* allocState(void) const;
    virtual void freeState(ompl::base::State *state) const;

    virtual void registerProjections(void);

};

} // end namespace motion_planning_libraries

#endif
