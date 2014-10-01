#ifndef _SHERPA_STATE_SPACE_HPP_
#define _SHERPA_STATE_SPACE_HPP_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

#include <motion_planning_libraries/Config.hpp>

namespace motion_planning_libraries {

class SherpaStateSpace : public ompl::base::CompoundStateSpace
{
protected: 
    Config mConfig;
    
public:

    /** \brief A state in SherpaStateSpace: (x, y, yaw, footprint_radius) */
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
        
        unsigned int getFootprintClass(void) const
        {
            return as<ompl::base::DiscreteStateSpace::StateType>(2)->value;
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

        void setFootprintClass(unsigned int footprint_class)
        {
            as<ompl::base::DiscreteStateSpace::StateType>(2)->value = footprint_class;
        }
    };

    /**
     * TODO Set longest valid fraction?
     */
    SherpaStateSpace(Config config = Config()) : ompl::base::CompoundStateSpace(),
            mConfig(config)
    {
        setName("Sherpa" + getName());
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(2)), 1.0);
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 0.5);
        // Width and length, which should not be regarded in the distance calculation (?).
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::DiscreteStateSpace(
                0, config.mNumFootprintClasses-1)), 1.0);
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
