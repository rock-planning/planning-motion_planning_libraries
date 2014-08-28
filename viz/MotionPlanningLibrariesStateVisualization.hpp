#ifndef motion_planning_libraries_MotionPlanningLibrariesStateVisualization_H
#define motion_planning_libraries_MotionPlanningLibrariesStateVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <motion_planning_libraries/State.hpp>

namespace vizkit3d
{
    class MotionPlanningLibrariesStateVisualization
        : public vizkit3d::Vizkit3DPlugin<motion_planning_libraries::State>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        MotionPlanningLibrariesStateVisualization();
        ~MotionPlanningLibrariesStateVisualization();

    Q_INVOKABLE void updateData(motion_planning_libraries::State const &sample)
    {vizkit3d::Vizkit3DPlugin<motion_planning_libraries::State>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(motion_planning_libraries::State const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
