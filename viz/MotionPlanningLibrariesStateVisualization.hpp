#ifndef motion_planning_libraries_MotionPlanningLibrariesStateVisualization_H
#define motion_planning_libraries_MotionPlanningLibrariesStateVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <motion_planning_libraries/State.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>

namespace vizkit3d
{
    class MotionPlanningLibrariesStateVisualization
        : public vizkit3d::Vizkit3DPlugin< motion_planning_libraries::State >
        , public vizkit3d::VizPluginAddType< std::vector<motion_planning_libraries::State> >
        , boost::noncopyable
    {
    Q_OBJECT
    Q_PROPERTY(QColor Color READ getColor WRITE setColor)
    
    public:
        MotionPlanningLibrariesStateVisualization();
        ~MotionPlanningLibrariesStateVisualization();

    Q_INVOKABLE void updateData(motion_planning_libraries::State const &data)
    {
        vizkit3d::Vizkit3DPlugin< motion_planning_libraries::State >::updateData(data);
    }
    
    Q_INVOKABLE void updateData(std::vector<motion_planning_libraries::State> const &data)
    {
        vizkit3d::Vizkit3DPlugin< motion_planning_libraries::State >::updateData(data);
    }
    
    public slots:
        void setColor(QColor q_color);
        QColor getColor() const;

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(motion_planning_libraries::State const& data);
        virtual void updateDataIntern(std::vector<motion_planning_libraries::State> const& data);
        void drawState(osg::Group* group, motion_planning_libraries::State& state);
        void drawLineBetweenStates(osg::Group* group) ;
        
    private:
        struct Data;
        Data* p;
        osg::Vec4 color;
    };
}
#endif
