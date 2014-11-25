#ifndef motion_planning_libraries_MotionPlanningLibrariesSbplMprimsVisualization_H
#define motion_planning_libraries_MotionPlanningLibrariesSbplMprimsVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <vizkit3d/ColorConversionHelper.hpp>
#include <osg/Geode>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>

namespace vizkit3d
{
    class MotionPlanningLibrariesSbplMprimsVisualization
        : public vizkit3d::Vizkit3DPlugin< motion_planning_libraries::SbplMotionPrimitives >
        , boost::noncopyable
    {
    Q_OBJECT
    
    Q_PROPERTY(bool showAllAngles READ allAnglesShown WRITE setShowAllAngles)
    Q_PROPERTY(double radiusEndpoints READ getRadiusEndpoints WRITE setRadiusEndpoints)
    
    public:
        MotionPlanningLibrariesSbplMprimsVisualization();
        ~MotionPlanningLibrariesSbplMprimsVisualization();

    Q_INVOKABLE void updateData(motion_planning_libraries::SbplMotionPrimitives const &data)
    {
        vizkit3d::Vizkit3DPlugin< motion_planning_libraries::SbplMotionPrimitives >::updateData(data);
    }
    
    public slots:
        bool allAnglesShown() const;
        void setShowAllAngles(bool enabled);
        double getRadiusEndpoints() const;
        void setRadiusEndpoints(double radius);

    protected:
        bool mAllAnglesShown;
        double mRadiusEndpoints;
        
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(motion_planning_libraries::SbplMotionPrimitives const& data);
        void addPrimitives(osg::Group* group, motion_planning_libraries::SbplMotionPrimitives& primitives);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
