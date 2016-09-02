#ifndef motion_planning_libraries_MotionPlanningLibrariesSbplSplineVisualization_H
#define motion_planning_libraries_MotionPlanningLibrariesSbplSplineVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <vizkit3d/ColorConversionHelper.hpp>
#include <osg/Geode>
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>

namespace vizkit3d
{
    class MotionPlanningLibrariesSbplSplineVisualization
        : public vizkit3d::Vizkit3DPlugin< motion_planning_libraries::SbplSplineMotionPrimitives>
        , boost::noncopyable
    {
    Q_OBJECT
    
    Q_PROPERTY(int angleNum READ getAngleNum WRITE setAngleNum)
    Q_PROPERTY(int endAngle READ getEndAngle WRITE setEndAngle)
    
    public:
        MotionPlanningLibrariesSbplSplineVisualization();
        ~MotionPlanningLibrariesSbplSplineVisualization();

    Q_INVOKABLE void updateData(motion_planning_libraries::SbplSplineMotionPrimitives const &data)
    {
        vizkit3d::Vizkit3DPlugin< motion_planning_libraries::SbplSplineMotionPrimitives>::updateData(data);
    }
    
    public slots:
        int getAngleNum() const;
        void setAngleNum(int num);
        int getEndAngle() const;
        void setEndAngle(int num);
        bool allAnglesShown() const;
        void setShowAllAngles(bool enabled);

    protected:
        int mAngleNum;
        int mEndAngle;
        bool mAllAnglesShown;
        
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(motion_planning_libraries::SbplSplineMotionPrimitives const& data);
        void addPrimitives(osg::Group* group, motion_planning_libraries::SbplSplineMotionPrimitives& primitives);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
