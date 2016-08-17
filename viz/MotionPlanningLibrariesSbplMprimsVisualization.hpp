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
    
    Q_PROPERTY(int angleNum READ getAngleNum WRITE setAngleNum)
    Q_PROPERTY(bool showAllAngles READ allAnglesShown WRITE setShowAllAngles)
    Q_PROPERTY(bool colorizeByType READ getColorizeTypes WRITE setColorizeTypes)
    Q_PROPERTY(double radiusEndpoints READ getRadiusEndpoints WRITE setRadiusEndpoints)
    
    public:
        MotionPlanningLibrariesSbplMprimsVisualization();
        ~MotionPlanningLibrariesSbplMprimsVisualization();

    Q_INVOKABLE void updateData(motion_planning_libraries::SbplMotionPrimitives const &data)
    {
        vizkit3d::Vizkit3DPlugin< motion_planning_libraries::SbplMotionPrimitives >::updateData(data);
    }
    
    public slots:
        int getAngleNum() const;
        void setAngleNum(int num);
        bool allAnglesShown() const;
        void setShowAllAngles(bool enabled);
        double getRadiusEndpoints() const;
        void setRadiusEndpoints(double radius);
        bool getColorizeTypes() const;
        void setColorizeTypes(const bool value);

    protected:
        int mAngleNum;
        bool mAllAnglesShown;
        double mRadiusEndpoints;
        bool mColorizeTypes; //if true, give every primitive type a different color
        
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
