#include "MotionPlanningLibrariesSbplSplineVisualization.hpp"

#include <iostream>

#include <osg/Geometry>
#include <osg/Group>
#include <osg/ShapeDrawable>

using namespace vizkit3d;
using namespace motion_planning_libraries;

struct MotionPlanningLibrariesSbplSplineVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    SbplSplineMotionPrimitives data;
};

// PUBLIC
MotionPlanningLibrariesSbplSplineVisualization::MotionPlanningLibrariesSbplSplineVisualization()
    : mAngleNum(0), mEndAngle(0), p(new Data)
{
}

MotionPlanningLibrariesSbplSplineVisualization::~MotionPlanningLibrariesSbplSplineVisualization()
{
    delete p;
}

// PUBLIC SLOTS
int MotionPlanningLibrariesSbplSplineVisualization::getAngleNum() const {
    return mAngleNum;
}

void MotionPlanningLibrariesSbplSplineVisualization::setAngleNum(int num) {
    
    if(num < 0)
    {
        while(num < 0)
        {
            //shifting by multiples of numAngles does not change the angle because of the modulo
            num += p->data.getConfig().numAngles; 
        }
        num = num % p->data.getConfig().numAngles;
    }
    
    mAngleNum = num;
    mEndAngle = num;
    emit propertyChanged("angleNum");
    emit propertyChanged("endAngle");
    setDirty();
}

int MotionPlanningLibrariesSbplSplineVisualization::getEndAngle() const
{
    return mEndAngle;
}

void MotionPlanningLibrariesSbplSplineVisualization::setEndAngle(int num)
{
    if(num < 0)
    {
        while(num < 0)
        {
            //shifting by multiples of numAngles does not change the angle because of the modulo
            num += p->data.getConfig().numAngles; 
        }
        num = num % p->data.getConfig().numAngles;
    }
    mEndAngle = num;
    emit propertyChanged("endAngle");
    setDirty();
}


bool MotionPlanningLibrariesSbplSplineVisualization::allAnglesShown() const {
    return mAllAnglesShown;
}

void MotionPlanningLibrariesSbplSplineVisualization::setShowAllAngles(bool enabled) {
    mAllAnglesShown = enabled;
    emit propertyChanged("show_all_angles");
    setDirty();
}

double MotionPlanningLibrariesSbplSplineVisualization::getMaxCurvature() const
{
    return maxCurvature;
}

void MotionPlanningLibrariesSbplSplineVisualization::setMaxCurvature(const double value)
{
    maxCurvature = value;
    setDirty();
    emit propertyChanged("maxCurvature");
}



osg::ref_ptr<osg::Node> MotionPlanningLibrariesSbplSplineVisualization::createMainNode()
{
    return new osg::Group();
}

void MotionPlanningLibrariesSbplSplineVisualization::updateMainNode ( osg::Node* node )
{
    osg::Group* group = node->asGroup();
    group->removeChildren(0, node->asGroup()->getNumChildren());
    
    addPrimitives(group, p->data);
}

void MotionPlanningLibrariesSbplSplineVisualization::updateDataIntern(SbplSplineMotionPrimitives const& data)
{
    p->data = data;
}

void MotionPlanningLibrariesSbplSplineVisualization::addPrimitives(osg::Group* group, 
        SbplSplineMotionPrimitives& primitives) {
        
  const std::vector<SplinePrimitive>& prims = primitives.getPrimitiveForAngle(mAngleNum);
  for(const SplinePrimitive& prim : prims)
  {
    if(static_cast<int>(prim.endAngle) != mEndAngle)
      continue;
    
    if(const_cast<SplinePrimitive&>(prim).spline.getCurvatureMax() > maxCurvature)
    {
        std::cout << const_cast<SplinePrimitive&>(prim).spline.getCurvatureMax() << " > " << maxCurvature << std::endl;
        continue;
    }
    
    osg::ref_ptr<osg::Geode> geode_intermediate_points = new osg::Geode();
    std::vector<base::Vector2d> poses = prim.spline.sample(0.01);

    osg::ref_ptr<osg::Geometry> fp_geometry = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> fp_vertices = new osg::Vec3Array();
    base::Vector3d v_tmp;
    for(const base::Vector2d& intermediatePose : poses)
    {
      fp_vertices->push_back(osg::Vec3(intermediatePose[0], intermediatePose[1], 0));
    }
    osg::Vec4 color(1.0, 0.0, 0.0, 1.0f); 
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    fp_geometry->setVertexArray(fp_vertices);
    fp_geometry->setColorArray(colors);
    fp_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    fp_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0, poses.size()));
    geode_intermediate_points->addDrawable(fp_geometry);

    colors->push_back(color);
    // Add intermediate points to the passed group.
    group->addChild(geode_intermediate_points);
      
    // Create triangle.
    osg::ref_ptr<osg::Geometry> triangle_geometry = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> triangle_vertices = new osg::Vec3Array();
    triangle_vertices->push_back(osg::Vec3(0.0, 0.01, 0));
    triangle_vertices->push_back(osg::Vec3(0.04, 0.0, 0));
    triangle_vertices->push_back(osg::Vec3(0.0, -0.01, 0));
    triangle_geometry->setVertexArray(triangle_vertices);
    osg::ref_ptr<osg::DrawElementsUInt> triangle_face = 
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    triangle_face->push_back(0);
    triangle_face->push_back(1);
    triangle_face->push_back(2);
    triangle_geometry->addPrimitiveSet(triangle_face);
    triangle_geometry->setColorArray(colors);
    triangle_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(triangle_geometry);
    
    // Move the sphere and the triangle to the endpose (converted from grid to world)
    // using a transform.
    osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
    osg::Vec3 position = osg::Vec3d(prim.spline.getEndPoint().x(), prim.spline.getEndPoint().y(), 0);
    position[2] += 0.001; // Moves the waypoints a little bit above the z=0 plane.
    transform->setPosition(position);
    transform->setAttitude(osg::Quat(prim.endAngleRad, osg::Vec3f(0,0,1)));
    transform->addChild(geode);
    group->addChild(transform);
  }
}




//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(MotionPlanningLibrariesSbplSplineVisualization)

