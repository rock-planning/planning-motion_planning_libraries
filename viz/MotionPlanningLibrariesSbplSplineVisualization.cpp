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
    mAngleNum = num;
    emit propertyChanged("angle_num_changed");
    setDirty();
}

int MotionPlanningLibrariesSbplSplineVisualization::getEndAngle() const
{
    return mEndAngle;
}

void MotionPlanningLibrariesSbplSplineVisualization::setEndAngle(int num)
{
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


// PROTECTED
osg::ref_ptr<osg::Node> MotionPlanningLibrariesSbplSplineVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    //return new osg::Geode();
    return new osg::Group();
}

void MotionPlanningLibrariesSbplSplineVisualization::updateMainNode ( osg::Node* node )
{
    //osg::Geode* geode = static_cast<osg::Geode*>(node);
    osg::Group* group = node->asGroup();
    group->removeChildren(0, node->asGroup()->getNumChildren());
    
    // Update the main node using the data in        double sphere_radius = mRadiusEndpoints; p->data
    addPrimitives(group, p->data);
}

void MotionPlanningLibrariesSbplSplineVisualization::updateDataIntern(SbplSplineMotionPrimitives const& data)
{
    p->data = data;
    //std::cout << "got new sample data vector" << std::endl;
}

void MotionPlanningLibrariesSbplSplineVisualization::addPrimitives(osg::Group* group, 
        SbplSplineMotionPrimitives& primitives) {
        
  const std::vector<SplinePrimitive>& prims = primitives.getPrimitiveForAngle(mAngleNum);
  for(const SplinePrimitive& prim : prims)
  {
    if(prim.endAngle != mEndAngle)
      continue;
    // Draw intermediate lines representing the mprim within the world.
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
    
    
  /*
  
    std::vector<Primitive> prim_list = primitives.mListPrimitives;
    std::vector<Primitive>::iterator it = prim_list.begin();
    
    double hue_step = 1.0 / (double)primitives.mConfig.mNumAngles;
    double hue = 0.0;
    float r=0, g=0, b=0;
    osg::Vec4 color(r, g, b, 1.0f); 
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    int last_angle = -1;
    
    for(; it != prim_list.end(); it++) {
        
        // If mAlleAnglesShow is not set, just draw all prims of mAngleNum.
        if(!mAllAnglesShown && it->mStartAngle != (unsigned int)mAngleNum) {
            continue;
        }
        
        if(mColorizeTypes)
        {
            const double step = 1.0/MOV_NUM_TYPES;
            const double h = step * it->mMovType;
            vizkit3d::hslToRgb(h, 1.0, 0.5, r, g, b);
            color = osg::Vec4(r, g, b, 1.0f);    
            colors = new osg::Vec4Array;
            colors->clear();
            colors->push_back(color);
        }
        
        
        // Changes the prim color for each new angle. (overrides mColorizeTypes)
        if((int)it->mStartAngle != last_angle) {
            vizkit3d::hslToRgb(hue, 1.0, 0.5, r, g, b);
            color = osg::Vec4(r, g, b, 1.0f);    
            colors = new osg::Vec4Array;
            colors->clear();
            colors->push_back( color );
            hue += hue_step;
            
            last_angle = it->mStartAngle;
        }
    
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        
        if(mRadiusEndpoints > 0) {
            // Create sphere.
            osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3d(0,0,0), mRadiusEndpoints);
            osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp.get());
            sd->setColor(color);
            geode->addDrawable(sd.get());
            

            
            // Move the sphere and the triangle to the endpose (converted from grid to world)
            // using a transform.
            base::Vector3d mEndPose = it->mEndPose;
            //std::cout << "Endpose " << mEndPose[0] << " " << mEndPose[1] << " scale factor " << primitives.mScaleFactor << std::endl;
            double x = mEndPose[0] * primitives.mConfig.mGridSize;
            double y = mEndPose[1] * primitives.mConfig.mGridSize;
            double z = 0; // mEndPose[2] * primitives.mScaleFactor; // z is discrete orientation
            double theta = mEndPose[2] * ((2*M_PI)/primitives.mConfig.mNumAngles);

            //std::cout << "Draw sphere and triangle at XYZTHETA: " << x << " "  << y << " " << z << " " << theta << " " << std::endl; 
            
            osg::ref_ptr<osg::PositionAttitudeTransform> transform = 
                    new osg::PositionAttitudeTransform();
            osg::Vec3 position = osg::Vec3d(x, y, z);
            position[2] += 0.01; // Moves the waypoints a little bit above the z=0 plane.
            transform->setPosition(position);
            // osg::Quat(0,0,1,heading) != osg::Quat(heading, Vec(0,0,1)).. why?
            transform->setAttitude(osg::Quat(theta, osg::Vec3f(0,0,1)));
            transform->addChild(geode);
        
            // Add transformer containing the spehere and the triangle to the passed group.
            group->addChild(transform);

            if(it->mMovType == MOV_FORWARD_TURN || 
                    it->mMovType == MOV_BACKWARD_TURN) {
                // Adds a circle for the center of rotation.
                osg::ref_ptr<osg::Geode> geode_cof = new osg::Geode();
                
                double x_cof = it->mCenterOfRotation[0] * primitives.mConfig.mGridSize;
                double y_cof = it->mCenterOfRotation[1] * primitives.mConfig.mGridSize;
                osg::ref_ptr<osg::Sphere> sp_cof = new osg::Sphere(osg::Vec3d(x_cof, y_cof, 0), mRadiusEndpoints);
                osg::ref_ptr<osg::ShapeDrawable> sd_cof = new osg::ShapeDrawable(sp_cof.get());
                sd_cof->setColor(color);
                geode_cof->addDrawable(sd_cof.get());
                
                // Adds a line from the cof to the end point.
                osg::ref_ptr<osg::Geometry> cof_geometry = new osg::Geometry();
                osg::ref_ptr<osg::Vec3Array> cof_vertices = new osg::Vec3Array();
                cof_vertices->push_back(osg::Vec3(x_cof, y_cof, 0));
                cof_vertices->push_back(osg::Vec3(x, y, 0));
                cof_geometry->setVertexArray(cof_vertices);
                cof_geometry->setColorArray(colors);
                cof_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
                cof_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,2));
                geode_cof->addDrawable(cof_geometry);
                
                group->addChild(geode_cof);
            }
        }
    }*/
}




//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(MotionPlanningLibrariesSbplSplineVisualization)

