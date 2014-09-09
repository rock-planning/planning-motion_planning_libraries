#include "MotionPlanningLibrariesStateVisualization.hpp"

#include <iostream>

#include <osg/Geometry>
#include <osg/Group>
#include <osg/ShapeDrawable>

using namespace vizkit3d;

struct MotionPlanningLibrariesStateVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::vector <motion_planning_libraries::State> data;
};


MotionPlanningLibrariesStateVisualization::MotionPlanningLibrariesStateVisualization()
    : p(new Data), color(1.0f, 0.0f, 0.0f, 1.0f)
{
}

MotionPlanningLibrariesStateVisualization::~MotionPlanningLibrariesStateVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> MotionPlanningLibrariesStateVisualization::createMainNode()
{
    std::cout << "create main node" << std::endl;
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    //return new osg::Geode();
    return new osg::Group();
}

void MotionPlanningLibrariesStateVisualization::updateMainNode ( osg::Node* node )
{
    std::cout << "update main node" << std::endl;
    //osg::Geode* geode = static_cast<osg::Geode*>(node);
    osg::Group* group = node->asGroup();
    group->removeChildren(0, node->asGroup()->getNumChildren());
    
    // Update the main node using the data in p->data
    std::vector <motion_planning_libraries::State>::iterator it = p->data.begin();
    for(;it != p->data.end(); ++it) {
        drawState(group, *it);
    }
}

void MotionPlanningLibrariesStateVisualization::updateDataIntern(std::vector<motion_planning_libraries::State> const& data)
{
    p->data = data;
    std::cout << "got new sample data vector" << std::endl;
}

void MotionPlanningLibrariesStateVisualization::updateDataIntern(motion_planning_libraries::State const& data)
{
    p->data.clear();
    p->data.push_back(data);
    std::cout << "got new sample data" << std::endl;
}

void MotionPlanningLibrariesStateVisualization::drawState(osg::Group* group, motion_planning_libraries::State& state) {
    // Create geode and add drawables.
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    
    // Create triangle.
    osg::ref_ptr<osg::Geometry> triangle_geometry = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> triangle_vertices = new osg::Vec3Array();
    triangle_vertices->push_back(osg::Vec3(0.0, 0.2, 0));
    triangle_vertices->push_back(osg::Vec3(0.6, 0.0, 0));
    triangle_vertices->push_back(osg::Vec3(0.0, -0.2, 0));
    triangle_geometry->setVertexArray(triangle_vertices);
    
    osg::ref_ptr<osg::DrawElementsUInt> triangle_face = 
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    triangle_face->push_back(0);
    triangle_face->push_back(1);
    triangle_face->push_back(2);
    triangle_geometry->addPrimitiveSet(triangle_face);
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back( color );
    triangle_geometry->setColorArray(colors);
    triangle_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    geode->addDrawable(triangle_geometry);
    
    // Create footprint
    if(state.mWidth > 0 && state.mLength > 0) {
        osg::ref_ptr<osg::Geometry> fp_geometry = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> fp_vertices = new osg::Vec3Array();
        double w_2=state.mWidth/2.0, l_2=state.mLength/2.0;
        fp_vertices->push_back(osg::Vec3(l_2, w_2, 0.0));
        fp_vertices->push_back(osg::Vec3(l_2, -w_2, 0.0));
        fp_vertices->push_back(osg::Vec3(-l_2, -w_2, 0.0));
        fp_vertices->push_back(osg::Vec3(-l_2, w_2, 0.0));
        fp_vertices->push_back(osg::Vec3(l_2, w_2, 0.0));

        fp_geometry->setVertexArray(fp_vertices);
        fp_geometry->setColorArray(colors);
        fp_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
        fp_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,5));
           
        geode->addDrawable(fp_geometry);
    }
    
    // Create transformer.
    osg::ref_ptr<osg::PositionAttitudeTransform> transform = 
            new osg::PositionAttitudeTransform();
    base::Vector3d vec = state.mPose.position;
    osg::Vec3 position = osg::Vec3d(vec.x(), vec.y(), vec.z());
    position[2] += 0.01; // Moves the waypoints a little bit above the z=0 plane.
    transform->setPosition(position);
    // osg::Quat(0,0,1,heading) != osg::Quat(heading, Vec(0,0,1)).. why?
    // -M_PI/2.0 because rock defines x to be the front axis.
    transform->setAttitude(osg::Quat(state.mPose.getYaw(), osg::Vec3f(0,0,1)));
    transform->addChild(geode);
    
    // Adds the waypoints to the main node.
    group->addChild(transform);
}

void MotionPlanningLibrariesStateVisualization::setColor(QColor q_color)
{
    color = osg::Vec4(q_color.redF(), q_color.greenF(), 
            q_color.blueF(), q_color.alphaF());
    setDirty();
}

QColor MotionPlanningLibrariesStateVisualization::getColor() const
{
    QColor q_color;
    q_color.setRgbF(color[0], color[1], color[2], color[3]);
    return q_color;
} 

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(MotionPlanningLibrariesStateVisualization)

