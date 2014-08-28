#include <iostream>
#include "MotionPlanningLibrariesStateVisualization.hpp"

using namespace vizkit3d;

struct MotionPlanningLibrariesStateVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    motion_planning_libraries::State data;
};


MotionPlanningLibrariesStateVisualization::MotionPlanningLibrariesStateVisualization()
    : p(new Data)
{
}

MotionPlanningLibrariesStateVisualization::~MotionPlanningLibrariesStateVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> MotionPlanningLibrariesStateVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void MotionPlanningLibrariesStateVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void MotionPlanningLibrariesStateVisualization::updateDataIntern(motion_planning_libraries::State const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(MotionPlanningLibrariesStateVisualization)

