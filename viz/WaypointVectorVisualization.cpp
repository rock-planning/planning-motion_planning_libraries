#include <iostream>
#include "WaypointVectorVisualization.hpp"

using namespace vizkit3d;

struct WaypointVectorVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::vector<base::Waypoint> data;
};


WaypointVectorVisualization::WaypointVectorVisualization()
    : p(new Data)
{
}

WaypointVectorVisualization::~WaypointVectorVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> WaypointVectorVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void WaypointVectorVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void WaypointVectorVisualization::updateDataIntern(std::vector<base::Waypoint> const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(WaypointVectorVisualization)

