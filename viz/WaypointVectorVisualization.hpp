#ifndef global_path_planner_WaypointVectorVisualization_H
#define global_path_planner_WaypointVectorVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/Waypoint.hpp>

namespace vizkit3d
{
    class WaypointVectorVisualization
        : public vizkit3d::Vizkit3DPlugin<std::vector<base::Waypoint>>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        WaypointVectorVisualization();
        ~WaypointVectorVisualization();

    Q_INVOKABLE void updateData(std::vector<base::Waypoint> const &sample)
    {vizkit3d::Vizkit3DPlugin<std::vector<base::Waypoint>>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(std::vector<base::Waypoint> const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
