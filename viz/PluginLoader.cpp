
#include "PluginLoader.hpp"
#include "MotionPlanningLibrariesSbplMprimsVisualization.hpp"
#include "MotionPlanningLibrariesStateVisualization.hpp"

using namespace vizkit3d;

QtPluginVizkit::QtPluginVizkit() {
}

QStringList* QtPluginVizkit::getAvailablePlugins() const
{
    QStringList *pluginNames = new QStringList();
    pluginNames->push_back("MotionPlanningLibrariesStateVisualization");
    pluginNames->push_back("MotionPlanningLibrariesSbplMprimsVisualization");
    return pluginNames;
}

QObject* QtPluginVizkit::createPlugin(QString const& pluginName)
{
    vizkit3d::VizPluginBase* plugin = 0;
    if (pluginName == "MotionPlanningLibrariesSbplMprimsVisualization")
    {
        plugin = new vizkit3d::MotionPlanningLibrariesSbplMprimsVisualization();
    }
    else if (pluginName == "MotionPlanningLibrariesStateVisualization")
    {
        plugin = new vizkit3d::MotionPlanningLibrariesStateVisualization();
    }

    if (plugin)
    {
        return plugin;
    }
    return NULL;
}
