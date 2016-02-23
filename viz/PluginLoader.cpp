#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "MotionPlanningLibrariesSbplMprimsVisualization.hpp"
#include "MotionPlanningLibrariesStateVisualization.hpp"

namespace vizkit3d {
    
class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
    private:
    public:

    QtPluginVizkit() {
    }

    /**
    * Returns a list of all available visualization plugins.
    * @return list of plugin names
    */
    virtual QStringList* getAvailablePlugins() const
    {
        QStringList *pluginNames = new QStringList();
        pluginNames->push_back("MotionPlanningLibrariesStateVisualization");
        pluginNames->push_back("MotionPlanningLibrariesSbplMprimsVisualization");
        return pluginNames;
    }

    virtual QObject* createPlugin(QString const& pluginName)
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
    };
};
Q_EXPORT_PLUGIN2(QtPluginVizkit, QtPluginVizkit)

} // end namespace vizkit3d
