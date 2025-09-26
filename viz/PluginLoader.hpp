#ifndef PLUGINLOADER_HPP
#define PLUGINLOADER_HPP

#include <vizkit3d/Vizkit3DPlugin.hpp>

namespace vizkit3d {
    
class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
    Q_OBJECT;
#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "rock.vizkit3d.VizkitPluginFactory")
#endif
    private:
    public:

    QtPluginVizkit();

    /**
    * Returns a list of all available visualization plugins.
    * @return list of plugin names
    */
    virtual QStringList* getAvailablePlugins() const;

    virtual QObject* createPlugin(QString const& pluginName);
};

} // end namespace vizkit3d

#endif /*PLUGINLOADER_HPP*/
