#include "pathcomparemainfactoryplugin.h"


PathcompareMainFactoryPlugin::PathcompareMainFactoryPlugin(QObject *parent) :
    QObject(parent)
{
}

ComperatorPluginPtr PathcompareMainFactoryPlugin::createComperatorPlugin(ROSManager * ros_manager) const
{
        qDebug() << ros_manager->getAllTopicTypes();
        ComperatorPluginPtr comp_plugin(new ComperatorPlugin());
        return comp_plugin;
}

QString PathcompareMainFactoryPlugin::getPluginName() const
{
        return QString("Main Compare");
}

Q_EXPORT_PLUGIN2(libmainplugin, PathcompareMainFactoryPlugin)
