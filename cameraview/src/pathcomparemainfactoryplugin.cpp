#include "pathcomparemainfactoryplugin.h"


PathcompareMainFactoryPlugin::PathcompareMainFactoryPlugin(QObject *parent) :
    QObject(parent)
{
}

ComperatorPluginPtr PathcompareMainFactoryPlugin::createComperatorPlugin(ROSManager * ros_manager, QWidget *tab_widget) const
{
        qDebug() << ros_manager->getAllTopicTypes();
        ComperatorPluginPtr comp_plugin(static_cast<ComperatorPlugin *>(new PathCompare(ros_manager, tab_widget)));
        return comp_plugin;
}

QString PathcompareMainFactoryPlugin::getPluginName() const
{
        return QString("Camera View");
}

Q_EXPORT_PLUGIN2(libmainplugin, PathcompareMainFactoryPlugin)
