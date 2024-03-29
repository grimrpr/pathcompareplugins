#ifndef PATHCOMPAREMAINFACTORYPLUGIN_H
#define PATHCOMPAREMAINFACTORYPLUGIN_H

//QT includes
#include <QObject>
#include <QWidget>

//project includes
#include "../../../pathcompare/src/interfaces/comperatorpluginfactoryinterface.h"
#include "../../../pathcompare/src/interfaces/comperatorplugin.h"
#include "../../../pathcompare/src/interfaces/rosmanager.h"

#include "pathcompare.h"


class PathcompareMainFactoryPlugin : public QObject, public ComperatorPluginFactoryInterface
{
        Q_OBJECT
        Q_INTERFACES(ComperatorPluginFactoryInterface)
public:
        explicit PathcompareMainFactoryPlugin(QObject *parent = 0);

        ComperatorPluginPtr createComperatorPlugin(ROSManager * ros_manager, QWidget *tab_widget) const;
        QString getPluginName() const;

Q_SIGNALS:


public Q_SLOTS:

};


#endif // PATHCOMPAREMAINFACTORYPLUGIN_H
