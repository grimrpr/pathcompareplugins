//Qt includes
#include <QDebug>
//project includes
#include "../../../pathcompare/src/interfaces/comperatorplugin.h"

ComperatorPlugin::ComperatorPlugin(QObject *parent) :
    QObject(parent)
{
}

void ComperatorPlugin::testFunction() const
{
        qDebug() << "Comperator Plugin Testausgabe";
}
