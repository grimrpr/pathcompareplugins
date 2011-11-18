#ifndef PATHCOMPARE_H
#define PATHCOMPARE_H

//std includes
#include <iostream>
#include <string>

//ROS includes
#include "nav_msgs/Path.h"

//project include
#include "../../../pathcompare/src/interfaces/comperatorplugin.h"
#include "../../../pathcompare/src/interfaces/rosmanager.h"


#include "ui_formmainplugin.h"

class PathCompare : public ComperatorPlugin
{
        Q_OBJECT

        Ui::Form *form;
        ROSManager *ros_mngr;
        const std::string topic_type_str;


public:
    PathCompare(ROSManager *ros_mngr, QWidget *tab_widget);

public Q_SLOTS:
    void updateTopics();

};

#endif // PATHCOMPARE_H
