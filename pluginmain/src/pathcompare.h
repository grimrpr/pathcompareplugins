#ifndef PATHCOMPARE_H
#define PATHCOMPARE_H

//Boost includes
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

//std includes
#include <iostream>
#include <fstream>
#include <string>

//ROS includes
#include "nav_msgs/Path.h"
#include "message_filters/cache.h"

//project include
#include "../../../pathcompare/src/interfaces/comperatorplugin.h"
#include "../../../pathcompare/src/interfaces/rosmanager.h"
#include "topicpath.h"
#include "topicpathmanager.h"
#include "graphtablemodel.h"
#include "ui_formmainplugin.h"


typedef boost::shared_ptr<message_filters::Cache<nav_msgs::Path> > PathCachePtr;
typedef boost::shared_ptr<TopicPathManager> TopicPathManagerPtr;
typedef boost::shared_ptr<GraphTableModel> GraphTableModelPtr;

class PathCompare : public ComperatorPlugin
{
        Q_OBJECT

        Ui::Form *form;
        ROSManager *ros_mngr;
        const std::string topic_type_str;
        QList<TopicPathManagerPtr> tpm_list;
        GraphTableModelPtr table_model;
        QList<message_filters::Connection> connections;

public:
    PathCompare(ROSManager *ros_mngr, QWidget *tab_widget);

Q_SIGNALS:
    void tpmListChanged(const QList<TopicPathManagerPtr> tm_list);

public Q_SLOTS:
    void updateTopics();
    void topicSelected(const QString topic_name);
    void writeCurrentData();

};

#endif // PATHCOMPARE_H
