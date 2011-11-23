#ifndef TOPICPATHMANAGER_H
#define TOPICPATHMANAGER_H

//Qt includes
#include <QObject>
#include <QString>

//ROS includes
#include <nav_msgs/Path.h>

//project includes
#include "topicpath.h"

typedef boost::shared_ptr<TopicPath> TopicPathPtr;

class TopicPathManager : public QObject
{
        Q_OBJECT

        //this list holds the data elements to be shown in the view
        const QString topic_name;
        TopicPathPtr current_path;
//        QList<QString> data;
        double pathlength;
        double median;
        QList<double> distances;
        TopicPathPtr current_ref_path;

public:
        explicit TopicPathManager(QString topic_name, QObject *parent = 0);
        void processNewPathMsg(const nav_msgs::PathConstPtr &path);
        int getNumData() const;

        QString getTopicName() const;
        TopicPathPtr getCurrentPath() const;
        void updateReferencePath(TopicPathPtr new_ref_tp);

//      QString getDataAt(int row) const;

        //data functions
        double updatePathLen();
        double updateMedian(TopicPathPtr ref_path);
private:
        void initData();
        void updateData(const TopicPathPtr &topic_path);

Q_SIGNALS:

public Q_SLOTS:

};

#endif // TOPICPATHMANAGER_H
