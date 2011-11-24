#ifndef TOPICPATHMANAGER_H
#define TOPICPATHMANAGER_H

//Qt includes
#include <QObject>
#include <QString>

//ROS includes
#include <nav_msgs/Path.h>

//project includes
#include "topicpath.h"


class TopicPathManager : public QObject
{
        Q_OBJECT

        //this list holds the data elements to be shown in the view
        const QString topic_name;
        TopicPathPtr current_path;

        //data attributes
        double pathlength;
        double median;
        int num_points;

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
        double getPathLength() const;

        //updated Median and distances list
        double updateMedian(TopicPathPtr ref_path);
        double getMedian() const;

        QList<double> getDistances() const;

        int updateNumPoints();
        int getNumPoints() const;


private:
        void initData();
        void updateData(const TopicPathPtr &topic_path);

Q_SIGNALS:

public Q_SLOTS:

};

typedef boost::shared_ptr<TopicPathManager> TopicPathManagerPtr;

#endif // TOPICPATHMANAGER_H
