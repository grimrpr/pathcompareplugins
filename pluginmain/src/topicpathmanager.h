#ifndef TOPICPATHMANAGER_H
#define TOPICPATHMANAGER_H

//Qt includes
#include <QObject>
#include <QString>
#include <QMap>
#include <QReadWriteLock>

//std includes
#include <string>

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
        double s_2;
        double arith_mean;

        //ordered list of distances
        QList<double> distances;
        //mapping between position and current distance to reference point
        QMap<Position, double> pos_dist_map;
        QReadWriteLock lock_current_path, lock_pos_dist_map;

        TopicPathPtr current_ref_path;

public:
        explicit TopicPathManager(QString topic_name, QObject *parent = 0);
        void processNewPathMsg(const nav_msgs::PathConstPtr &path);
        int getNumData() const;

        QString getTopicName() const;
        TopicPathPtr getCurrentPath() const;
        void updateReferencePath(TopicPathPtr new_ref_tp);

        //data functions
        double updatePathLen();
        double getPathLength() const;

        //update Median and distances list
        double updateMedian(TopicPathPtr ref_path);
        double getMedian() const;

        //updates arithmetic mean returens cov s'^2
        double updateS2AndArithMean();
        double getS2() const;
        double getS() const;
        double getArithMean() const;

        QList<double> getDistances() const;

        void writeCSVstring(std::stringstream &outstr) const;

        int updateNumPoints();
        int getNumPoints() const;


private:
        void updateData(const TopicPathPtr &topic_path);
        void writeDataToStream(std::stringstream &stream,
                               QMap<Position, double> pos_dist_map_,
                               double pathlength_,
                               double median_,
                               double s_2_,
                               double arith_mean_
                               ) const;

Q_SIGNALS:
        void refreshTPM(const QString & topic);

public Q_SLOTS:

};

typedef boost::shared_ptr<TopicPathManager> TopicPathManagerPtr;

#endif // TOPICPATHMANAGER_H
