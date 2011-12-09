#ifndef TOPICPATH_H
#define TOPICPATH_H

//ROS includes
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>

//openCV
#include <cv.hpp>

//Qt includes
#include <QObject>
#include <QList>
#include <QString>


class Position
{
public:
        const QString frame_id;
        const uint sequence_nbr;
        const cv::Point3d point;
        const ros::Time timestamp;

        explicit Position(const uint &sequence_nbr, const cv::Point3d &point, const ros::Time &timestamp) :
                sequence_nbr(sequence_nbr),
                point(point),
                timestamp(timestamp)
        {}

        bool operator<(const Position & rhs) const
        {
                return timestamp < rhs.timestamp;
        }

        bool operator<=(const Position & rhs) const
        {
                return timestamp <= rhs.timestamp;
        }
};


//TODO: remove QObject bloat if not needed

class TopicPath : public QObject
{
    Q_OBJECT

public:
    QString frame_id;
    uint path_seq;
    ros::Time lastupdated;
    QList<Position> points;
    nav_msgs::Path nav_path;
//    nav_msgs::PathConstPtr nav_path;

    TopicPath();
    TopicPath(const nav_msgs::PathConstPtr &path);

    void setDataFromNavPath(const nav_msgs::Path &nav_path);

    QList<cv::Point3d > getAllPointsOrdered() const;

};

typedef boost::shared_ptr<TopicPath> TopicPathPtr;
#endif // TOPICPATH_H
