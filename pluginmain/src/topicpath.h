#ifndef TOPICPATH_H
#define TOPICPATH_H

//ROS includes
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>

//Qt includes
#include <QVector3D>
#include <QList>


class Position
{
public:
        const QString frame_id;
        const uint sequence_nbr;
        const QVector3D point;
        const ros::Time timestamp;

        explicit Position(const uint &sequence_nbr, const QVector3D &point, const ros::Time &timestamp) :
                sequence_nbr(sequence_nbr),
                point(point),
                timestamp(timestamp)
        {}

        bool operator<(const Position & rhs) const
        {
                return timestamp < rhs.timestamp;
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

    TopicPath();
    TopicPath(const nav_msgs::PathConstPtr &path);

    void setDataFromNavPath(const nav_msgs::PathConstPtr &nav_path);

    QList<QVector3D> getAllPointsOrdered() const;

};

typedef boost::shared_ptr<TopicPath> TopicPathPtr;
#endif // TOPICPATH_H
