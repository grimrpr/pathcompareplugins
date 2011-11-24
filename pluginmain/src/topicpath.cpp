#include "topicpath.h"

TopicPath::TopicPath()
{
}

TopicPath::TopicPath(const nav_msgs::PathConstPtr &path)
{
        setDataFromNavPath(path);
}

//CAREFUL
//we assume that we got all points ordered in the path message
QList<cv::Point3d > TopicPath::getAllPointsOrdered() const
{
        QList<cv::Point3d > result;

        //CAREFUL
        //we assume that we got all points ordered in the path message

        QList<Position>::const_iterator it;
        for(it = points.constBegin(); it != points.constEnd(); ++it)
        {
                result << it->point;
        }

        return result;
}

///initialize attributes
void TopicPath::setDataFromNavPath(const nav_msgs::PathConstPtr &nav_path)
{

        frame_id = QString(nav_path->header.frame_id.c_str());
        path_seq = nav_path->header.seq;
        lastupdated = nav_path->header.stamp;

        for(uint i = 0; i < nav_path->poses.size(); ++i)
        {
                const geometry_msgs::PoseStamped *pose_stmpd = &(nav_path->poses[i]);

                cv::Point3d point(pose_stmpd->pose.position.x,
                          pose_stmpd->pose.position.y,
                          pose_stmpd->pose.position.z);

                Position position(pose_stmpd->header.seq,
                                  point,
                                  pose_stmpd->header.stamp);

                points << position;
        }
}
