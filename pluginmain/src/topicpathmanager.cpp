#include "topicpathmanager.h"

TopicPathManager::TopicPathManager(QString topic_name, QObject *parent) :
    QObject(parent),
        topic_name(topic_name)
{
        initData();
}

void TopicPathManager::processNewPathMsg(const nav_msgs::PathConstPtr &path)
{
        current_path = TopicPathPtr(new TopicPath(path));

        updateData(current_path);
}

/*
QString TopicPathManager::getDataAt(int row) const
{
        if(data.size() <= row)
                return QString();
        else
                return data.at(row);
}
*/

void TopicPathManager::initData()
{
//        for(int i = 0; i < num_data_fields; ++i)
//                data << QString();
}

void TopicPathManager::updateData(const TopicPathPtr &topic_path)
{
        pathlength = updatePathLen();
        /*
        std::cout << "The path of topic: "
                  << topic_name.toLocal8Bit().constData()
                  << " is now: " << pathlength
                  << " long. " << std::endl;
                  */

        median = updateMedian(current_ref_path);
        /*
        std::cout << "The path of topic: "
                  << topic_name.toLocal8Bit().constData()
                  << " has now: " << median
                  << " as median devergence to reference path. " << std::endl;
                  */

        num_points = updateNumPoints();

        Q_EMIT refreshTPM(topic_name);
}

TopicPathPtr  TopicPathManager::getCurrentPath() const
{
        return current_path;
}

void TopicPathManager::updateReferencePath(TopicPathPtr new_ref_tp)
{
        if(new_ref_tp == TopicPathPtr())
                return;

        current_ref_path = new_ref_tp;
        updateData(current_path);
}

QString TopicPathManager::getTopicName() const
{
        return topic_name;
}

//TODO testing
double TopicPathManager::updatePathLen()
{
        double len = 0;
        QList<cv::Point3d > points = current_path->getAllPointsOrdered();

        if(points.size() >= 2)
        {
                QList<cv::Point3d >::const_iterator it;
                for(it = points.constBegin(); it < (points.constEnd()-1); ++it)
                {
                        cv::Point3d p(*it-*(it+1));
                        len += sqrt(p.ddot(p));
                }
        }

        return len;
}

double TopicPathManager::getPathLength() const
{
        return pathlength;
}

double TopicPathManager::updateMedian(TopicPathPtr ref_path)
{
        QList<Position>::const_iterator it_current, it_ref1, it_ref2;
        double k;
        cv::Point3d section, dist;
        QList<double> distan;


        if(ref_path == TopicPathPtr() || ref_path->points.size() < 2)
                return 0;


        it_ref1 = ref_path->points.constBegin();
        it_ref2 = it_ref1 + 1;


        std::cout << topic_name.toLocal8Bit().constData() << std::endl;
        for(it_current = current_path->points.constBegin(); it_current < current_path->points.constEnd(); ++it_current)
        {
                //slide compare section in reference path
                while(((*it_ref2) < (*it_current)) && (it_ref2 < (ref_path->points.constEnd() - 1)))
                {
                        std::cout << "slide compare section" << std::endl;
                        ++it_ref1;
                        ++it_ref2;
                }
                section = (*it_ref1).point - (*it_ref2).point;
                double len_squared = section.ddot(section);

                if(len_squared > 0)
                        k = section.ddot((*it_current).point - (*it_ref1).point) / len_squared;
                else
                        k = -2.0;

                std::cout << "k: " << k << std::endl;

                if(k < 0)
                {
                        dist =  (*it_ref2).point - (*it_current).point;
                        distan << sqrt(dist.ddot(dist));
                        std::cout << distan.last() << std::endl;
                }

                else if( k > 1)
                {
                        dist =  (*it_ref1).point - (*it_current).point;
                        distan << sqrt(dist.ddot(dist));
                        std::cout << distan.last() << std::endl;
                }
                else
                {
                        dist = ((*it_ref1).point + (section*k)) - (*it_current).point;
                        distan << sqrt(dist.ddot(dist));
                        std::cout << distan.last() << std::endl;
                }
        }

        qSort(distan);

        distances = distan;

        /*
        std::cout << "print list size: " << distan.size() << std::endl;

        Q_FOREACH(const double &d, distances)
        {
                std::cout << d << std::endl;
        }
        */

        return distan.at(distan.size()/2);
}

double TopicPathManager::getMedian() const
{
        return median;
}

QList<double> TopicPathManager::getDistances() const
{
        return distances;
}

int TopicPathManager::updateNumPoints()
{
        return current_path->points.size();
}

int TopicPathManager::getNumPoints() const
{
        return num_points;
}
