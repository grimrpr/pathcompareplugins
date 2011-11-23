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
        std::cout << "The path of topic: "
                  << topic_name.toLocal8Bit().constData()
                  << " is now: " << pathlength
                  << " long. " << std::endl;

        median = updateMedian(current_ref_path);
        std::cout << "The path of topic: "
                  << topic_name.toLocal8Bit().constData()
                  << " has now: " << median
                  << " as median devergence to reference path. " << std::endl;

}

TopicPathPtr  TopicPathManager::getCurrentPath() const
{
        return current_path;
}

void TopicPathManager::updateReferencePath(TopicPathPtr new_ref_tp)
{
        current_ref_path = new_ref_tp;
}

QString TopicPathManager::getTopicName() const
{
        return topic_name;
}

//TODO testing
double TopicPathManager::updatePathLen()
{
        double len = 0;
        QList<QVector3D> points = current_path->getAllPointsOrdered();

        if(points.size() >= 2)
        {
                QList<QVector3D>::const_iterator it;
                for(it = points.constBegin(); it < (points.constEnd()-1); ++it)
                        len += (*it-*(it+1)).length();
        }

        return len;
}

double TopicPathManager::updateMedian(TopicPathPtr ref_path)
{
        QList<Position>::const_iterator it_current, it_ref1, it_ref2;
        double k;
        if(ref_path == TopicPathPtr() || ref_path->points.size() < 2)
                return 0;

        distances.clear();

        it_ref1 = ref_path->points.constBegin();
        it_ref2 = it_ref1 + 1;

        QVector3D section;

        for(it_current = current_path->points.constBegin(); it_current < current_path->points.constEnd(); ++it_current)
        {
                //slide compare section in reference path
                while(((*it_ref2) < (*it_current)) && (it_ref2 < (ref_path->points.constEnd() - 1)))
                {
                        ++it_ref1;
                        ++it_ref2;
                }
                section = (*it_ref1).point - (*it_ref2).point;

                k = QVector3D::dotProduct(section, (*it_current).point) / section.lengthSquared();

                if(k < 0.0)
                        distances << ((*it_ref2).point.distanceToLine((*it_current).point, QVector3D()));
                else if( k > 1.0)
                        distances << ((*it_ref1).point.distanceToLine((*it_current).point, QVector3D()));
                else
                        distances << (*it_current).point.distanceToLine((*it_ref2).point, section);
        }

        std::cout << "Distances from topic: " << topic_name.toLocal8Bit().constData()
                  << "to reference path" << std::endl;

        for(int i = 0; i < distances.size(); ++i)
                std::cout << distances.at(i) << std::endl;

        qSort(distances);
        return distances.at(distances.size()/2);
}