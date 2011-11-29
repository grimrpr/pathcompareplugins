#include "topicpathmanager.h"

TopicPathManager::TopicPathManager(QString topic_name, QObject *parent) :
    QObject(parent),
    topic_name(topic_name)
{
}

void TopicPathManager::processNewPathMsg(const nav_msgs::PathConstPtr &path)
{
        lock_current_path.lockForWrite();
        current_path = TopicPathPtr(new TopicPath(path));
        lock_current_path.unlock();

        updateData(current_path);
}

void TopicPathManager::updateData(const TopicPathPtr &topic_path)
{
        std::cout << "update topic: " << topic_name.toLocal8Bit().constData() << std::endl;

        pathlength = updatePathLen();

        median = updateMedian(current_ref_path);

        num_points = updateNumPoints();

        s_2 = updateS2AndArithMean();

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

double TopicPathManager::updateS2AndArithMean()
{
        //calculate arithmetical mean
        double arith_mean_ = 0;

        lock_pos_dist_map.lockForRead();
        const QMap<Position, double> pos_dist_map_ = pos_dist_map;
        lock_pos_dist_map.unlock();

        QMap<Position, double>::const_iterator it;

        for(it = pos_dist_map_.constBegin(); it != pos_dist_map_.constEnd(); ++it)
        {
                arith_mean_ += it.value();
        }

        arith_mean = arith_mean_ / pos_dist_map_.size();

        double s_2_ = 0;
        double diff;
        for(it = pos_dist_map_.constBegin(); it != pos_dist_map_.constEnd(); ++it)
        {
                diff = (it.value() - arith_mean);
                s_2_ += diff*diff;
        }

        s_2_ /= (pos_dist_map_.size() - 1);

        return s_2_;
}

double TopicPathManager::getS2() const
{
        return s_2;
}

double TopicPathManager::getS() const
{
        return sqrt(s_2);
}

double TopicPathManager::getArithMean() const
{
        return arith_mean;
}

//TODO testing
double TopicPathManager::updatePathLen()
{
        double len = 0;
        lock_current_path.lockForRead();
        QList<cv::Point3d > points = current_path->getAllPointsOrdered();
        lock_current_path.unlock();

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


        lock_current_path.lockForRead();
        lock_pos_dist_map.lockForWrite();
        for(it_current = current_path->points.constBegin(); it_current < current_path->points.constEnd(); ++it_current)
        {
                //slide compare section in reference path
                while(((*it_ref2) < (*it_current)) && (it_ref2 < (ref_path->points.constEnd() - 1)))
                {
                        ++it_ref1;
                        ++it_ref2;
                }
                section = (*it_ref1).point - (*it_ref2).point;
                double len_squared = section.ddot(section);

                if(len_squared > 0)
                        k = section.ddot((*it_current).point - (*it_ref1).point) / len_squared;
                else
                        k = -2.0;

                if(k < 0)
                {
                        dist =  (*it_ref2).point - (*it_current).point;
//                        std::cout << distan.last() << std::endl;
                }

                else if( k > 1)
                {
                        dist =  (*it_ref1).point - (*it_current).point;
//                        std::cout << distan.last() << std::endl;
                }
                else
                {
                        dist = ((*it_ref1).point + (section*k)) - (*it_current).point;
//                        std::cout << distan.last() << std::endl;
                }

                distan << sqrt(dist.ddot(dist));
                pos_dist_map[*it_current] = distan.last();
        }
        lock_pos_dist_map.unlock();
        lock_current_path.unlock();

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

void TopicPathManager::writeCSVstring(std::stringstream &outstr) const
{
        //write table of lines: topicname;pathlength;mediandist;timestamp;pointx;pointy;pointz;distref
        std::cout << "LENGTH OF pos_dist_map: " << pos_dist_map.size() << std::endl;

        writeDataToStream(outstr, pos_dist_map, pathlength, median);

}

void TopicPathManager::writeDataToStream(std::stringstream &outstr,
                                         QMap<Position, double> pos_dist_map_,
                                         double pathlength_,
                                         double median_) const
{
    //write table of lines: topicname;pathlength;mediandist;timestamp;pointx;pointy;pointz;distref
    QMap<Position, double>::const_iterator it;
    for(it = pos_dist_map_.constBegin(); it != pos_dist_map_.constEnd(); ++it)
    {
            ros::Time time = it.key().timestamp;

            outstr << topic_name.toLocal8Bit().constData() << ";";
            outstr << pathlength_ << ";";
            outstr << median_<< ";";
            outstr << time.toNSec() << ";";
            outstr << it.key().point.x << ";";
            outstr << it.key().point.y << ";";
            outstr << it.key().point.z << ";";
            outstr << it.value() << std::endl;
    }
}

int TopicPathManager::updateNumPoints()
{
        lock_current_path.lockForRead();
        int result = current_path->points.size();
        lock_current_path.unlock();
        return result;
}

int TopicPathManager::getNumPoints() const
{
        return num_points;
}
