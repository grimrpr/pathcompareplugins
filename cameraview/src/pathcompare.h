#ifndef PATHCOMPARE_H
#define PATHCOMPARE_H

//Boost includes
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

//std includes
#include <iostream>
#include <string>

//ROS includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/cache.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

//project include
#include "../../../pathcompare/src/interfaces/comperatorplugin.h"
#include "../../../pathcompare/src/interfaces/rosmanager.h"


#include "ui_formmainplugin.h"

class PathCompare : public ComperatorPlugin
{
        Q_OBJECT

        Ui::Form *form;
        ROSManager *ros_mngr;
        const std::string topic_type_str;
        boost::shared_ptr< message_filters::Cache<sensor_msgs::Image> > topic_cache;
        cv_bridge::CvImageConstPtr cv_ptr;
        message_filters::Connection con;

public:
        PathCompare(ROSManager *ros_mngr, QWidget *tab_widget);

private:
        QImage cvMat2QImage(const cv::Mat& image, unsigned int idx) const;
        void camera_callback(const sensor_msgs::ImageConstPtr &image);

Q_SIGNALS:
        void newPixmap(QImage img);

public Q_SLOTS:
        void updateTopics();
        void topicSelected(const QString &topic_name);
        void updatePixmap(QImage img);

};

#endif // PATHCOMPARE_H
