#include "pathcompare.h"


PathCompare::PathCompare(ROSManager *ros_mngr ,QWidget * tab_widget) :
        ComperatorPlugin(),
        form(new Ui::Form),
        ros_mngr(ros_mngr),
        topic_type_str("sensor_msgs/Image")

{
        form->setupUi(tab_widget);
        connect(form->topicComboBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(topicSelected(QString)));

        updateTopics();

        //connect to ros_mngr topic update tick
        connect(ros_mngr, SIGNAL(updateModel()), this, SLOT(updateTopics()));

        //connect camera_callback to pixmap update slot
        connect(this, SIGNAL(newPixmap(QImage)), this, SLOT(updatePixmap(QImage)));
}

//void PathCompare::camera_callback(const sensor_msgs::ImageConstPtr &image)
void PathCompare::camera_callback(const sensor_msgs::ImageConstPtr &image)
{

        cv_bridge::CvImageConstPtr cv_ptr;

        try
        {
          if (sensor_msgs::image_encodings::isColor(image->encoding))
            cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::RGB8);
          else
            cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        QImage frame((uchar *) cv_ptr->image.data,
                     cv_ptr->image.size().width,
                     cv_ptr->image.size().height,
                     cv_ptr->image.step,
                     QImage::Format_RGB888);

                        /*
        sensor_msgs::CvBridge bridge;
        std::cout << "not converted image!: " << image->encoding<< std::endl;
        cv::Mat img = bridge.imgMsgToCv(image);
        std::cout << "converted img to cv::Mat with size : " << img.size << std::endl;
        QImage frame = cvMat2QImage(img, 0) ;
                        */

        Q_EMIT newPixmap(frame.copy());

}

void PathCompare::updatePixmap(QImage frame)
{
        form->labelCameraView->setPixmap(QPixmap::fromImage(frame));
}

void PathCompare::topicSelected(const QString &topic_name)
{
        std::cout << "new topic: " << topic_name.toLocal8Bit().constData() << " selected" << std::endl;

        //disconnect old callback
        con.disconnect();

        //unsubscribe from topic
        //subscribe and get cache
        topic_cache = ros_mngr->subscribeToTopic<sensor_msgs::Image>(topic_name.toLocal8Bit().constData(),
                                                                     5,
                                                                     static_cast<ComperatorPlugin*>(this));

        //register new callback
        con = topic_cache->registerCallback(boost::bind(&PathCompare::camera_callback, this, _1));
}

void PathCompare::updateTopics()
{
        QStringList current_topics_lst = ros_mngr->getTopicNamesOfType(topic_type_str);

        //delete old topics that are not available anymore
        int old_topic_count = form->topicComboBox->count();
        for(int i = 0; i < old_topic_count; ++i)
        {
                if(!current_topics_lst.contains(form->topicComboBox->itemText(i)))
                {
                        form->topicComboBox->removeItem(i);
                }
        }

        if(current_topics_lst.size() > 0)
        {

                Q_FOREACH(QString str, current_topics_lst)
                {
                        //if not in selection keep track of this topic
                        //we never remove because we want to keep information of a topic even if it is deleted
                        if(form->topicComboBox->findText(str) < 0)
                        {
                                //add new item to ComboBox
                                form->topicComboBox->addItem(str);
                        }
                }
        }
}


QImage PathCompare::cvMat2QImage(const cv::Mat& image, unsigned int idx) const
{
  std::vector<cv::Mat> rgba_buffers_;

  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }

  if(image.rows != rgba_buffers_[idx].rows || image.cols != rgba_buffers_[idx].cols){
    rgba_buffers_[idx] = cv::Mat( image.rows, image.cols, CV_8UC4); 
  }

  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance

  cv::Mat in[] = { image, alpha };
  // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
  // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
  int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
  mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  //cv::cvtColor(image, rgba_buffers_, CV_GRAY2RGBA);}
  //}
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}

//template boost::shared_ptr< message_filters::Cache<sensor_msgs::Image> > ROSManager::subscribeToTopic (const std::string &, const unsigned int, ComperatorPlugin*);
