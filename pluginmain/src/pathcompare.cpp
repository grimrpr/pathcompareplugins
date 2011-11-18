#include "pathcompare.h"


PathCompare::PathCompare(ROSManager *ros_mngr ,QWidget * tab_widget) :
        ComperatorPlugin(),
        form(new Ui::Form),
        ros_mngr(ros_mngr),
        topic_type_str("nav_msgs/Path")
//        topic_type_str("sensor_msgs/PointCloud2")

{
        form->setupUi(tab_widget);

        updateTopics();

        //connect to ros_mngr topic update tick
        connect(ros_mngr, SIGNAL(updateModel()), this, SLOT(updateTopics()));
}

void PathCompare::updateTopics()
{
        QStringList current_topics_lst = ros_mngr->getTopicNamesOfType(topic_type_str);

        if(current_topics_lst.size() > 0)
                {
                std::cout << "PathCompare -> updateTopics()"
                          << current_topics_lst.first().toLocal8Bit().constData()
                          << std::endl;

                Q_FOREACH(QString str, current_topics_lst)
                {
                        //if not in selection keep track of this topic
                        //we never remove because we want to keep information of a topic even if it is deleted
                        if(form->ReferencePathSelection->findText(str) < 0)
                        {
                                //add new item to ComboBox
                                form->ReferencePathSelection->addItem(str);

                        }
                }
        }
}
