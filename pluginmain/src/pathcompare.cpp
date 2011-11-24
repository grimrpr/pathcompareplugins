#include "pathcompare.h"


PathCompare::PathCompare(ROSManager *ros_mngr ,QWidget * tab_widget) :
        ComperatorPlugin(),
        form(new Ui::Form),
        ros_mngr(ros_mngr),
        topic_type_str("nav_msgs/Path"),
        tpm_list(),
        table_model(new GraphTableModel(tpm_list))

{
        form->setupUi(tab_widget);
        connect(form->ReferencePathSelection, SIGNAL(currentIndexChanged(QString)), this, SLOT(topicSelected(QString)));

        updateTopics();

        //connect to table model update function
        connect(this, SIGNAL(tpmListChanged(QList<TopicPathManagerPtr>)),
                table_model.get(), SLOT(updataTPMList(QList<TopicPathManagerPtr>)));

        //connect to ros_mngr topic update tick
        connect(ros_mngr, SIGNAL(updateModel()), this, SLOT(updateTopics()));

        form->PathInformationTable->setModel(table_model.get());
//        form->PathInformationTable->setModel(new GraphTableModel(tpm_list));
}

void PathCompare::topicSelected(const QString topic_name)
{
        int i;
        for(i = 0; i < tpm_list.size(); ++i)
                if(tpm_list.at(i)->getTopicName() == topic_name)
                        break;

        //shared pointer
        TopicPathManagerPtr tpm = tpm_list.at(i);

        TopicPathPtr ref_tp =  tpm->getCurrentPath();
        QList<TopicPathManagerPtr>::iterator it;
        for(it = tpm_list.begin(); it < tpm_list.end(); ++it)
                (*it)->updateReferencePath(ref_tp);
}

void PathCompare::updateTopics()
{

        QStringList current_topics_lst = ros_mngr->getTopicNamesOfType(topic_type_str);


        if(current_topics_lst.size() > 0)
                {
                std::cout << "PathCompare -> updateTopics()"
                          << current_topics_lst.first().toLocal8Bit().constData()
                          << std::endl;

                Q_FOREACH(const QString &str, current_topics_lst)
                {
                        //if not in selection keep track of this topic
                        //we dont remove old topics because we want to keep information of this topic even if it is deleted
                        if(form->ReferencePathSelection->findText(str) < 0)
                        {

                                TopicPathManagerPtr tpm(new TopicPathManager(str));
                                connect(tpm.get(), SIGNAL(refreshTPM(QString)), table_model.get(), SLOT(updateTPM(QString)));
                                tpm_list.append(tpm);

                                //add new item to ComboBox
                                form->ReferencePathSelection->addItem(str);

                                //subscribe to new topic
                                PathCachePtr cache_ptr = ros_mngr->subscribeToTopic<nav_msgs::Path>(str.toLocal8Bit().constData(),
                                                                                    static_cast<uint>(5),
                                                                                    static_cast<ComperatorPlugin*>(this));

                                //register callback for it
                                connections << cache_ptr->registerCallback(boost::bind(&TopicPathManager::processNewPathMsg,
                                                                                       tpm.get(),
                                                                                       _1));
                        }
                }
                Q_EMIT tpmListChanged(tpm_list);
        }
}
