#ifndef GRAPHTABLEMODEL_H
#define GRAPHTABLEMODEL_H

//QT includes
#include <QAbstractTableModel>
#include <QList>
#include <QStringList>
#include <QMap>

//project includes
#include "topicpathmanager.h"

/**
  The GraphTableModel follows the Qt model-view concept by implementing the model for the Connections view.
  Therefore it inherits from QAbstractTableModel to implement its interface.
  For more details on the Qt model-view concept see http://doc.qt.nokia.com/latest/model-view-programming.html
  */
class GraphTableModel : public QAbstractTableModel
{
        Q_OBJECT

        QList<TopicPathManagerPtr> tpm_list;
        QList<QString> analysis_header_data;

private:
        int num_data_entry;


public:
    /**
      constructor
      */
    GraphTableModel(const QList<TopicPathManagerPtr> &tpm_list, QObject *parent=0):
            QAbstractTableModel(parent)
    {
            analysis_header_data << "total number of points"
                                 << "pathlength"
                                 << "median distance to reference"
                                 << "mean distance to reference"
                                 << "s squared (variance)"
                                 << "s";

            num_data_entry = analysis_header_data.size();

            //add strings for greatest 20 distances
            for(int i = 1; i <= 20; ++i)
                    analysis_header_data << QString().setNum(i);

            updataTPMList(tpm_list);
    }


    /**
      implementation of interface function as defined in QAbstractTableModel
      */
    int rowCount(const QModelIndex &parent = QModelIndex()) const;

    /**
      implementation of interface function as defined in QAbstractTableModel
      */
    int columnCount(const QModelIndex &parent = QModelIndex()) const;

    /**
      implementation of interface function as defined in QAbstractTableModel
      */
    QVariant data(const QModelIndex &index, int role) const;

    /**
      implementation of interface function as defined in QAbstractTableModel
      */
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;

    /**
      implementation of interface function as defined in QAbstractTableModel
      */
    Qt::ItemFlags flags(const QModelIndex &index) const;

public Q_SLOTS:
    void updataTPMList(const QList<TopicPathManagerPtr> new_list);
    void updateTPM(const QString topic);
};

#endif // GRAPHTABLEMODEL_H
