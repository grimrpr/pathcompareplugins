#include "graphtablemodel.h"

int GraphTableModel::rowCount(const QModelIndex &parent) const{
        if((parent == QModelIndex()))
                return analysis_header_data.size();
        else
                return 0;
}

int GraphTableModel::columnCount(const QModelIndex &parent) const{
        return tpm_list.size();
}

QVariant GraphTableModel::data(const QModelIndex &index, int role) const{

        if (!index.isValid())
                return QVariant();
        if (index.row() >= analysis_header_data.size())
                return QVariant();
        if (index.column() >= tpm_list.size())
                return QVariant();

        if(role == Qt::DisplayRole || role == Qt::EditRole){
                TopicPathManagerPtr tpm = tpm_list.at(index.column());

                switch(index.row())
                {
                //#points
                case 0:
                        return QString().setNum(tpm->getNumPoints());
                //pathlength
                case 1:
                        return QString().setNum(tpm->getPathLength());
                //distance median
                case 2:
                        return QString().setNum(tpm->getMedian());
                }
        }

        return QVariant();
}

QVariant GraphTableModel::headerData(int section, Qt::Orientation orientation, int role) const{
        if(role != Qt::DisplayRole)
                return QVariant();
        if (orientation == Qt::Horizontal)
                return tpm_list.at(section)->getTopicName();
        else
                return analysis_header_data.at(section);
}


Qt::ItemFlags GraphTableModel::flags(const QModelIndex &index) const{
        if(!index.isValid())
                return Qt::ItemIsEnabled;
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}


void GraphTableModel::updataTPMList(const QList<TopicPathManagerPtr> &new_list)
{
        if(tpm_list.size() > 0)
        {
                beginRemoveColumns(QModelIndex(), 0, tpm_list.size()-1);
                tpm_list.clear();
                endRemoveColumns();
        }

        if(new_list.size() > 0)
        {
                beginInsertColumns(QModelIndex(), 0, new_list.size()-1);
                tpm_list = new_list;
                endInsertColumns();
        }
}

void GraphTableModel::updateTPM(const QString &topic)
{

        int i;
        for(i = 0; i < tpm_list.size(); ++i)
        {
                if(tpm_list.at(i)->getTopicName() == topic)
                        break;
        }
        Q_EMIT dataChanged(index(0,i),index(analysis_header_data.size(),i));
}
