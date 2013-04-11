#ifndef TREEMODEL_H
#define TREEMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

class Frame;
class Object;
class Link;
class Group;
class RigidBody;
class TreeItem;

class TreeModel : public QAbstractItemModel {
  public:
    TreeModel(const QStringList &headers, QObject *parent = 0);
    ~TreeModel();

    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const;
    QModelIndex parent(const QModelIndex &index) const;

    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &) const {return 2;}

    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
    bool setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role = Qt::EditRole);

    bool removeRows(int position, int rows, const QModelIndex &parent = QModelIndex());

    TreeItem *getItem(const QModelIndex &index) const;

    void removeElement(const QModelIndex &parent = QModelIndex());
    void addSolver(const QModelIndex &parent = QModelIndex());
    void addGroup(const QModelIndex &parent = QModelIndex());
    void addRigidBody(const QModelIndex &parent = QModelIndex());
    void addFrame(const QModelIndex &parent = QModelIndex());
    void addJoint(const QModelIndex &parent = QModelIndex());
    void createGroupItem(Group *group, const QModelIndex &parent = QModelIndex());
    void createObjectItem(Object *object, const QModelIndex &parent = QModelIndex());
    void createLinkItem(Link *link, const QModelIndex &parent = QModelIndex());
    void createFrameItem(Frame *frame, const QModelIndex &parent = QModelIndex());

    void updateView(const QModelIndex &index) {emit dataChanged(index,index);}

    std::map<std::string, QModelIndex> idEleMap;
    int IDcounter;
 private:
    TreeItem *rootItem;
};

#endif
