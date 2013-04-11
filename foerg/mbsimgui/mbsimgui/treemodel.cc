#include <QtGui>

#include "treeitem.h"
#include "treemodel.h"
#include "solver.h"
#include "group.h"
#include "frame.h"
#include "rigidbody.h"
#include "joint.h"

#include <iostream>

using namespace std;

TreeModel::TreeModel(const QStringList &headers, QObject *parent) : QAbstractItemModel(parent), IDcounter(0) {

  QVector<QVariant> rootData;
  foreach (QString header, headers)
    rootData << header;

  rootItem = new TreeItem(new BasicItemData("Name","Type"));
}

TreeModel::~TreeModel() {
  delete rootItem;
}

QVariant TreeModel::data(const QModelIndex &index, int role) const {
  if(role==Qt::DisplayRole || role==Qt::EditRole) {
    TreeItem *item = getItem(index);
    return item->getData(index.column());
  } 
  else if(role==Qt::ForegroundRole) {
    if(getItem(index)->getData1()=="") {
      QPalette palette;
      QBrush brush=palette.brush(QPalette::Disabled, QPalette::Text);
      return brush;
    }
  }
  return QVariant();
}

Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const {
  if (!index.isValid())
    return 0;

  return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

TreeItem *TreeModel::getItem(const QModelIndex &index) const {
  if (index.isValid()) {
    TreeItem *item = static_cast<TreeItem*>(index.internalPointer());
    if (item) return item;
  }
  return rootItem;
}

QVariant TreeModel::headerData(int section, Qt::Orientation orientation, int role) const {
  if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    return rootItem->getData(section);

  return QVariant();
}

QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const {
  if (parent.isValid() && parent.column() != 0)
    return QModelIndex();

  TreeItem *parentItem = getItem(parent);

  TreeItem *childItem = parentItem->child(row);
  if (childItem)
    return createIndex(row, column, childItem);
  else
    return QModelIndex();
}

void TreeModel::addFrame(const QModelIndex &parent) {
  FixedRelativeFrame *frame = new FixedRelativeFrame("P"+toStr(IDcounter++),static_cast<Element*>(getItem(parent)->getItemData()));
  static_cast<Element*>(getItem(parent)->getItemData())->addFrame(frame);
  createFrameItem(frame,parent.child(0,0));
}

void TreeModel::addSolver(const QModelIndex &parent) {
  Solver *solver = new Solver("MBS",0);
  createGroupItem(solver,parent);
}

void TreeModel::addGroup(const QModelIndex &parent) {
  Group *group = new Group("Group"+toStr(IDcounter++),static_cast<Element*>(getItem(parent)->getItemData()));
  static_cast<Element*>(getItem(parent)->getItemData())->addGroup(group);
  createGroupItem(group,parent.child(2,0));
}

void TreeModel::addRigidBody(const QModelIndex &parent) {
  RigidBody *rigidbody = new RigidBody("RigidBody"+toStr(IDcounter++),static_cast<Element*>(getItem(parent)->getItemData()));
  static_cast<Element*>(getItem(parent)->getItemData())->addObject(rigidbody);
  createObjectItem(rigidbody,parent.child(3,0));
}

void TreeModel::addJoint(const QModelIndex &parent) {
  Joint *joint = new Joint("Joint"+toStr(IDcounter++),static_cast<Element*>(getItem(parent)->getItemData()));
  static_cast<Element*>(getItem(parent)->getItemData())->addLink(joint);
  createLinkItem(joint,parent.child(4,0));
}

void TreeModel::removeElement(const QModelIndex &index) {
  Element *element = static_cast<Element*>(getItem(index)->getItemData());
  element->getParent()->removeElement(element);
  removeRow(index.row(), index.parent());
}

void TreeModel::createGroupItem(Group *group, const QModelIndex &parent) {

  TreeItem *parentItem = getItem(parent);

  int i = rowCount(parent);
  beginInsertRows(parent, i, i);
  TreeItem *item = new TreeItem(group,parentItem);
  parentItem->insertChildren(item,1);
  endInsertRows();

  QModelIndex index;
  if(parent.row()==-1)
    index = this->index(0,0,parent);
  else
    index = parent.child(i,0);
  i = rowCount(index);
  beginInsertRows(index, i, i+4);
  item->insertChildren(new TreeItem(new BasicItemData("frames",""),item),1);
  item->insertChildren(new TreeItem(new BasicItemData("contours",""),item),1);
  item->insertChildren(new TreeItem(new BasicItemData("groups",""),item),1);
  item->insertChildren(new TreeItem(new BasicItemData("objects",""),item),1);
  item->insertChildren(new TreeItem(new BasicItemData("links",""),item),1);
  item->insertChildren(new TreeItem(new BasicItemData("observers",""),item),1);
  endInsertRows();
  i = rowCount(index);

  for(int i=0; i<group->getNumberOfFrames(); i++)
    createFrameItem(group->getFrame(i),index.child(0,0));
  for(int i=0; i<group->getNumberOfObjects(); i++)
    createObjectItem(group->getObject(i),index.child(3,0));
  for(int i=0; i<group->getNumberOfLinks(); i++)
    createLinkItem(group->getLink(i),index.child(4,0));
}

void TreeModel::createObjectItem(Object *object, const QModelIndex &parent) {

  TreeItem *parentItem = getItem(parent);

  int i = rowCount(parent);
  beginInsertRows(parent, i, i);
  TreeItem *item = new TreeItem(object,parentItem);
  parentItem->insertChildren(item,1);
  endInsertRows();

  idEleMap.insert(make_pair(object->getID(), parent.child(i,0)));

  QModelIndex index = parent.child(i,0);
  i = rowCount(index);
  beginInsertRows(index, i, i+1);
  item->insertChildren(new TreeItem(new BasicItemData("frames",""),item),1);
  item->insertChildren(new TreeItem(new BasicItemData("contours",""),item),1);
  endInsertRows();
  i = rowCount(index);

  for(int i=0; i<object->getNumberOfFrames(); i++)
    createFrameItem(object->getFrame(i),index.child(0,0));
}

void TreeModel::createLinkItem(Link *link, const QModelIndex &parent) {

  TreeItem *parentItem = getItem(parent);

  int i = rowCount(parent);
  beginInsertRows(parent, i, i);
  TreeItem *item = new TreeItem(link,parentItem);
  parentItem->insertChildren(item,1);
  endInsertRows();

  idEleMap.insert(make_pair(link->getID(), parent.child(i,0)));
}

void TreeModel::createFrameItem(Frame *frame, const QModelIndex &parent) {

  TreeItem *parentItem = getItem(parent);

  int i = rowCount(parent);
  beginInsertRows(parent, i, i);
  TreeItem *item = new TreeItem(frame,parentItem);
  parentItem->insertChildren(item,1);
  endInsertRows();

  idEleMap.insert(make_pair(frame->getID(), parent.child(i,0)));
}

QModelIndex TreeModel::parent(const QModelIndex &index) const {
  if (!index.isValid())
    return QModelIndex();

  TreeItem *childItem = getItem(index);
  TreeItem *parentItem = childItem->parent();

  if (parentItem == rootItem)
    return QModelIndex();

  return createIndex(parentItem->childNumber(), 0, parentItem);
}

bool TreeModel::removeRows(int position, int rows, const QModelIndex &parent) {
  TreeItem *parentItem = getItem(parent);
  bool success = true;

  beginRemoveRows(parent, position, position + rows - 1);
  success = parentItem->removeChildren(position, rows);
  endRemoveRows();

  return success;
}

int TreeModel::rowCount(const QModelIndex &parent) const {
  return getItem(parent)->childCount();
}

bool TreeModel::setData(const QModelIndex &index, const QVariant &value, int role) {
  if (role != Qt::EditRole)
    return false;

  TreeItem *item = getItem(index);
  if(index.column()==0)
    item->setData0(value);

  emit dataChanged(index, index);

  return true;
}

bool TreeModel::setHeaderData(int section, Qt::Orientation orientation,
    const QVariant &value, int role)
{
  if (role != Qt::EditRole || orientation != Qt::Horizontal)
    return false;

  if(section==0)
    rootItem->setData0(value);
  else
    rootItem->setData1(value);

  emit headerDataChanged(orientation, section, section);

  return true;
}
