/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include "treemodel.h"
#include "treeitem.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "object.h"
#include "link.h"
#include "constraint.h"
#include "observer.h"
#include "parameter.h"
#include <iostream>

using namespace std;

namespace MBSimGUI {

  TreeModel::TreeModel(QObject *parent) : QAbstractItemModel(parent), rootItem(0) {
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
      TreeItem *item = getItem(index);
      return item->getForeground();
    }
    else if(role==Qt::UserRole) {
      TreeItem *item = getItem(index);
      return item->getEnabled();
    }
    return QVariant();
  }

  Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const {
    if(!index.isValid())
      return Qt::NoItemFlags;

    return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  }

  TreeItem *TreeModel::getItem(const QModelIndex &index) const {
    if(index.isValid()) {
      TreeItem *item = static_cast<TreeItem*>(index.internalPointer());
      if(item) return item;
    }
    return rootItem;
  }

  QVariant TreeModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if(orientation == Qt::Horizontal && role == Qt::DisplayRole)
      return rootItem->getData(section);

    return QVariant();
  }

  QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const {
    if(parent.isValid() && parent.column() != 0)
      return QModelIndex();

    TreeItem *parentItem = getItem(parent);

    TreeItem *childItem = parentItem->child(row);
    if(childItem)
      return createIndex(row, column, childItem);
    else
      return QModelIndex();
  }

  QModelIndex TreeModel::parent(const QModelIndex &index) const {
    if(!index.isValid())
      return QModelIndex();

    TreeItem *childItem = getItem(index);
    TreeItem *parentItem = childItem->parent();

    if(parentItem == rootItem)
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
    if(role != Qt::EditRole && role != Qt::ForegroundRole)
      return false;

    TreeItem *item = getItem(index);
    if(role == Qt::EditRole && index.column()==0)
      item->setData0(value);
    if(role == Qt::ForegroundRole && index.column()==0)
      item->setForeground(value.value<QBrush>());

    emit dataChanged(index, index);

    return true;
  }

  bool TreeModel::setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role) {
    if(role != Qt::EditRole || orientation != Qt::Horizontal)
      return false;

    rootItem->setData(section,value);

    emit headerDataChanged(orientation, section, section);

    return true;
  }

  ElementTreeModel::ElementTreeModel(QObject *parent) : TreeModel(parent) {

    rootItem = new TreeItem(new BasicItemData("Name","Value"));
  }

  void ElementTreeModel::createFrameItem(Frame *frame, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(frame,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    idEleMap.insert(make_pair(frame->getID(), parent.child(i,0)));
  }

  void ElementTreeModel::createContourItem(Contour *contour, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(contour,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    idEleMap.insert(make_pair(contour->getID(), parent.child(i,0)));
  }

  void ElementTreeModel::createGroupItem(Group *group, const QModelIndex &parent) {

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
    beginInsertRows(index, i, i+6);
    item->insertChildren(new TreeItem(new FrameItemData(group),item,0,Qt::gray),1);
    item->insertChildren(new TreeItem(new ContourItemData(group),item,1,Qt::gray),1);
    item->insertChildren(new TreeItem(new GroupItemData(group),item,1,Qt::gray),1);
    item->insertChildren(new TreeItem(new ObjectItemData(group),item,1,Qt::gray),1);
    item->insertChildren(new TreeItem(new LinkItemData(group),item,1,Qt::gray),1);
    item->insertChildren(new TreeItem(new ConstraintItemData(group),item,1,Qt::gray),1);
    item->insertChildren(new TreeItem(new ObserverItemData(group),item,1,Qt::gray),1);
    endInsertRows();
    i = rowCount(index);

    for(int i=0; i<group->getNumberOfFrames(); i++)
      createFrameItem(group->getFrame(i),index.child(0,0));
    for(int i=0; i<group->getNumberOfContours(); i++)
      createContourItem(group->getContour(i),index.child(1,0));
    for(int i=0; i<group->getNumberOfGroups(); i++)
      createGroupItem(group->getGroup(i),index.child(2,0));
    for(int i=0; i<group->getNumberOfObjects(); i++)
      createObjectItem(group->getObject(i),index.child(3,0));
    for(int i=0; i<group->getNumberOfLinks(); i++)
      createLinkItem(group->getLink(i),index.child(4,0));
    for(int i=0; i<group->getNumberOfConstraints(); i++)
      createConstraintItem(group->getConstraint(i),index.child(5,0));
    for(int i=0; i<group->getNumberOfObservers(); i++)
      createObserverItem(group->getObserver(i),index.child(6,0));
  }

  void ElementTreeModel::createObjectItem(Object *object, const QModelIndex &parent) {

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
    item->insertChildren(new TreeItem(new FrameItemData(object),item,0,Qt::gray),1);
    item->insertChildren(new TreeItem(new ContourItemData(object),item,1,Qt::gray),1);
    endInsertRows();
    i = rowCount(index);

    for(int i=0; i<object->getNumberOfFrames(); i++)
      createFrameItem(object->getFrame(i),index.child(0,0));
    for(int i=0; i<object->getNumberOfContours(); i++)
      createContourItem(object->getContour(i),index.child(1,0));
  }

  void ElementTreeModel::createLinkItem(Link *link, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(link,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    idEleMap.insert(make_pair(link->getID(), parent.child(i,0)));
  }

  void ElementTreeModel::createConstraintItem(Constraint *constraint, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(constraint,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    idEleMap.insert(make_pair(constraint->getID(), parent.child(i,0)));
  }

  void ElementTreeModel::createObserverItem(Observer *observer, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(observer,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    idEleMap.insert(make_pair(observer->getID(), parent.child(i,0)));
  }

  EmbeddingTreeModel::EmbeddingTreeModel(QObject *parent) : TreeModel(parent) {

    rootItem = new TreeItem(new BasicItemData("Name","Value"));
  }

  void EmbeddingTreeModel::createElementItem(Element *element, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(element,parentItem,1,Qt::gray);
    parentItem->insertChildren(item,1);
    endInsertRows();

    QModelIndex index;
    if(parent.row()==-1)
      index = this->index(0,0,parent);
    else
      index = parent.child(i,0);
    i = rowCount(index);

    idEleMap.insert(make_pair(element, index));

    for(int i=0; i<element->getNumberOfParameters(); i++)
      createParameterItem(element->getParameter(i),index);
    for(int i=0; i<element->getNumberOfFrames(); i++)
      createElementItem(element->getFrame(i),index);
    for(int i=0; i<element->getNumberOfContours(); i++)
      createElementItem(element->getContour(i),index);
    for(int i=0; i<element->getNumberOfGroups(); i++)
      createElementItem(element->getGroup(i),index);
    for(int i=0; i<element->getNumberOfObjects(); i++)
      createElementItem(element->getObject(i),index);
    for(int i=0; i<element->getNumberOfLinks(); i++)
      createElementItem(element->getLink(i),index);
    for(int i=0; i<element->getNumberOfConstraints(); i++)
      createElementItem(element->getConstraint(i),index);
    for(int i=0; i<element->getNumberOfObservers(); i++)
      createElementItem(element->getObserver(i),index);
  }

  QModelIndex EmbeddingTreeModel::createEmbeddingItem(TreeItemData *itemData, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    //TreeItem *item = new TreeItem(new Embedding(element),parentItem,1,Qt::gray);
    TreeItem *item = new TreeItem(itemData,parentItem,1,Qt::gray);
    parentItem->insertChildren(item,1);
    endInsertRows();

    QModelIndex index;
    if(parent.row()==-1)
      index = this->index(0,0,parent);
    else
      index = parent.child(i,0);
    i = rowCount(index);

    for(int i=0; i<itemData->getNumberOfParameters(); i++)
      createParameterItem(itemData->getParameter(i),index);

    return index;
  }

  QModelIndex EmbeddingTreeModel::createParameterItem(Parameter *parameter, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    if(dynamic_cast<Element*>(getItem(parent.child(i-1,0))->getItemData())) i--;
    beginInsertRows(parent, i, i);

    TreeItem *item = new TreeItem(parameter,parentItem);
    parentItem->insertChildren(item,i,1);
    endInsertRows();
    return parent.child(i,0);
  }


}
