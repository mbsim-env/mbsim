/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "treemodel.h"
#include "treeitem.h"
#include "basicitemdata.h"
#include "project.h"
#include "dynamic_system_solver.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "body.h"
#include "link_.h"
#include "constraint.h"
#include "observer.h"
#include "solver.h"
#include "parameter.h"
#include "fileitemdata.h"

using namespace std;

namespace MBSimGUI {

  TreeModel::~TreeModel() {
    delete rootItem->getItemData();
    delete rootItem;
  }

  bool TreeModel::setData(const QModelIndex &index, const QVariant &value, int role) {
    if(role==Qt::ForegroundRole) {
      TreeItem *item = getItem(index);
      item->setForeground(value.value<QBrush>());
      return true;
    }
    return QAbstractItemModel::setData(index, value, role);
  }

  QVariant TreeModel::data(const QModelIndex &index, int role) const {
    if(role==Qt::DisplayRole || role==Qt::EditRole) {
      TreeItem *item = getItem(index);
      return item->getData(index.column());
    } 
    else if(role==Qt::DecorationRole and index.column()==0) {
      TreeItem *item = getItem(index);
      return item->getDecoration();
    }
    else if(role==Qt::ForegroundRole) {
      TreeItem *item = getItem(index);
      return item->getForeground();
    }
    else if(role==Qt::BackgroundRole) {
      TreeItem *item = getItem(index);
      return item->getBackground();
    }
    else if(role==Qt::FontRole) {
      TreeItem *item = getItem(index);
      return item->getFont();
    }
    else if(role==Qt::UserRole) {
      TreeItem *item = getItem(index);
      return item->getItemData()->getEnabled();
    }
    return QVariant();
  }

  Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const {
    if(!index.isValid())
      return Qt::NoItemFlags;

    return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  }

  TreeItem* TreeModel::getItem(const QModelIndex &index) const {
    if(index.isValid()) {
      auto *item = static_cast<TreeItem*>(index.internalPointer());
      if(item) return item;
    }
    return rootItem;
  }

  QModelIndex TreeModel::findItem(const TreeItemData *item, const QModelIndex &parentIndex) const {
    QModelIndex index;
    if(getItem(parentIndex)->getItemData()==item)
      return parentIndex;
    for (int i = 0; i < rowCount(parentIndex); i++) {
      QString name = getItem(this->index(i, 0, parentIndex))->getItemData()->getName();
      index = findItem(item, this->index(i, 0, parentIndex));
      if(getItem(index)->getItemData()==item)
        return index;
    }
    return index;
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

  ElementTreeModel::ElementTreeModel(QObject *parent) : TreeModel(parent) {

    rootItem = new TreeItem(new TreeItemData);
  }

  void ElementTreeModel::createProjectItem(Project *project, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(project,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    project->setModelIndex(this->index(0,0,parent));
    updateProjectItem(project);
  }

  void ElementTreeModel::createFrameItem(Frame *frame, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(frame,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    frame->setModelIndex(parent.model()->index(i,0,parent));
  }

  void ElementTreeModel::createContourItem(Contour *contour, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(contour,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    contour->setModelIndex(parent.model()->index(i,0,parent));
  }

  void ElementTreeModel::createGroupItem(Group *group, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(group,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    QModelIndex index = parent.model()->index(i,0,parent);
    group->setModelIndex(index);
    updateGroupItem(group);
  }

  void ElementTreeModel::createObjectItem(Object *object, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(object,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    QModelIndex index = parent.model()->index(i,0,parent);
    object->setModelIndex(index);
    updateObjectItem(object);
  }

  void ElementTreeModel::createLinkItem(Link *link, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(link,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    link->setModelIndex(parent.model()->index(i,0,parent));
  }

  void ElementTreeModel::createConstraintItem(Constraint *constraint, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(constraint,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    constraint->setModelIndex(parent.model()->index(i,0,parent));
  }

  void ElementTreeModel::createObserverItem(Observer *observer, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(observer,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    observer->setModelIndex(parent.model()->index(i,0,parent));
  }

  void ElementTreeModel::createSolverItem(Solver *solver, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(solver,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    solver->setModelIndex(parent.model()->index(i,0,parent));
  }

  void ElementTreeModel::updateProjectItem(Project *project) {

    createGroupItem(project->getDynamicSystemSolver(),index(0,0));
    createSolverItem(project->getSolver(),index(0,0));
  }

  void ElementTreeModel::updateElementItem(EmbedItemData *element) {
    if(dynamic_cast<Group*>(element))
      updateGroupItem(static_cast<Group*>(element));
    else if(dynamic_cast<Object*>(element))
      updateObjectItem(static_cast<Object*>(element));
    else if(dynamic_cast<Project*>(element))
      updateProjectItem(static_cast<Project*>(element));
  }

  void ElementTreeModel::updateGroupItem(Group *group) {

    QModelIndex index = group->getModelIndex();
    TreeItem *item = getItem(index);
    int i = rowCount(index);
    beginInsertRows(index, i, i+6);
    item->insertChildren(new TreeItem(new FrameItemData(group),item),1);
    item->insertChildren(new TreeItem(new ContourItemData(group),item),1);
    item->insertChildren(new TreeItem(new GroupItemData(group),item),1);
    item->insertChildren(new TreeItem(new ObjectItemData(group),item),1);
    item->insertChildren(new TreeItem(new LinkItemData(group),item),1);
    item->insertChildren(new TreeItem(new ConstraintItemData(group),item),1);
    item->insertChildren(new TreeItem(new ObserverItemData(group),item),1);
    endInsertRows();

    for(int i=0; i<group->getNumberOfFrames(); i++)
      createFrameItem(group->getFrame(i),index.model()->index(0,0,index));
    for(int i=0; i<group->getNumberOfContours(); i++)
      createContourItem(group->getContour(i),index.model()->index(1,0,index));
    for(int i=0; i<group->getNumberOfGroups(); i++)
      createGroupItem(group->getGroup(i),index.model()->index(2,0,index));
    for(int i=0; i<group->getNumberOfObjects(); i++)
      createObjectItem(group->getObject(i),index.model()->index(3,0,index));
    for(int i=0; i<group->getNumberOfLinks(); i++)
      createLinkItem(group->getLink(i),index.model()->index(4,0,index));
    for(int i=0; i<group->getNumberOfConstraints(); i++)
      createConstraintItem(group->getConstraint(i),index.model()->index(5,0,index));
    for(int i=0; i<group->getNumberOfObservers(); i++)
      createObserverItem(group->getObserver(i),index.model()->index(6,0,index));
  }

  void ElementTreeModel::updateObjectItem(Object *object) {

    if(dynamic_cast<Body*>(object)) {
      QModelIndex index = object->getModelIndex();
      TreeItem *item = getItem(index);
      int i = rowCount(index);
      beginInsertRows(index, i, i+1);
      item->insertChildren(new TreeItem(new FrameItemData(object),item),1);
      item->insertChildren(new TreeItem(new ContourItemData(object),item),1);
      endInsertRows();

      for(int i=0; i<object->getNumberOfFrames(); i++)
        createFrameItem(object->getFrame(i),index.model()->index(0,0,index));
      for(int i=0; i<object->getNumberOfContours(); i++)
        createContourItem(object->getContour(i),index.model()->index(1,0,index));
    }
  }

  ParameterTreeModel::ParameterTreeModel(QObject *parent) : TreeModel(parent) {

    rootItem = new TreeItem(new TreeItemData);
  }

  void ParameterTreeModel::createParameterItem(Parameters *parameters, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(parameters,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    QModelIndex index;
    if(parent.row()==-1)
      index = this->index(0,0,parent);
    else
      index = parent.model()->index(i,0,parent);

    parameters->setModelIndex(index);
    updateParameterItem(parameters);
  }

  void ParameterTreeModel::updateParameterItem(Parameters *parameters) {

    QModelIndex index = parameters->getModelIndex();

    for(int i=0; i<parameters->getParent()->getNumberOfParameters(); i++)
      createParameterItem(parameters->getParent()->getParameter(i),index);
  }

  void ParameterTreeModel::createParameterItem(Parameter *parameter, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    if(dynamic_cast<Parameters*>(getItem(parent.model()->index(i-1,0,parent))->getItemData())) i--;
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(parameter,parentItem);
    parentItem->insertChildren(item,i,1);
    endInsertRows();

    parameter->setModelIndex(parent.model()->index(i,0,parent));
    parameter->updateValue();
  }

  FileTreeModel::FileTreeModel(QObject *parent) : TreeModel(parent) {

    rootItem = new TreeItem(new TreeItemData);
  }

  void FileTreeModel::createFileItem(FileItemData *fileItemData, const QModelIndex &parent) {

    TreeItem *parentItem = getItem(parent);

    int i = rowCount(parent);
    beginInsertRows(parent, i, i);
    TreeItem *item = new TreeItem(fileItemData,parentItem);
    parentItem->insertChildren(item,1);
    endInsertRows();

    fileItemData->setModelIndex(this->index(i,0,parent));
  }

}
