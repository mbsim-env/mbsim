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

#ifndef _TREEMODEL_H
#define _TREEMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QBrush>

namespace MBSimGUI {

  class Project;
  class EmbedItemData;
  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;
  class Solver;
  class Parameter;
  class Parameters;
  class TreeItem;
  class TreeItemData;
  class FileItemData;

  class TreeModel : public QAbstractItemModel {
    public:
      TreeModel(QObject *parent = nullptr) : QAbstractItemModel(parent) { }
      ~TreeModel() override;

      QVariant data(const QModelIndex &index, int role) const override;
      QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

      QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
      QModelIndex parent(const QModelIndex &index) const override;

      int rowCount(const QModelIndex &parent = QModelIndex()) const override;
      int columnCount(const QModelIndex &) const override { return 4; }

      Qt::ItemFlags flags(const QModelIndex &index) const override;

      bool removeRows(int position, int rows, const QModelIndex &parent = QModelIndex()) override;

      TreeItem* getItem(const QModelIndex &index) const;
      QModelIndex findItem(const TreeItemData *item, const QModelIndex &parentIndex) const;

    protected:
      TreeItem *rootItem{0};
  };

  class ElementTreeModel : public TreeModel {
    public:
      ElementTreeModel(QObject *parent = nullptr);

      void createProjectItem(Project *project, const QModelIndex &parent = QModelIndex());
      void createFrameItem(Frame *frame, const QModelIndex &parent = QModelIndex());
      void createContourItem(Contour *contour, const QModelIndex &parent = QModelIndex());
      void createGroupItem(Group *group, const QModelIndex &parent = QModelIndex());
      void createObjectItem(Object *object, const QModelIndex &parent = QModelIndex());
      void createLinkItem(Link *link, const QModelIndex &parent = QModelIndex());
      void createConstraintItem(Constraint *constraint, const QModelIndex &parent = QModelIndex());
      void createObserverItem(Observer *observer, const QModelIndex &parent = QModelIndex());
      void createSolverItem(Solver *solver, const QModelIndex &parent = QModelIndex());
      void updateProjectItem(Project *project);
      void updateElementItem(EmbedItemData *element);
      void updateGroupItem(Group *group);
      void updateObjectItem(Object *object);
  };

  class ParameterTreeModel : public TreeModel {
    public:
      ParameterTreeModel(QObject *parent = nullptr);

      void createParameterItem(Parameters *parameters, const QModelIndex &parent = QModelIndex());
      void updateParameterItem(Parameters *parameters);
      void createParameterItem(Parameter *parameter, const QModelIndex &parent = QModelIndex());
  };

  class FileTreeModel : public TreeModel {
    public:
      FileTreeModel(QObject *parent = nullptr);

      void createFileItem(FileItemData *fileItemData, const QModelIndex &parent = QModelIndex());
  };

}

#endif
