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

#ifndef _TREEMODEL_H
#define _TREEMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

namespace MBSimGUI {

  class EmbedItemData;
  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;
  class Parameter;
  class TreeItem;
  class TreeItemData;

  class TreeModel : public QAbstractItemModel {
    public:
      TreeModel(QObject *parent = nullptr);
      ~TreeModel() override;

      QVariant data(const QModelIndex &index, int role) const override;
      QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

      QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
      QModelIndex parent(const QModelIndex &index) const override;

      int rowCount(const QModelIndex &parent = QModelIndex()) const override;
      int columnCount(const QModelIndex &) const override {return 3;}

      Qt::ItemFlags flags(const QModelIndex &index) const override;
      bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
      bool setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role = Qt::EditRole) override;

      bool removeRows(int position, int rows, const QModelIndex &parent = QModelIndex()) override;

      TreeItem* getItem(const QModelIndex &index) const;
      QModelIndex findItem(const TreeItemData *item, const QModelIndex &parentIndex) const;

    protected:
      TreeItem *rootItem{0};
  };

  class ElementTreeModel : public TreeModel {
    public:
      ElementTreeModel(QObject *parent = nullptr);

      void createFrameItem(Frame *frame, const QModelIndex &parent = QModelIndex());
      void createContourItem(Contour *contour, const QModelIndex &parent = QModelIndex());
      void createGroupItem(Group *group, const QModelIndex &parent = QModelIndex(), bool recursive=true);
      void createObjectItem(Object *object, const QModelIndex &parent = QModelIndex(), bool recursive=true);
      void createLinkItem(Link *link, const QModelIndex &parent = QModelIndex());
      void createConstraintItem(Constraint *constraint, const QModelIndex &parent = QModelIndex());
      void createObserverItem(Observer *observer, const QModelIndex &parent = QModelIndex());

      std::map<QString, QModelIndex> idEleMap;
  };

  class EmbeddingTreeModel : public TreeModel {
    public:
      EmbeddingTreeModel(QObject *parent = nullptr);

      QModelIndex createEmbeddingItem(EmbedItemData *itemData, const QModelIndex &parent = QModelIndex());
      QModelIndex createParameterItem(Parameter *parameter, const QModelIndex &parent = QModelIndex());

      std::map<Element*, QModelIndex> idEleMap;
  };

}

#endif
