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

#ifndef _TREEITEM_H
#define _TREEITEM_H

#include "treeitemdata.h"
#include <QList>
#include <QMenu>
#include <QVariant>
#include <QVector>
#include <QApplication>

namespace MBSimGUI {

  class TreeItem {
    public:

      TreeItem(TreeItemData *itemData = nullptr, TreeItem *parent = nullptr, int ID_ = 1,
          const QBrush &brush=QApplication::palette().brush(QPalette::Active, QPalette::Text)) :
          itemData(itemData), parentItem(parent), ID(ID_), foreground(brush) {
        getData_[0] = &TreeItem::getData0;
        getData_[1] = &TreeItem::getData1;
        getData_[2] = &TreeItem::getData2;
      }
      ~TreeItem();

      TreeItem *child(int number) {return childItems.value(number);}
      int childCount() const {return childItems.count();}

      TreeItem *parent() {return parentItem;}
      bool insertChildren(TreeItem *item, int count);
      bool insertChildren(TreeItem *item, int index, int count);
      bool removeChildren(int position, int count);
      int childNumber() const;
      void setItemData(TreeItemData *data_) {
        itemData = data_;
      }
      TreeItemData* getItemData() const {return itemData;}
      QVariant getData0() const {return itemData->getName();}
      QVariant getData1() const {return itemData->getValue();}
      QVariant getData2() const {return itemData->getType();}
      QVariant (TreeItem::*getData_[3])() const;
      QVariant getData(int column) const {return (this->*getData_[column])();}
      int getID() const {return ID;}
      QBrush getForeground() { return foreground; }
      void setForeground(const QBrush &brush) { foreground=brush; }
      bool getEnabled() {
        return foreground==QApplication::palette().brush(QPalette::Active, QPalette::Text);
      }

    protected:
      QList<TreeItem*> childItems;
      TreeItemData *itemData;
      TreeItem *parentItem;
      int ID;
      QBrush foreground;
  };

}

#endif
