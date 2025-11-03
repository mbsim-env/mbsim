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

#ifndef _TREEITEM_H
#define _TREEITEM_H

#include "treeitemdata.h"
#include <QList>
#include <QVariant>
#include <QPalette>
#include <QFont>
#include <QIcon>
#include <QApplication>

class QTreeView;

namespace MBSimGUI {

  class TreeItem {
    public:

      TreeItem(TreeItemData *itemData=nullptr, TreeItem *parent=nullptr);
      ~TreeItem();

      TreeItem *child(int number) { if(number<0 || number>=childItems.size()) return nullptr; return childItems.value(number); }

      // if model==nullptr hidden and none-hidden childs are counted
      // if model is given only none-hidden childs of the specified model are counted
      int childCount(QTreeView *model=nullptr) const;

      TreeItem *parent() { return parentItem; }
      bool insertChildren(TreeItem *item, int count);
      bool insertChildren(TreeItem *item, int index, int count);
      bool removeChildren(int position, int count);
      int childNumber() const;
      TreeItemData* getItemData() const { return itemData; }
      QVariant getData0() const { return itemData->getName(); }
      QVariant getData1() const { return itemData->getValue(); }
      QVariant getData2() const { return itemData->getComment(); }
      QVariant getData3() const { return itemData->getType(); }
      QVariant getData4() const { return itemData->getReference(); }
      QVariant (TreeItem::*getData_[5])() const;
      QVariant getData(int column) const { return (this->*getData_[column])(); }
      QFont getFont() { return font[0]; }
      QIcon getDecoration() { return itemData->getDecoration(); }
      QBrush getForeground() { return foreground; }
      void setForeground(const QBrush &brush) { foreground=brush; }
      QBrush getBackground() { return background[itemData->hasReference()]; }

    protected:
      QList<TreeItem*> childItems;
      TreeItemData *itemData { nullptr };
      TreeItem *parentItem { nullptr };
      QFont font[2];
      QIcon decoration;
      QBrush foreground { QApplication::palette().brush(QPalette::Active, QPalette::Text) }; // foreground is adapted by AbstractViewFilter
      QBrush background[2];
  };

}

#endif
