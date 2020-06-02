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
#include <QVariant>
#include <QPalette>
#include <QFont>
#include <QIcon>
#include <QApplication>

namespace MBSimGUI {

  class TreeItem {
    public:

      TreeItem(TreeItemData *itemData=nullptr, TreeItem *parent=nullptr, const QFont &font=QApplication::font(), const QIcon &decoration=QIcon(), const QBrush &foregroundA=QApplication::palette().brush(QPalette::Active, QPalette::Text), const QBrush &foregroundD=QApplication::palette().brush(QPalette::Disabled, QPalette::Text), const QBrush &backGround=QApplication::palette().brush(QPalette::Active, QPalette::Base));
      ~TreeItem();

      TreeItem *child(int number) { return childItems.value(number); }
      int childCount() const { return childItems.count(); }

      TreeItem *parent() { return parentItem; }
      bool insertChildren(TreeItem *item, int count);
      bool insertChildren(TreeItem *item, int index, int count);
      bool removeChildren(int position, int count);
      int childNumber() const;
      void setItemData(TreeItemData *data_) { itemData = data_; }
      TreeItemData* getItemData() const { return itemData; }
      QVariant getData0() const { return itemData->getName(); }
      QVariant getData1() const { return itemData->getValue(); }
      QVariant getData2() const { return itemData->getType(); }
      QVariant (TreeItem::*getData_[3])() const;
      QVariant getData(int column) const { return (this->*getData_[column])(); }
      QFont getFont() { return font; }
      QIcon getDecoration() { return decoration; }
      QBrush getForeground() { return foreground[itemData->isEnabled()]; }
      QBrush getBackground() { return background; }

    protected:
      QList<TreeItem*> childItems;
      TreeItemData *itemData;
      TreeItem *parentItem;
      QFont font;
      QIcon decoration;
      QBrush foreground[2], background;
  };

}

#endif
