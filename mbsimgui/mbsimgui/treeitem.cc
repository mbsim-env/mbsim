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
#include "treeitem.h"
#include "parameter.h"
#include "qtreeview.h"

namespace MBSimGUI {

  TreeItem::TreeItem(TreeItemData *itemData, TreeItem *parent) : itemData(itemData), parentItem(parent) {
    background[0] = QApplication::palette().brush(QPalette::Active, QPalette::Base);
    background[1] = QApplication::palette().brush(QPalette::Active, QPalette::AlternateBase);
    getData_[0] = &TreeItem::getData0;
    getData_[1] = &TreeItem::getData1;
    getData_[2] = &TreeItem::getData2;
    getData_[3] = &TreeItem::getData3;
    getData_[4] = &TreeItem::getData4;
    font[0] = QApplication::font();
    font[1] = QApplication::font();
//    font[1].setItalic(true);
  }

  TreeItem::~TreeItem() {
    qDeleteAll(childItems);
  }

  int TreeItem::childNumber() const {
    if (parentItem)
      return parentItem->childItems.indexOf(const_cast<TreeItem*>(this));

    return 0;
  }

  bool TreeItem::insertChildren(TreeItem *item, int count) {

    for (int row = 0; row < count; ++row)
      childItems.insert(childItems.count(), item);

    return true;
  }

  bool TreeItem::insertChildren(TreeItem *item, int index, int count) {

    for (int row = 0; row < count; ++row)
      childItems.insert(index, item);

    return true;
  }

  bool TreeItem::removeChildren(int position, int count) {
    if (position < 0 || position + count > childItems.size())
      return false;

    for (int row = 0; row < count; ++row)
      delete childItems.takeAt(position);

    return true;
  }

  int TreeItem::childCount(QTreeView *model) const {
    if(model==nullptr)
      return childItems.count();

    int count=0;
    for(auto &ci : childItems) {
      auto index = ci->getItemData()->getModelIndex();
      if(model->isRowHidden(index.row(), index.parent()))
        continue;
      count++;
    }
    return count;
  }

}
