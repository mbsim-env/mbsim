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
#include "treeitem.h"
#include "parameter.h"

namespace MBSimGUI {

  TreeItem::TreeItem(TreeItemData *itemData, TreeItem *parent) : itemData(itemData), parentItem(parent) {
    foreground[0] = QApplication::palette().brush(QPalette::Disabled, QPalette::Text);
    foreground[1] = QApplication::palette().brush(QPalette::Active, QPalette::Text);
    background[0] = QApplication::palette().brush(QPalette::Active, QPalette::Base);
    background[1] = QApplication::palette().brush(QPalette::Active, QPalette::AlternateBase);
    getData_[0] = &TreeItem::getData0;
    getData_[1] = &TreeItem::getData1;
    getData_[2] = &TreeItem::getData2;
    getData_[3] = &TreeItem::getData3;
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

}
