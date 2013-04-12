#include <QStringList>
#include <iostream>

#include "treeitem.h"

using namespace std;

TreeItem::~TreeItem() {
  delete itemData;
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

bool TreeItem::removeChildren(int position, int count) {
  if (position < 0 || position + count > childItems.size())
    return false;

  for (int row = 0; row < count; ++row)
    delete childItems.takeAt(position);

  return true;
}
