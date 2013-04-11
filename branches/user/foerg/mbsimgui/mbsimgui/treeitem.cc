#include <QStringList>
#include <iostream>

#include "treeitem.h"

using namespace std;

BasicItemData::~BasicItemData() {
  cout << "destroy BasicItemData" << endl;
}

TreeItem::~TreeItem() {
  cout << "destroy TreeItem data" << endl; 
  delete itemData;
  cout << "destroy TreeItem" << endl; 
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

//Container::Container(const QString &name, TreeItem *parent) : TreeItem(parent) {
//  itemData[0] = name;
//}
//
//Group::Group(const QString &name, TreeItem *parent) : TreeItem(parent) {
//  itemData[0] = name;
//  itemData[1] = "Group";
//}
//
//Object::Object(const QString &name, TreeItem *parent) : TreeItem(parent) {
//  itemData[0] = name;
//  itemData[1] = "Object";
//}
//
//Link::Link(const QString &name, TreeItem *parent) : TreeItem(parent) {
//  itemData[0] = name;
//  itemData[1] = "Link";
//}
//
