#ifndef TREEITEM_H
#define TREEITEM_H

#include "treeitemdata.h"
#include <QList>
#include <QVariant>
#include <QVector>

class BasicItemData : public TreeItemData {
  private:
    std::string name, type;
  public:
    BasicItemData(const std::string &name_, const std::string &type_) : name(name_), type(type_) {}
    ~BasicItemData(); 
    const std::string& getName() const {return name;}
    std::string getType() const {return type;}
    void setName(const std::string &name_) {name = name_;}
    void setType(const std::string &type_) {type = type_;}
    bool isRemovable() {return false;}
};

class TreeItem {
  public:

    TreeItem(TreeItemData *itemData = 0, TreeItem *parent = 0) : itemData(itemData), parentItem(parent) {
      getData_[0] = &TreeItem::getData0;
      getData_[1] = &TreeItem::getData1;
      setData_[0] = &TreeItem::setData0;
      setData_[1] = &TreeItem::setData1;
    }
    ~TreeItem();

    TreeItem *child(int number) {return childItems.value(number);}
    int childCount() const {return childItems.count();}

    TreeItem *parent() {return parentItem;}
    bool insertChildren(TreeItem *item, int count);
    bool removeChildren(int position, int count);
    int childNumber() const;
    void setItemData(TreeItemData *data_) {itemData = data_;}
    TreeItemData* getItemData() const {return itemData;}
    QVariant getData0() const {return QString::fromStdString(itemData->getName());}
    QVariant getData1() const {return QString::fromStdString(itemData->getType());}
    void setData0(const QVariant &value) {itemData->setName(value.toString().toStdString());}
    void setData1(const QVariant &value) {itemData->setType(value.toString().toStdString());}
    QVariant (TreeItem::*getData_[2])() const;
    void (TreeItem::*setData_[2])(const QVariant &value);
    QVariant getData(int column) const {return (this->*getData_[column])();}
    void setData(int column, const QVariant &value) {(this->*setData_[column])(value);}
    bool isRemovable() {return itemData->isRemovable();}

  protected:
    QList<TreeItem*> childItems;
    TreeItemData *itemData;
    TreeItem *parentItem;
};

#endif
