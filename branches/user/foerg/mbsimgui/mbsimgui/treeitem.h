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
#include "element_context_menu.h"

class Element;
class Property;

class BasicItemData : public TreeItemData {
  protected:
    std::string name, value, unit, evaluation;
  public:
    BasicItemData(const std::string &name_, const std::string &value_, const std::string &unit_, const std::string &evaluation_) : name(name_), value(value_), unit(unit_), evaluation(evaluation_) {}
    const std::string& getName() const {return name;}
    const std::string& getValue() const {return value;}
    const std::string& getUnit() const {return unit;}
    const std::string& getEvaluation() const {return evaluation;}
    void setName(const std::string &name_) {name = name_;}
    void setValue(const std::string &value_) {value = value_;}
    void setUnit(const std::string &unit_) {unit = unit_;}
    void setEvaluation(const std::string &evaluation_) {evaluation = evaluation_;}
    virtual QMenu* createContextMenu() {return new QMenu;}
};

class FrameItemData : public BasicItemData {
  private:
    Element *element;
  public:
    FrameItemData(Element *element_) : BasicItemData("frames","","",""), element(element_) {}
    virtual QMenu* createContextMenu() {return new FrameContextContextMenu(element);}
};

class ContourItemData : public BasicItemData {
  private:
    Element *element;
  public:
    ContourItemData(Element *element_) : BasicItemData("contours","","",""), element(element_) {}
    virtual QMenu* createContextMenu() {return new ContourContextContextMenu(element);}
};

class GroupItemData : public BasicItemData {
  private:
    Element *element;
  public:
    GroupItemData(Element *element_) : BasicItemData("groups","","",""), element(element_) {}
    virtual QMenu* createContextMenu() {return new GroupContextContextMenu(element);}
};

class ObjectItemData : public BasicItemData {
  private:
    Element *element;
  public:
    ObjectItemData(Element *element_) : BasicItemData("objects","","",""), element(element_) {}
    virtual QMenu* createContextMenu() {return new ObjectContextContextMenu(element);}
};

class LinkItemData : public BasicItemData {
  private:
    Element *element;
  public:
    LinkItemData(Element *element_) : BasicItemData("links","","",""), element(element_) {}
    virtual QMenu* createContextMenu() {return new LinkContextContextMenu(element);}
};

class ObserverItemData : public BasicItemData {
  private:
    Element *element;
  public:
    ObserverItemData(Element *element_) : BasicItemData("observers","","",""), element(element_) {}
    virtual QMenu* createContextMenu() {return new ObserverContextContextMenu(element);}
};

//class PropertyItemData : public BasicItemData {
//  private:
//    Element *element;
//  public:
//    PropertyItemData(Element *element_) : BasicItemData("properties",""), element(element_) {}
//    virtual QMenu* createContextMenu() {return 0;}
////    virtual QMenu* createContextMenu() {return new ObserverContextContextMenu(element);}
//};

//class TreeItem {
//  public:
//
//    TreeItem(TreeItemData *itemData = 0, TreeItem *parent = 0, int ID_ = 1) : itemData(itemData), parentItem(parent), ID(ID_) {
//      getData_[0] = &TreeItem::getData0;
//      getData_[1] = &TreeItem::getData1;
//      setData_[0] = &TreeItem::setData0;
//      setData_[1] = &TreeItem::setData1;
//    }
//    ~TreeItem();
//
//    TreeItem *child(int number) {return childItems.value(number);}
//    int childCount() const {return childItems.count();}
//
//    TreeItem *parent() {return parentItem;}
//    bool insertChildren(TreeItem *item, int count);
//    bool removeChildren(int position, int count);
//    int childNumber() const;
//    void setItemData(TreeItemData *data_) {itemData = data_;}
//    TreeItemData* getItemData() const {return itemData;}
//    QVariant getData0() const {return QString::fromStdString(itemData->getName());}
//    QVariant getData1() const {return QString::fromStdString(itemData->getValue());}
//    void setData0(const QVariant &value) {itemData->setName(value.toString().toStdString());}
//    void setData1(const QVariant &value) {itemData->setValue(value.toString().toStdString());}
//    QVariant (TreeItem::*getData_[2])() const;
//    void (TreeItem::*setData_[2])(const QVariant &value);
//    QVariant getData(int column) const {return (this->*getData_[column])();}
//    void setData(int column, const QVariant &value) {(this->*setData_[column])(value);}
//    int getID() const {return ID;}
//
//  protected:
//    QList<TreeItem*> childItems;
//    TreeItemData *itemData;
//    TreeItem *parentItem;
//    int ID;
//};

class TreeItem {
  public:

    TreeItem(TreeItemData *itemData = 0, TreeItem *parent = 0, int ID_ = 1);
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
    QVariant getData1() const {return QString::fromStdString(itemData->getValue());}
    QVariant getData2() const {return QString::fromStdString(itemData->getUnit());}
    QVariant getData3() const {return QString::fromStdString(itemData->getEvaluation());}
    bool isDisabled() const {return itemData->isDisabled();}
    void setData0(const QVariant &value) {itemData->setName(value.toString().toStdString());}
    void setData1(const QVariant &value) {itemData->setValue(value.toString().toStdString());}
    void setData2(const QVariant &value) {itemData->setUnit(value.toString().toStdString());}
    void setData3(const QVariant &value) {itemData->setEvaluation(value.toString().toStdString());}
    QVariant (TreeItem::*getData_[4])() const;
    void (TreeItem::*setData_[4])(const QVariant &value);
    QVariant getData(int column) const {return (this->*getData_[column])();}
    void setData(int column, const QVariant &value) {(this->*setData_[column])(value);}
    int getID() const {return ID;}

  protected:
    QList<TreeItem*> childItems;
    TreeItemData *itemData;
    TreeItem *parentItem;
    int ID;
};

#endif