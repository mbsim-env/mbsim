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
#include "element_context_menu.h"
#include "element.h"

namespace MBSimGUI {

  class Element;

  class BasicItemData : public TreeItemData {
    protected:
      std::string name, value, type;
    public:
      BasicItemData(const std::string &name_, const std::string &value_, const std::string &type_="") : name(name_), value(value_), type(type_) {}
      const std::string& getName() const {return name;}
      std::string getValue() const {return value;}
      std::string getType() const {return type;}
      void setName(const std::string &name_) {name = name_;}
      void setValue(const std::string &value_) {value = value_;}
      void setType(const std::string &value_) {type = value_;}
      virtual QMenu* createContextMenu() {return new QMenu;}
  };

  class FrameItemData : public BasicItemData {
    private:
      Element *element;
    public:
      FrameItemData(Element *element_) : BasicItemData("frames",""), element(element_) {}
      virtual QMenu* createContextMenu() {return element->createFrameContextMenu();}
  };

  class ContourItemData : public BasicItemData {
    private:
      Element *element;
    public:
      ContourItemData(Element *element_) : BasicItemData("contours",""), element(element_) {}
      virtual QMenu* createContextMenu() {return new ContourContextContextMenu(element);}
  };

  class GroupItemData : public BasicItemData {
    private:
      Element *element;
    public:
      GroupItemData(Element *element_) : BasicItemData("groups",""), element(element_) {}
      virtual QMenu* createContextMenu() {return new GroupContextContextMenu(element);}
  };


  class ObjectItemData : public BasicItemData {
    private:
      Element *element;
    public:
      ObjectItemData(Element *element_) : BasicItemData("objects",""), element(element_) {}
      virtual QMenu* createContextMenu() {return new ObjectContextContextMenu(element);}
  };

  class LinkItemData : public BasicItemData {
    private:
      Element *element;
    public:
      LinkItemData(Element *element_) : BasicItemData("links",""), element(element_) {}
      virtual QMenu* createContextMenu() {return new LinkContextContextMenu(element);}
  };

  class ConstraintItemData : public BasicItemData {
    private:
      Element *element;
    public:
      ConstraintItemData(Element *element_) : BasicItemData("constraints",""), element(element_) {}
      virtual QMenu* createContextMenu() {return new ConstraintContextContextMenu(element);}
  };

  class ObserverItemData : public BasicItemData {
    private:
      Element *element;
    public:
      ObserverItemData(Element *element_) : BasicItemData("observers",""), element(element_) {}
      virtual QMenu* createContextMenu() {return new ObserverContextContextMenu(element);}
  };

  class TreeItem {
    public:

      TreeItem(TreeItemData *itemData = 0, TreeItem *parent = 0, int ID_ = 1,
          const QBrush &brush=QApplication::palette().brush(QPalette::Active, QPalette::Text)) :
          itemData(itemData), parentItem(parent), ID(ID_), foreground(brush) {
        getData_[0] = &TreeItem::getData0;
        getData_[1] = &TreeItem::getData1;
        getData_[2] = &TreeItem::getData2;
        setData_[0] = &TreeItem::setData0;
        setData_[1] = &TreeItem::setData1;
        setData_[2] = &TreeItem::setData2;
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
      QVariant getData0() const {return QString::fromStdString(itemData->getName());}
      QVariant getData1() const {return QString::fromStdString(itemData->getValue());}
      QVariant getData2() const {return QString::fromStdString(itemData->getType());}
      void setData0(const QVariant &value) {itemData->setName(value.toString().toStdString());}
      void setData1(const QVariant &value) {itemData->setValue(value.toString().toStdString());}
      void setData2(const QVariant &value) {itemData->setType(value.toString().toStdString());}
      QVariant (TreeItem::*getData_[3])() const;
      void (TreeItem::*setData_[3])(const QVariant &value);
      QVariant getData(int column) const {return (this->*getData_[column])();}
      void setData(int column, const QVariant &value) {(this->*setData_[column])(value);}
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
