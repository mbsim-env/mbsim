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

#ifndef _TREEITEMDATA__H_
#define _TREEITEMDATA__H_

#include <QString>
#include <QModelIndex>
#include <QIcon>
#include <vector>
#include <mbxmlutilshelper/dom.h>
#include "objectfactory.h"

class QMenu;

namespace MBXMLUtils {
  class FQN;
}

namespace MBSimGUI {

  class TreeItemData : public ObjectFactoryBase {
    MBSIMGUI_OBJECTFACTORY_CLASS(TreeItemData, ObjectFactoryBase, MBXMLUtils::FQN("dummy"), "Type");

    public:
      virtual ~TreeItemData() { for(auto & i : treeItemData) delete i; }
      virtual QString getName() const { return "Name"; }
      virtual QString getValue() const { return "Value"; }
      virtual QString getReference() const { return "Reference"; }
      virtual QString getComment() const { return "Comment"; }
      QIcon getDecoration() const { return icon; }
      virtual bool getEnabled() const { return true; }
      virtual bool hasReference() const { return false; }
      virtual QMenu* createContextMenu() { return nullptr; }
      void addTreeItemData(TreeItemData *treeItemData_) { treeItemData.push_back(treeItemData_); }
      void setModelIndex(const QModelIndex &index_) { index = index_; }
      const QModelIndex& getModelIndex() const { return index; }
    protected:
      std::vector<TreeItemData*> treeItemData;
      QModelIndex index;
      QIcon icon;
      QIcon orgIcon;
  };

}

#endif
