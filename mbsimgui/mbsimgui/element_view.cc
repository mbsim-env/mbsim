/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "element_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  void ElementView::save(const QModelIndex &index, Node &node) {
    auto *model = static_cast<ElementTreeModel*>(this->model());
    TreeItem *item = model->getItem(index);
    node.addChild(Node(item->getItemData()->getName(),isExpanded(index),selectionModel()->isSelected(index)));
    for(int i=0; i<item->childCount(); i++)
      save(model->index(i,0,index),node.getChild(node.getNumberOfChilds()-1));
  }

  void ElementView::restore(const QModelIndex &index, Node &node) {
    auto *model = static_cast<ElementTreeModel*>(this->model());
    TreeItem *item = model->getItem(index);
    int i;
    for(i=0; i<node.getNumberOfChilds(); i++) {
      if(item->getItemData()->getName() == node.getChild(i).getName()) {
	setExpanded(index,node.getChild(i).isExpanded());
	if(node.getChild(i).isSelected())
	  selectionModel()->setCurrentIndex(index, QItemSelectionModel::ClearAndSelect);
	break;
      }
    }
    if(isExpanded(index)) {
      for(int j=0; j<item->childCount(); j++)
	restore(model->index(j,0,index),node.getChild(i));
    }
  }

  void ElementView::mouseDoubleClickEvent(QMouseEvent *event) {
    mw->openElementEditor();
  }

  void ElementView::mousePressEvent(QMouseEvent *event) {
    if(not mw->editorIsOpen())
      QTreeView::mousePressEvent(event);
  }

}
