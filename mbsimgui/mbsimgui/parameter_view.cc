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
#include "parameter_view.h"
#include "mainwindow.h"
#include "single_line_delegate.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterView::ParameterView(QWidget *parent) : QTreeView(parent) {
    valueDelegate=new SingleLineDelegate(this);
    setItemDelegateForColumn(1, valueDelegate);
    commentDelegate=new SingleLineDelegate(this);
    setItemDelegateForColumn(2, commentDelegate);
  }

  ParameterView::~ParameterView() {
    delete valueDelegate;
    delete commentDelegate;
  }

  void ParameterView::mouseDoubleClickEvent(QMouseEvent *event) {
    mw->openParameterEditor();
  }

  void ParameterView::mousePressEvent ( QMouseEvent * event ) {
    if(not mw->editorIsOpen())
      QTreeView::mousePressEvent(event);
  }

  void ParameterView::expandToDepth(const QModelIndex &index, int depth) {
    setExpanded(index,depth>-1);
    for(int i=0; i<model()->rowCount(index); i++) {
      const QModelIndex child = model()->index(i, 0, index);
      if(depth>0) {
	if(model()->rowCount(child)>0) setExpanded(child,true);
      }
      else {
	if(model()->rowCount(child)>0) setExpanded(child,false);
      }
      expandToDepth(child, depth-1);
    }
  }

}
