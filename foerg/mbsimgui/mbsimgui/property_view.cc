/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "property_view.h"
#include "property.h"
#include "treemodel.h"
#include "treeitem.h"
#include "property_property_dialog.h"
#include "basic_widgets.h"
#include <iostream>
#include "mainwindow.h"

extern MainWindow *mw;

void PropertyView::openEditor () {
  if(!editor) {
    index = selectionModel()->currentIndex();

    property = dynamic_cast<Property*>(static_cast<PropertyTreeModel*>(model())->getItem(index)->getItemData());
    if(property) {
      property->setDisabled(false);
      editor = new PropertyPropertyDialog(property,property->createWidget());//,property->getUnits().getNumberOfUnits()?(new UnitWidget(property->getUnits(),property->getCurrentUnit())):0);
      //if(index.column()==1) {
      //  editor = new PropertyPropertyDialog(property);
      //}
      //else if(index.column()==2)
      //  editor = new PropertyPropertyDialog(property,new UnitWidget(property->getUnits(),property->getCurrentUnit()));
      //else if(index.column()==4 and property->disabling())
      //  editor = new PropertyPropertyDialog(property,new BoolWidget);
      if(editor) {
        editor->toWidget();
        editor->setAttribute(Qt::WA_DeleteOnClose);
        editor->show();
        connect(editor,SIGNAL(apply()),this,SLOT(apply()));
        connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
      }
    }
  }
}

void PropertyView::mouseDoubleClickEvent ( QMouseEvent * event ) {
//  index = selectionModel()->currentIndex();
//  if(index.column()==1)
    openEditor();
//  else if(index.column()==2) {
//    unit = new QSpinBox;
//    unit->show();
//  }
}

void PropertyView::mousePressEvent ( QMouseEvent * event ) {
  if(!editor)
    QTreeView::mousePressEvent(event);
//    std::cout << "mouse pressed" << std::endl;
}

void PropertyView::dialogFinished(int result) {
  if(result != 0) {
    mw->mbsimxml(1);
  }
  editor = 0;
  property = 0;
}

void PropertyView::apply() {
  mw->mbsimxml(1);
}
