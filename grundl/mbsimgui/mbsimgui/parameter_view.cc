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
#include "parameter_view.h"
#include "parameter.h"
#include "parameter_property_dialog.h"
#include "treemodel.h"
#include "treeitem.h"
#include "mainwindow.h"
#include <QEvent>

extern MainWindow *mw;

void ParameterView::mouseDoubleClickEvent ( QMouseEvent * event ) {
  if(!editor) {
    index = selectionModel()->currentIndex();
    if(index.isValid()) {
      Parameter *parameter = static_cast<Parameter*>(static_cast<ParameterListModel*>(model())->getItem(index)->getItemData());
      editor = parameter->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,SIGNAL(apply()),this,SLOT(apply()));
      connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
    }
  }
}

void ParameterView::mousePressEvent ( QMouseEvent * event ) {
  if(!editor)
    QTreeView::mousePressEvent(event);
}

void ParameterView::dialogFinished(int result) {
  if(result != 0) {
    mw->updateOctaveParameters();
    mw->mbsimxml(1);
  }
  editor = 0;
}

void ParameterView::apply() {
  update(index);
  update(index.sibling(index.row(),1));
  mw->updateOctaveParameters();
  mw->mbsimxml(1);
}

