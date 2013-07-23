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
#include "element_view.h"
#include "element.h"
#include "element_property_dialog.h"
#include "treemodel.h"
#include "treeitem.h"
#include "mainwindow.h"
#include <QEvent>

extern MainWindow *mw;

void ElementView::openEditor () {
  if(!editor) {
    index = selectionModel()->currentIndex();
    element = dynamic_cast<Element*>(static_cast<ElementTreeModel*>(model())->getItem(index)->getItemData());
    if(element) {
      if(element->isEmbedded())
        mw->updateOctaveParameters(element->getParameterList());
      editor = element->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,SIGNAL(apply()),this,SLOT(apply()));
      connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
    }
  }
}

void ElementView::mouseDoubleClickEvent ( QMouseEvent * event ) {
  openEditor();
}

void ElementView::mousePressEvent ( QMouseEvent * event ) {
  if(!editor)
    QTreeView::mousePressEvent(event);
}

void ElementView::dialogFinished(int result) {
  if(result != 0) {
    if(element->isEmbedded())
      mw->updateOctaveParameters(ParameterList());
    mw->mbsimxml(1);
  }
  editor = 0;
  element = 0;
}

void ElementView::apply() {
  update(index);
  if(element->isEmbedded())
    mw->updateOctaveParameters(element->getParameterList());
  mw->mbsimxml(1);
}
