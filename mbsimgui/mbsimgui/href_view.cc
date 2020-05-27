/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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
#include "href_view.h"
#include "element.h"
#include "element_property_dialog.h"
#include "treemodel.h"
#include "treeitem.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  void HrefView::openEditor() {
    if(not mw->editorIsOpen()) {
      mw->setAllowUndo(false);
      index = selectionModel()->currentIndex();
      element = dynamic_cast<Element*>(static_cast<ElementTreeModel*>(model())->getItem(index)->getItemData());
      if(element) {
        mw->updateParameters(element);
        editor = element->createPropertyDialog();
        editor->setAttribute(Qt::WA_DeleteOnClose);
        if(config)
          editor->toWidget();
        else
          editor->setCancel(false);
        editor->show();
        connect(editor,&ElementPropertyDialog::apply,this,&ElementView::apply);
        connect(editor,&ElementPropertyDialog::finished,this,&ElementView::dialogFinished);
      }
    }
  }

  void HrefView::mouseDoubleClickEvent(QMouseEvent *event) {
    openEditor();
  }

  void HrefView::mousePressEvent(QMouseEvent *event) {
    if(!editor)
      QTreeView::mousePressEvent(event);
  }

  void HrefView::dialogFinished(int result) {
//    if(result != 0) {
//      if(editor->getCancel())
//        mw->setProjectChanged(true);
//      editor->fromWidget();
//      if(mw->getAutoRefresh()) mw->refresh();
//    }
//    editor = nullptr;
//    element = nullptr;
//    mw->setAllowUndo(true);
  }

  void HrefView::apply() {
//    if(editor->getCancel())
//      mw->setProjectChanged(true);
//    editor->fromWidget();
//    update(index);
//    if(mw->getAutoRefresh()) mw->refresh();
//    editor->setCancel(true);
//    editor->setCancel(true);
  }

}
