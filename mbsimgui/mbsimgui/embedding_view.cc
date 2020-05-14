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
#include "embedding_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include "embeditemdata.h"
#include "parameter.h"
#include "mainwindow.h"
#include "element_view.h"
#include "solver_view.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  void ParameterView::openEditor(bool config) {
    if(not mw->editorIsOpen()) {
      mw->setAllowUndo(false);
      index = selectionModel()->currentIndex();
      parameter = dynamic_cast<Parameter*>(static_cast<EmbeddingTreeModel*>(model())->getItem(index)->getItemData());
      if(parameter) {
        Parameters *item = static_cast<Parameters*>(static_cast<EmbeddingTreeModel*>(model())->getItem(index.parent())->getItemData());
        mw->updateParameters(item->getItem(),true);
        editor = parameter->createPropertyDialog();
        editor->setAttribute(Qt::WA_DeleteOnClose);
        if(config)
          editor->toWidget();
        else
          editor->setCancel(false);
        editor->show();
        connect(editor,&ParameterPropertyDialog::apply,this,&ParameterView::apply);
        connect(editor,&ParameterPropertyDialog::finished,this,&ParameterView::dialogFinished);
        return;
      }
    }
  }

  void ParameterView::mouseDoubleClickEvent(QMouseEvent *event) {
    openEditor();
  }

  void ParameterView::mousePressEvent ( QMouseEvent * event ) {
    if(!editor)
      QTreeView::mousePressEvent(event);
  }

  void ParameterView::dialogFinished(int result) {
    if(result != 0) {
      if(editor->getCancel())
        mw->setProjectChanged(true);
      editor->fromWidget();
      if(mw->getAutoRefresh()) mw->refresh();
      if(parameter)
        parameter->getParent()->updateStatus();
    }
    parameter = nullptr;
    editor = nullptr;
    mw->setAllowUndo(true);
  }

  void ParameterView::apply() {
    if(editor->getCancel())
      mw->setProjectChanged(true);
    editor->fromWidget();
    update(index);
    update(index.sibling(index.row(),1));
    if(mw->getAutoRefresh()) mw->refresh();
    if(parameter) {
      editor->setCancel(true);
      parameter->getParent()->updateStatus();
    }
    editor->setCancel(true);
  }

}
