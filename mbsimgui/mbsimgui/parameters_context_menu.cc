/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "parameters_context_menu.h"
#include "mainwindow.h"
#include "parameter.h"
#include "embeditemdata.h"
#include "frame.h"
#include "utils.h"
#include "view_menu.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ParametersContextMenu::ParametersContextMenu(EmbedItemData *item_, const QString &title, QWidget *parent) : QMenu(title,parent), item(item_) {
    setToolTipsVisible(true);
    if(not dynamic_cast<InternalFrame*>(item_)) {
      QAction *action=new QAction(QIcon::fromTheme("document-properties"), "Edit XML", this);
      action->setEnabled(item->getNumberOfParameters());
      connect(action,&QAction::triggered,mw,&MainWindow::editParametersSource);
      addAction(action);
      addSeparator();
      action = new QAction(QIcon::fromTheme("document-save-as"), "Export parameters to file...", this);
      action->setEnabled(item->getNumberOfParameters());
      connect(action,&QAction::triggered,this,[=](){ mw->exportParameters(); });
      addAction(action);
      addSeparator();
      action = new QAction(QIcon::fromTheme("document-open"), "Import/Reference parameters from file...", this);
      action->setToolTip("Import parameters from a file or use the XML 'Embed' functionality to reference a external parameter file.");
      connect(action,&QAction::triggered,this,[=](){ mw->loadParameter(item); });
      addAction(action);
      action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
      action->setShortcut(QKeySequence::Paste);
      action->setEnabled(mw->getParameterBuffer().first);
      connect(action,&QAction::triggered,this,[=](){ mw->pasteParameter(item, mw->getParameterBuffer().first); });
      addAction(action);
      addSeparator();
      action = new QAction(QIcon::fromTheme("edit-delete"), "Remove all", this);
      action->setEnabled(item->getNumberOfParameters());
      connect(action,&QAction::triggered,this,[=](){ mw->removeParameter(item); });
      addAction(action);
      addSeparator();
      createContextMenuFor<Parameter>(this, item, "Add '");
    }
    addSeparator();
    addMenu(new ViewMenu(this));
  }

}
