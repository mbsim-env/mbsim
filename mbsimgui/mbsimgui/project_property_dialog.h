/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#ifndef _PROJECT_PROPERTY_DIALOG_H_
#define _PROJECT_PROPERTY_DIALOG_H_

#include "property_dialog.h"
#include "widget.h"

namespace MBSimGUI {

  class MainWindow;
  class ExtWidget;

  class ProjectPropertyDialog : public PropertyDialog {

    public:
      ProjectPropertyDialog(MainWindow *mw, QWidget * parent = 0, Qt::WindowFlags f = 0);
      virtual void toWidget(MainWindow *mw);
      virtual void fromWidget(MainWindow *mw);
      void toWidget() {toWidget(mw);}
      void fromWidget() {fromWidget(mw);}
    protected:
      MainWindow *mw;
      ExtWidget *evalSelect;
  };

}

#endif
