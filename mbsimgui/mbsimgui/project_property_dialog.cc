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

#include <config.h>
#include "project_property_dialog.h"
#include "mainwindow.h"
#include "basic_widgets.h"
#include "extended_widgets.h"

using namespace std;

namespace MBSimGUI {

  ProjectPropertyDialog::ProjectPropertyDialog(MainWindow *mw_, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), mw(mw_) {
    addTab("General");
    vector<QString> list;
    list.push_back("octave");
    list.push_back("python");
    evalSelect = new ExtWidget("Evaluator",new TextChoiceWidget(list,0),false);
    addToTab("General",evalSelect);
  }

  void ProjectPropertyDialog::toWidget(MainWindow *mw) {
//    mw->evalSelect.toWidget(evalSelect);
  }

  void ProjectPropertyDialog::fromWidget(MainWindow *mw) {
//    mw->evalSelect.fromWidget(evalSelect);
  }

}
