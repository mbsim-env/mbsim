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
#include "parameter_property_dialog.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "parameter.h"

using namespace std;

ParameterPropertyDialog::ParameterPropertyDialog(QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f) {
  addTab("General");
  name = new TextWidget;
  ExtWidget *name_=new ExtWidget("Name",name);
  addToTab("General",name_);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),QStringList(),0));
  //input.push_back(new PhysicalStringWidget(new VecSizeVarWidget(3,1,1000),QStringList(),0));
  input.push_back(new PhysicalStringWidget(new MatRowsColsVarWidget(3,3,1,1000,1,1000),QStringList(),0));
  value = new ExtWidget("Value",new ExtPhysicalVarWidget(input,1));
  addToTab("General", value);
}

void ParameterPropertyDialog::toWidget(Parameter *parameter) {
 name->setText(QString::fromStdString(parameter->getName()));
 parameter->value.toWidget(value);
}

void ParameterPropertyDialog::fromWidget(Parameter *parameter) {
  parameter->setName(name->getText().toStdString());
  parameter->value.fromWidget(value);
}
