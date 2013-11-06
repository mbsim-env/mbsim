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
#include "property_property_dialog.h"
#include "property.h"
#include "widget.h"

using namespace std;

PropertyPropertyDialog::PropertyPropertyDialog(Property *property_, Widget *widget1_, Widget *widget2_, Widget *widget3_, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), property(property_), widget1(widget1_), widget2(widget2_), widget3(widget3_) {
  addTab("General");
  if(widget1)
    addToTab("General",widget1);
  if(widget2)
    addToTab("General",widget2);
  if(widget3)
    addToTab("General",widget3);
}

void PropertyPropertyDialog::toWidget(Property *property) {
  if(widget1)
    widget1->fromProperty(property);
  if(widget2)
    widget2->fromProperty(property);
  if(widget3)
    widget3->fromProperty(property);
}

void PropertyPropertyDialog::fromWidget(Property *property) {
  if(widget1)
    widget1->toProperty(property);
  if(widget2)
    widget2->toProperty(property);
  if(widget3)
    widget3->toProperty(property);
}


