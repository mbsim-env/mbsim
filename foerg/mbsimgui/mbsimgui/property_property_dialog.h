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

#ifndef _PROPERTY_PROPERTY_DIALOG_H_
#define _PROPERTY_PROPERTY_DIALOG_H_

#include "property_dialog.h"

class Property;
class Widget;

class PropertyPropertyDialog : public PropertyDialog {

  public:
    PropertyPropertyDialog(Property *property, Widget *widget1=0, Widget *widget2=0, Widget *widget3=0, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Property *property);
    virtual void fromWidget(Property *property);
    void toWidget() {toWidget(property);}
    void fromWidget() {fromWidget(property);}
    Property* getProperty() {return property;}
  protected:
    Property *property;
    Widget *widget1, *widget2, *widget3;
};

#endif
