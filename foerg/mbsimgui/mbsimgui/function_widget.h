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

#ifndef _FUNCTION_WIDGET_H_
#define _FUNCTION_WIDGET_H_

#include "widget.h"
#include "property_context_menu.h"

class QComboBox;

class FunctionWidget : public Widget {
  Q_OBJECT
  public:
    virtual int getArg1Size() const {return 0;}
    virtual int getArg2Size() const {return 0;}
    virtual void setArg1Size(int i) {}
  public slots:
    virtual void resize_(int m, int n) {}
  signals:
    void arg1SizeChanged(int);
};

class FunctionChoiceWidget : public Widget {
  public:
    FunctionChoiceWidget();
    void fromProperty(Property *property);
    void toProperty(Property *property);
  protected:
    QComboBox *comboBox;
};

class FunctionChoiceContextMenu : public PropertyContextMenu {

  Q_OBJECT
  public:
    FunctionChoiceContextMenu(Property *property, QWidget * parent = 0, bool removable=false);
  protected:
    std::map<QAction*,int> actions;
  protected slots:
    void setFunction(QAction*);
};


#endif
