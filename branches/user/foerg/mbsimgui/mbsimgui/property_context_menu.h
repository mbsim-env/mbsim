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

#ifndef _PROPERTY_CONTEXT_MENU_H_
#define _PROPERTY_CONTEXT_MENU_H_

#include <QMenu>

class Property;

class PropertyContextMenu : public QMenu {

  Q_OBJECT
  public:
    PropertyContextMenu(Property *property, QWidget * parent = 0, bool removable=false);
  protected slots:
    void disableProperty(bool disable);
  protected:
    Property *property;
};

class ChoiceProperty2ContextMenu : public PropertyContextMenu {

  Q_OBJECT
  public:
    ChoiceProperty2ContextMenu(Property *property, QWidget * parent = 0, bool removable=false);
  protected slots:
    void selectProperty();
};

// class FunctionChoiceContextMenu : public PropertyContextMenu {
// 
//   Q_OBJECT
//   public:
//     FunctionChoiceContextMenu(Property *property, QWidget * parent = 0, bool removable=false);
//   protected:
//     std::map<QAction*,int> actions;
//   protected slots:
//     void setFunction(QAction*);
// };

#endif
