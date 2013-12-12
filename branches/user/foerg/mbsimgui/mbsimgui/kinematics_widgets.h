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

#ifndef _KINEMATICS_WIDGETS_H_
#define _KINEMATICS_WIDGETS_H_

#include "property_context_menu.h"

class Translation;
class StateDependentTranslation;

class TranslationChoiceContextMenu : public PropertyContextMenu {

  Q_OBJECT
  public:
    TranslationChoiceContextMenu(Translation *property, QWidget * parent = 0, bool removable=false);
  protected:
    std::map<QAction*,int> actions;
    std::vector<std::string> name;
  protected slots:
    void setTranslation(QAction*);
    void setFunction(QAction*);
};


#endif

