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

#ifndef _PARAMETERS_CONTEXT_MENU_H_
#define _PARAMETERS_CONTEXT_MENU_H_

#include <QMenu>

namespace MBSimGUI {

  class EmbedItemData;

  class ParametersContextMenu : public QMenu {
    public:
      ParametersContextMenu(EmbedItemData *item_, const QString &title="", QWidget *parent=nullptr);
    private:
      EmbedItemData *item;
  };

}

#endif
