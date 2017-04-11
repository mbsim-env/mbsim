/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

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

#ifndef _TREEITEMDATA__H_
#define _TREEITEMDATA__H_

#include <QString>

class QMenu;

namespace MBSimGUI {

  class TreeItemData {
    protected:
      QString name, value;

    public:
      TreeItemData(const QString &name_="", const QString &value_="") : name(name_), value(value_) { }
      virtual ~TreeItemData() { }
      const QString& getName() const { return name; }
      void setName(const QString &str) { name = str; }
      const QString& getValue() const { return value; }
      void setValue(const QString &str) { value = str; }
      virtual QString getType() const { return ""; }
      void setType(const QString &str) { }
      virtual QMenu* createContextMenu() { return NULL; }
  };

}

#endif
