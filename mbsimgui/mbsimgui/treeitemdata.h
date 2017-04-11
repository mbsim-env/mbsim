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
#include <xercesc/util/XercesDefs.hpp>

class QMenu;

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
}

namespace MBSimGUI {

  class EmbeddingPropertyDialog;
  class Parameter;

  class TreeItemData {
    public:
      virtual ~TreeItemData() { }
      virtual const QString& getName() const = 0;
      virtual const QString& getValue() const = 0;
      virtual QString getType() const { return ""; }
      virtual void setName(const QString &data) { }
      virtual void setValue(const QString &data) { }
      virtual void setType(const QString &data) { }
      virtual const QString& getCounterName() const = 0;
      virtual void setCounterName(const QString &str) = 0;
      virtual QMenu* createContextMenu() = 0;
      virtual EmbeddingPropertyDialog* createEmbeddingPropertyDialog() { return NULL; }
      virtual std::vector<TreeItemData*> getParents() { return std::vector<TreeItemData*>(); }
      virtual int getNumberOfParameters() const { return 0; }
      virtual Parameter* getParameter(int i) { return NULL; }
      virtual xercesc::DOMElement* getXMLElement() { return NULL; }
  };

}

#endif
