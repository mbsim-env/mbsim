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

#ifndef _EMBEDITEMDATA__H_
#define _EMBEDITEMDATA__H_

#include "treeitemdata.h"
#include "embedding_property_dialog.h"
#include "embedding_context_menu.h"
#include <xercesc/util/XercesDefs.hpp>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
}

namespace MBSimGUI {

  class Parameter;

  class EmbedItemData : public TreeItemData {
    protected:
      QString counterName;
      std::vector<Parameter*> parameter;
      std::vector<Parameter*> removedParameter;
      xercesc::DOMElement *element;

    public:
      EmbedItemData(const QString &name="") : TreeItemData(name), element(NULL) { }
      ~EmbedItemData();
      const QString& getCounterName() const { return counterName; }
      void setCounterName(const QString &str) { counterName = str; }
      virtual std::vector<EmbedItemData*> getParents() { return std::vector<EmbedItemData*>(); }
      int getNumberOfParameters() const { return parameter.size(); }
      Parameter* getParameter(int i) { return parameter[i]; }
      void addParameter(Parameter *param);
      void removeParameter(Parameter *param);
      xercesc::DOMElement* getXMLElement() { return element; }
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() { return new EmbeddingPropertyDialog(this); }
      QMenu* createEmbeddingContextMenu() { return new EmbeddingContextMenu(this); }
  };

}

#endif
