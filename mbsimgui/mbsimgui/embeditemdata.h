/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

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

#ifndef _EMBEDITEMDATA__H_
#define _EMBEDITEMDATA__H_

#include "treeitemdata.h"
#include "property_dialog.h"
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMDocument;
}

namespace MBSimGUI {

  class Parameter;
  class Parameters;
  class FileItemData;

  class EmbedItemData : public TreeItemData {
    protected:
      QString name;
      std::vector<Parameter*> parameter;
      xercesc::DOMElement *element{nullptr}, *embed{nullptr};
      Parameters *parameters;
      FileItemData *fileItem{nullptr};
      FileItemData *parameterFileItem{nullptr};
      FileItemData *dedicatedFileItem{nullptr};
      FileItemData *dedicatedParameterFileItem{nullptr};

    public:
      EmbedItemData();
      ~EmbedItemData() override;
      QString getName() const override { return name; }
      QString getValue() const override { return ""; }
      bool isActive();
      virtual void create() { updateName(); }
      virtual void clear() { }
      void createParameters();
      void clearParameters();
      virtual EmbedItemData* getEmbedItemParent() { return nullptr; }
      std::vector<EmbedItemData*> getEmbedItemParents();
      int getNumberOfParameters() const { return parameter.size(); }
      Parameter* getParameter(int i) { return parameter[i]; }
      void setParameter(Parameter *param, int i) { parameter[i] = param; }
      void addParameter(Parameter *param);
      void removeParameter(Parameter *param);
      int getIndexOfParameter(Parameter *param) const;
      xercesc::DOMElement* getXMLElement() { return element; }
      void setXMLElement(xercesc::DOMElement *element_) { element = element_; }
      virtual void removeXMLElements();
      void removeXMLElement(bool removeEmbedding=true);
      xercesc::DOMElement* createParameterXMLElement();
      virtual xercesc::DOMElement* createEmbedXMLElement();
      xercesc::DOMElement* getEmbedXMLElement() { return embed; }
      void setEmbedXMLElement(xercesc::DOMElement *embed_) { embed = embed_; }
      virtual void maybeRemoveEmbedXMLElement();
      bool hasParameterXMLElement() const;
      void setFileItem(FileItemData *fileItem_);
      void setParameterFileItem(FileItemData *parameterFileItem_);
      virtual void setDedicatedFileItem(FileItemData *dedicatedFileItem_) { dedicatedFileItem = dedicatedFileItem_; }
      virtual void setDedicatedParameterFileItem(FileItemData *dedicatedParameterFileItem_) { dedicatedParameterFileItem = dedicatedParameterFileItem_; }
      virtual EmbedItemData *getDedicatedItem() { return this; }
      Parameters* getParameters() { return parameters; }
      FileItemData *getFileItem() { return fileItem; }
      FileItemData *getParameterFileItem() { return parameterFileItem; }
      FileItemData *getDedicatedFileItem() { return dedicatedFileItem; }
      FileItemData *getDedicatedParameterFileItem() { return dedicatedParameterFileItem; }
      virtual xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element);
      virtual void updateStatus() { }
      virtual PropertyDialog* createPropertyDialog() { return new EmbedItemPropertyDialog(this); }
      bool getSelfEmbeded() const { return embed and MBXMLUtils::E(embed)->hasAttribute("href"); }
      QString getReference() const override;
      bool hasReference() const override { return fileItem; }
      bool hasParameterReference() const { return parameterFileItem; }
      void updateName();
      virtual void updateNames() { updateName(); }
      virtual void updateValues();
  };

}

#endif
