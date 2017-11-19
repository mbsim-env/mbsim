/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "embeditemdata.h"
#include "parameter.h"
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  EmbedItemData::~EmbedItemData() {
    for (auto & it : parameter)
      delete it;
    for(auto & i : removedParameter)
      delete i;
  }

//  QString EmbedItemData::getName() const {
//      //return element?QString::fromStdString(MBXMLUtils::E(element)->getAttribute("name")):"Name";
//    return QString::fromStdString(MBXMLUtils::E(element)->getAttribute("name"));
//  }

  void EmbedItemData::addParameter(Parameter *param) {
    parameter.push_back(param);
    param->setParent(this);
  }

  void EmbedItemData::removeParameter(Parameter *param) {
    for (auto it = parameter.begin(); it != parameter.end(); it++) {
      if(*it == param) {
        parameter.erase(it);
        break;
      }
    }
    removedParameter.push_back(param);
  }

  int EmbedItemData::getIndexOfParameter(Parameter *param) const {
    for(size_t i=0; i<parameter.size(); i++)
      if(parameter[i] == param)
        return i;
    return -1;
  }

  void EmbedItemData::removeXMLElement(bool removeEmbedding) {
    DOMNode *parent = element->getParentNode();
    if(removeEmbedding and X()%parent->getNodeName()=="Embed") {
      DOMNode *e = parent->getFirstChild();
      while(e) {
        DOMNode *en=e->getNextSibling();
        parent->removeChild(e);
        e = en;
      }
      DOMNode *ps = parent->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        parent->getParentNode()->removeChild(ps);
      parent->getParentNode()->removeChild(parent);
    }
    else {
      DOMNode *ps = element->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        parent->removeChild(ps);
      parent->removeChild(element);
    }
  }

  DOMElement* EmbedItemData::createParameterXMLElement() {
    DOMElement *param = E(createEmbedXMLElement())->getFirstElementChildNamed(PV%"Parameter");
    if(not param) {
      param = D(getXMLElement()->getOwnerDocument())->createElement(PV%"Parameter");
      getEmbedXMLElement()->insertBefore(param,getEmbedXMLElement()->getFirstElementChild());
    }
    return param;
  }

  DOMElement* EmbedItemData::createEmbedXMLElement() {
    if(not getEmbedXMLElement()) {
      setEmbedXMLElement(D(getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      getXMLElement()->getParentNode()->insertBefore(getEmbedXMLElement(),getXMLElement());
      getEmbedXMLElement()->insertBefore(getXMLElement(),nullptr);
    }
    return getEmbedXMLElement();
  }

  bool EmbedItemData::hasParameterXMLElement() const {
    return embed and E(embed)->getFirstElementChildNamed(PV%"Parameter");
  }

  bool EmbedItemData::hasHref() const {
    return embed and embed->hasAttribute(X()%"href");
  }

  bool EmbedItemData::hasParameterHref() const {
    return embed and embed->hasAttribute(X()%"parameterHref");
  }

  DOMElement* EmbedItemData::processFileID(DOMElement *element) {
    if(MBXMLUtils::E(element)->hasAttribute("href")) {
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getXMLElement(),true));
      element->insertBefore(ele2,NULL);
      MBXMLUtils::E(element)->removeAttribute("href");
    }
    if(MBXMLUtils::E(element)->hasAttribute("parameterHref") and getNumberOfParameters()) {
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getParameter(0)->getXMLElement()->getParentNode(),true));
      element->insertBefore(ele2,element->getFirstElementChild());
      MBXMLUtils::E(element)->removeAttribute("parameterHref");
    }
    return E(element)->getTagName()==PV%"Embed"?element->getLastElementChild():element;
  }

}
