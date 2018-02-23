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
#include <xercesc/dom/DOMNamedNodeMap.hpp>

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

  vector<EmbedItemData*> EmbedItemData::getEmbedItemParents() {
    vector<EmbedItemData*> parents;
    if(getEmbedItemParent()) {
      parents = getEmbedItemParent()->getEmbedItemParents();
      parents.push_back(getEmbedItemParent());
    }
    return parents;
  }

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

  void EmbedItemData::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
  }

  void EmbedItemData::removeXMLElement(bool removeEmbedding) {
    if(removeEmbedding and embed) {
      DOMNode *e = embed->getFirstChild();
      while(e) {
        DOMNode *en=e->getNextSibling();
        embed->removeChild(e);
        e = en;
      }
      DOMNode *ps = embed->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        embed->getParentNode()->removeChild(ps);
      embed->getParentNode()->removeChild(embed);
    }
    else {
      DOMNode *ps = element->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        element->getParentNode()->removeChild(ps);
      element->getParentNode()->removeChild(element);
    }
  }

  DOMElement* EmbedItemData::createParameterXMLElement() {
    DOMElement *param = E(createEmbedXMLElement())->getFirstElementChildNamed(PV%"Parameter");
    if(not param) {
      param = D(getEmbedXMLElement()->getOwnerDocument())->createElement(PV%"Parameter");
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

  void EmbedItemData::maybeRemoveEmbedXMLElement() {
    if(embed and not getNumberOfParameters()) {
      DOMElement *param = E(embed)->getFirstElementChildNamed(PV%"Parameter");
      if(param) {
        DOMNode *ps = param->getPreviousSibling();
        if(ps and X()%ps->getNodeName()=="#text")
          embed->removeChild(ps);
        embed->removeChild(param);
      }
      if(not E(embed)->hasAttribute("count") and not E(embed)->hasAttribute("counterName") and not E(embed)->hasAttribute("href") and not E(embed)->hasAttribute("parameterHref")) {
        embed->getParentNode()->insertBefore(element,embed);
        embed->getParentNode()->removeChild(embed);
        embed = nullptr;
      }
    }
  }

  bool EmbedItemData::hasParameterXMLElement() const {
    return embed and E(embed)->getFirstElementChildNamed(PV%"Parameter");
  }

  DOMElement* EmbedItemData::processIDAndHref(DOMElement *element) {
    if(MBXMLUtils::E(element)->hasAttribute("href")) {
      E(getXMLElement())->setOriginalFilename();
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getXMLElement(),true));
      element->insertBefore(ele2,NULL);
      MBXMLUtils::E(element)->removeAttribute("href");
    }
    else
      E(E(element)->getTagName()==PV%"Embed"?element->getLastElementChild():element)->setOriginalFilename();
    if(MBXMLUtils::E(element)->hasAttribute("parameterHref") and getNumberOfParameters()) {
      E(static_cast<xercesc::DOMElement*>(getParameter(0)->getXMLElement()->getParentNode()))->setOriginalFilename();
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getParameter(0)->getXMLElement()->getParentNode(),true));
      element->insertBefore(ele2,element->getFirstElementChild());
      MBXMLUtils::E(element)->removeAttribute("parameterHref");
    }
    return E(element)->getTagName()==PV%"Embed"?element->getLastElementChild():element;
  }

  DOMElement* EmbedItemData::processHref(DOMElement *element) {
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
