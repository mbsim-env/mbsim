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
#include "fileitemdata.h"
#include "mainwindow.h"
#include <mbxmlutils/eval.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMNamedNodeMap.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  EmbedItemData::EmbedItemData() : parameters(new Parameters(this)) {
  }

  EmbedItemData::~EmbedItemData() {
    for (auto & it : parameter)
      delete it;
    delete parameters;
    if(parameterFileItem) parameterFileItem->removeReference(this);
    if(fileItem) fileItem->removeReference(this);
  }

  bool EmbedItemData::isActive() {
    if(not embed or not MBXMLUtils::E(embed)->hasAttribute("count"))
      return true;
    mw->updateParameters(this);
    bool active = true;
    try {
      active = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(embed)->getAttribute("count"),element,true))!="0";
    }
    catch(MBXMLUtils::DOMEvalException &ex) {
      mw->setExitBad();
      cerr << ex.getMessage() << endl;
    }
    catch(...) {
      mw->setExitBad();
      cerr << "Unknown exception" << endl;
    }
    return active;
  }

  void EmbedItemData::createParameters() {
    auto param = Parameter::createParameters(createParameterXMLElement());
    for(auto & i : param)
      addParameter(i);
  }

  void EmbedItemData::clearParameters() {
    for (auto it = parameter.begin(); it != parameter.end(); ++it)
      delete *it;
    parameter.erase(parameter.begin(),parameter.end());
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
    delete param;
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
    if(parameterFileItem) return parameterFileItem->getXMLElement();
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
    if(embed and not getNumberOfParameters() and not E(embed)->hasAttribute("count") and not E(embed)->hasAttribute("counterName") and not E(embed)->hasAttribute("href") and not E(embed)->hasAttribute("parameterHref")) {
      embed->getParentNode()->insertBefore(element,embed);
      embed->getParentNode()->removeChild(embed);
      embed = nullptr;
    }
  }

  bool EmbedItemData::hasParameterXMLElement() const {
    return embed and E(embed)->getFirstElementChildNamed(PV%"Parameter");
  }

  void EmbedItemData::setFileItem(FileItemData *fileItem_) {
    fileItem = fileItem_;
    if(fileItem) fileItem->addReference(this);
    setDedicatedFileItem(fileItem);
  }

  void EmbedItemData::setParameterFileItem(FileItemData *parameterFileItem_) {
    if(parameterFileItem) parameterFileItem->removeReference(this);
    parameterFileItem = parameterFileItem_;
    if(parameterFileItem) parameterFileItem->addReference(this);
    setDedicatedParameterFileItem(parameterFileItem);
  }

  DOMElement* EmbedItemData::processIDAndHref(DOMElement *element) {
    if(MBXMLUtils::E(element)->hasAttribute("href")) {
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getXMLElement(),true));
      element->insertBefore(ele2,nullptr);
      boost::filesystem::path orgFileName=E(getXMLElement())->getOriginalFilename();
      DOMProcessingInstruction *filenamePI=ele2->getOwnerDocument()->createProcessingInstruction(X()%"OriginalFilename",
          X()%orgFileName.string());
      ele2->insertBefore(filenamePI, ele2->getFirstChild());
      MBXMLUtils::E(element)->removeAttribute("href");
    }
    if(MBXMLUtils::E(element)->hasAttribute("parameterHref") and getNumberOfParameters()) {
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getParameter(0)->getXMLElement()->getParentNode(),true));
      element->insertBefore(ele2,element->getFirstElementChild());
      boost::filesystem::path orgFileName=E(getParameter(0)->getXMLElement())->getOriginalFilename();
      DOMProcessingInstruction *filenamePI=ele2->getOwnerDocument()->createProcessingInstruction(X()%"OriginalFilename",
          X()%orgFileName.string());
      ele2->insertBefore(filenamePI, ele2->getFirstChild());
      MBXMLUtils::E(element)->removeAttribute("parameterHref");
    }
    return E(element)->getTagName()==PV%"Embed"?element->getLastElementChild():element;
  }

  QString EmbedItemData::getReference() const {
    return dedicatedFileItem?dedicatedFileItem->getName():"";
  }

  void EmbedItemData::updateName() {
    name = QString::fromStdString(MBXMLUtils::E(element)->getAttribute("name"));
    if(name[0]=='{') {
      mw->updateParameters(this,false);
      try{
	name = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(name.toStdString(),getXMLElement(),false)));
	name = name.mid(1,name.size()-2);
      }
      catch(MBXMLUtils::DOMEvalException &e) {
	mw->setExitBad();
	std::cerr << e.getMessage() << std::endl;
      }
      catch(...) {
	mw->setExitBad();
	std::cerr << "Unknwon error" << std::endl;
      }
    }
  }
}
