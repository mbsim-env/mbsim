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

#include <config.h>
#include "embeditemdata.h"
#include "parameter.h"
#include "fileitemdata.h"
#include "mainwindow.h"
#include "utils.h"
#include <mbxmlutils/eval.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMNamedNodeMap.hpp>
#include <xercesc/dom/DOMComment.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  EmbedItemData::EmbedItemData() : name("Unnamed"), parameterEmbedItem(new ParameterEmbedItem(this)) {
  }

  EmbedItemData::~EmbedItemData() {
    for (auto & it : parameter)
      delete it;
    delete parameterEmbedItem;
    if(parameterFileItem) parameterFileItem->removeReference(this);
    if(fileItem) fileItem->removeReference(this);
  }

  bool EmbedItemData::isActive() {
    if(not embed or not E(embed)->hasAttribute("count") or E(embed)->getAttribute("count")!="0")
      return true;
    return false;
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
    if(embed and not getNumberOfParameters() and not E(embed)->hasAttribute("count") and not E(embed)->hasAttribute("counterName") and
                                                 not E(embed)->hasAttribute("href") and not E(embed)->hasAttribute("parameterHref") and
                                                 not E(embed)->hasAttribute("onlyif")) {
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
    if(!embed)
      return element;

    if(E(element)->getTagName()!=PV%"Embed")
      throw runtime_error("Internal error: EmbedItemData::embed is set but element is not a <Embed> element.");

    if(E(element)->hasAttribute("href")) {
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getXMLElement(),true));
      element->insertBefore(ele2,nullptr);
      boost::filesystem::path orgFileName=E(getXMLElement())->getOriginalFilename();
      E(ele2)->addEmbedData("MBXMLUtils_OriginalFilename", orgFileName.string());
      E(ele2)->setOriginalElementLineNumber(E(element)->getLineNumber());
      E(element)->removeAttribute("href");
    }
    if(E(element)->hasAttribute("parameterHref") and getNumberOfParameters()) {
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(getParameter(0)->getXMLElement()->getParentNode(),true));
      element->insertBefore(ele2,element->getFirstElementChild());
      boost::filesystem::path orgFileName=E(getParameter(0)->getXMLElement())->getOriginalFilename();
      E(ele2)->addEmbedData("MBXMLUtils_OriginalFilename", orgFileName.string());
      E(ele2)->setOriginalElementLineNumber(E(element)->getLineNumber());
      E(element)->removeAttribute("parameterHref");
    }
    return element->getLastElementChild();
  }

  QString EmbedItemData::getReference() const {
    return dedicatedFileItem?dedicatedFileItem->getName():"";
  }

  void EmbedItemData::updateName() {
    if(E(element)->getTagName()==PV%"Embed") // A embed element which cannot be handled by mbsimgui
      name = "<unhandled array/pattern>";
    else {
      name = QString::fromStdString(E(element)->getAttribute("name"));
      if(name.contains('{')) {
        // instantiate a new evaluator on mw->eval and restore the old one at scope end
        NewParamLevel npl(mw->eval);

        auto parameterLevels = mw->updateParameters(this);
        auto values = MainWindow::evaluateForAllArrayPattern(parameterLevels, name.toStdString(), getXMLElement(), false, false, true).second;
        // build the evaluated display name
        name.clear();
        set<string> uniqueNames;
        for(auto &v : values)
          try {
            auto curName = mw->eval->cast<string>(v.second);
            if(uniqueNames.insert(curName).second) // only add unique names
              name += QString(" ❙ ") + QString::fromStdString(curName);
          }
          catch(DOMEvalException &e) {
            mw->setExitBad();
            mw->statusBar()->showMessage(("Cannot evaluate element name to string: " + e.getMessage()).c_str());
            std::cerr << "Cannot evaluate element name to string: " << e.getMessage() << std::endl;
          }
          catch(...) {
            mw->setExitBad();
            mw->statusBar()->showMessage("Cannot evaluate element name to string: Unknwon exception");
            std::cerr << "Cannot evaluate element name to string: Unknwon exception" << std::endl;
          }
        if(name.isEmpty())
          name = "<not used>";
        else
          name = name.mid(3);
      }
    }
    auto *cele = E(element)->getFirstCommentChild();
    if(cele)
      comment = QString::fromStdString(X()%cele->getNodeValue());
    else
      comment.clear();
    if(orgIcon.isNull())
      orgIcon=icon;
    if(E(element)->getFirstProcessingInstructionChildNamed("MBSIMGUI_CONTEXT_ACTION")!=nullptr)
      icon = QIcon(new OverlayIconEngine(orgIcon,
        Utils::QIconCached((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"contextactionoverlay.svg").string().c_str())));
    else
      icon = orgIcon;
  }

  void EmbedItemData::updateValues() {
    for(auto & i : parameter)
      i->updateValue();
  }

}
