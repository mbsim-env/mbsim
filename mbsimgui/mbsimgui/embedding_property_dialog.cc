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

#include <config.h>
#include "embedding_property_dialog.h"
#include "element.h"
#include "parameter.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "utils.h"
#include <boost/lexical_cast.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <unordered_map>
#include <QFileInfo>
#include <QDir>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern QDir mbsDir;
  extern DOMImplementation *impl;

  EmbeddingPropertyDialog::EmbeddingPropertyDialog(EmbedItemData *item_, bool name_, QWidget *parent, const Qt::WindowFlags& f) : PropertyDialog(parent,f), item(item_), name(nullptr), count(nullptr), counterName(nullptr) {
    addTab("Embedding");
    if(name_) {
      name = new ExtWidget("Name",new TextWidget);
      addToTab("Embedding",name);
      count = new ExtWidget("Count",new PhysicalVariableWidget(new ScalarWidget("1")), true);
      addToTab("Embedding",count);
      counterName = new ExtWidget("Counter name", new TextWidget("n"), true);
      addToTab("Embedding",counterName);
    }
    href = new ExtWidget("File", new FileWidget(item->getName()+".mbsim.xml", "XML model files", "xml files (*.xml)", 1, false, true), true);
    href->setDisabled(true);
    addToTab("Embedding",href);
    parameterHref = new ExtWidget("Parameter file", new FileWidget(item->getName()+".parameter.xml", "XML parameter files", "xml files (*.xml)", 1, false, true), true);
    parameterHref->setDisabled(true);
    addToTab("Embedding",parameterHref);
  }

  DOMElement* EmbeddingPropertyDialog::initializeUsingXML(DOMElement *ele) {
    if(name) static_cast<TextWidget*>(name->getWidget())->setText(item->getName());
    DOMElement *parent = item->getEmbedXMLElement();
    if(parent and E(parent)->getTagName()==PV%"Embed") {
      if(count and E(parent)->hasAttribute("count")) {
        count->setActive(true);
        static_cast<PhysicalVariableWidget*>(count->getWidget())->setValue(QString::fromStdString(E(parent)->getAttribute("count")));
      }
      if(counterName and E(parent)->hasAttribute("counterName")) {
        counterName->setActive(true);
        static_cast<TextWidget*>(counterName->getWidget())->setText(QString::fromStdString(E(parent)->getAttribute("counterName")));
      }
      if(E(parent)->hasAttribute("href")) {
        href->setActive(true);
        static_cast<FileWidget*>(href->getWidget())->setFile(QString::fromStdString(E(parent)->getAttribute("href")));
      }
      if(E(parent)->hasAttribute("parameterHref")) {
        parameterHref->setActive(true);
        static_cast<FileWidget*>(parameterHref->getWidget())->setFile(QString::fromStdString(E(parent)->getAttribute("parameterHref")));
      }
    }
    return nullptr;
  }

  DOMElement* EmbeddingPropertyDialog::writeXMLFile(DOMNode *node, DOMNode *ref) {
    DOMElement *embedNode = item->getEmbedXMLElement();
    if(not embedNode) {
      DOMDocument *doc=node->getOwnerDocument();
      embedNode=D(doc)->createElement(PV%"Embed");
      node->getParentNode()->insertBefore(embedNode,node);
      embedNode->insertBefore(node,nullptr);
      item->setEmbedXMLElement(embedNode);
    }
    if(name and not href->isActive())
      E(embedNode->getLastElementChild())->setAttribute("name",static_cast<TextWidget*>(name->getWidget())->getText().toStdString());
    if(count) {
      if(count->isActive())
        E(embedNode)->setAttribute("count",static_cast<PhysicalVariableWidget*>(count->getWidget())->getValue().toStdString());
      else
        E(embedNode)->removeAttribute("count");
    }
    if(counterName) {
      if(counterName->isActive())
        E(embedNode)->setAttribute("counterName",static_cast<TextWidget*>(counterName->getWidget())->getText().toStdString());
      else
        E(embedNode)->removeAttribute("counterName");
    }

    if((not count or not count->isActive()) and (not counterName or not counterName->isActive()) and (not href->isActive()) and (not parameterHref->isActive()) and (not item->getNumberOfParameters())) {
      embedNode->getParentNode()->insertBefore(node,embedNode);
      embedNode->getParentNode()->removeChild(embedNode);
    }
    return nullptr;
  }

  void EmbeddingPropertyDialog::toWidget(EmbedItemData *item) {
    initializeUsingXML(item->getXMLElement());
  }

  void EmbeddingPropertyDialog::fromWidget(EmbedItemData *item) {
    writeXMLFile(item->getXMLElement());
  }

}
