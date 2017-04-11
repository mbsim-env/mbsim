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
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "utils.h"
#include <boost/lexical_cast.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  EmbeddingPropertyDialog::EmbeddingPropertyDialog(TreeItemData *item_, bool embedding, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), item(item_) {
    addTab("Embedding");
    if(embedding) {
      name = new ExtWidget("Name",new TextWidget);
      addToTab("Embedding",name);

      href = new ExtWidget("File", new FileWidget(item->getName()+".mbsim.xml", "XML model files", "xml files (*.xml)", 1, false, true), true);
      addToTab("Embedding",href);
      count = new ExtWidget("Count",new PhysicalVariableWidget(new ScalarWidget("1")), true);
      addToTab("Embedding",count);
      counterName = new ExtWidget("Counter name", new TextWidget("n"), true);
      addToTab("Embedding",counterName);
      parameterHref = new ExtWidget("Parameter file", new FileWidget(item->getName()+".parameter.xml", "XML parameter files", "xml files (*.xml)", 1, false, true), true);
      addToTab("Embedding",parameterHref);
    }
  }

  DOMElement* EmbeddingPropertyDialog::initializeUsingXML(DOMElement *ele) {
    if(name) {
      static_cast<TextWidget*>(name->getWidget())->setText(item->getName());
      DOMElement *parent = static_cast<DOMElement*>(ele->getParentNode());
      if(E(parent)->getTagName()==PV%"Embed") {
        DOMProcessingInstruction *instr = E(parent)->getFirstProcessingInstructionChildNamed("href");
        if(instr) {
          href->setActive(true);
          static_cast<FileWidget*>(href->getWidget())->setFile(QString::fromStdString(X()%instr->getData()));
        }
        if(E(parent)->hasAttribute("count")) {
          count->setActive(true);
          static_cast<PhysicalVariableWidget*>(count->getWidget())->setValue(QString::fromStdString(E(parent)->getAttribute("count")));
        }
        if(E(parent)->hasAttribute("counterName")) {
          counterName->setActive(true);
          static_cast<TextWidget*>(counterName->getWidget())->setText(QString::fromStdString(E(parent)->getAttribute("counterName")));
        }
        instr = E(parent)->getFirstProcessingInstructionChildNamed("parameterHref");
        if(instr) {
          parameterHref->setActive(true);
          static_cast<FileWidget*>(parameterHref->getWidget())->setFile(QString::fromStdString(X()%instr->getData()));
        }
      }
    }
    return NULL;
  }

  DOMElement* EmbeddingPropertyDialog::writeXMLFile(DOMNode *node, DOMNode *ref) {
    if(name) {
      DOMNode* embedNode = node->getParentNode();
      if(X()%embedNode->getNodeName()!="Embed") {
        DOMDocument *doc=node->getOwnerDocument();
        DOMNode *ele=D(doc)->createElement(PV%"Embed");
        embedNode->insertBefore(ele,node);
        embedNode = ele;
        embedNode->insertBefore(node,NULL);
      }
      int removeHref = 0;
      int removeParameterHref = 0;
      bool addHref = false;
      DOMElement *element = static_cast<DOMElement*>(embedNode);
      item->setName(static_cast<TextWidget*>(name->getWidget())->getText());
      E(element->getLastElementChild())->setAttribute("name",item->getName().toStdString());
      DOMProcessingInstruction *instr = E(element)->getFirstProcessingInstructionChildNamed("href");
      if(href->isActive() and (not static_cast<FileWidget*>(href->getWidget())->getFile().isEmpty())) {
        if(not instr) {
          DOMDocument *doc=element->getOwnerDocument();
          instr=doc->createProcessingInstruction(X()%"href", X()%static_cast<FileWidget*>(href->getWidget())->getFile().toStdString());
          element->insertBefore(instr, element->getFirstChild());
          addHref = true;
        }
        else
          instr->setData(X()%static_cast<FileWidget*>(href->getWidget())->getFile().toStdString());
        removeHref = -1;
      }
      else if(instr) {
        element->removeChild(instr);
        removeHref = 1;
      }
      if(count->isActive()) {
        item->setValue(static_cast<PhysicalVariableWidget*>(count->getWidget())->getValue());
        E(element)->setAttribute("count",item->getValue().toStdString());
      }
      else {
        item->setValue("");
        E(element)->removeAttribute("count");
      }
      if(counterName->isActive()) {
        item->setCounterName(static_cast<TextWidget*>(counterName->getWidget())->getText());
        E(element)->setAttribute("counterName",item->getCounterName().toStdString());
      }
      else {
        item->setCounterName("");
        E(element)->removeAttribute("counterName");
      }
      instr = E(element)->getFirstProcessingInstructionChildNamed("parameterHref");
      if(parameterHref->isActive() and (not static_cast<FileWidget*>(parameterHref->getWidget())->getFile().isEmpty())) {
        if(not instr) {
          DOMDocument *doc=element->getOwnerDocument();
          instr=doc->createProcessingInstruction(X()%"parameterHref", X()%static_cast<FileWidget*>(parameterHref->getWidget())->getFile().toStdString());
          element->insertBefore(instr, element->getFirstChild());
          addHref = true;
        }
        else
          instr->setData(X()%static_cast<FileWidget*>(parameterHref->getWidget())->getFile().toStdString());
        removeParameterHref = -1;
      }
      else if(instr) {
        element->removeChild(instr);
        removeParameterHref = 1;
      }
      if((removeHref + removeParameterHref) > 0) {
        DOMDocument *doc=embedNode->getOwnerDocument();
        instr = E(doc->getDocumentElement())->getFirstProcessingInstructionChildNamed("hrefCount");
        string count = X()%instr->getData();
        if(count == "1")
          doc->getDocumentElement()->removeChild(instr);
        else
          instr->setData(X()%toStr(boost::lexical_cast<int>(count)-1));
      }
      else if(addHref) {
        DOMDocument *doc=embedNode->getOwnerDocument();
        instr = E(doc->getDocumentElement())->getFirstProcessingInstructionChildNamed("hrefCount");
        if(not instr) {
          instr=doc->createProcessingInstruction(X()%"hrefCount", X()%"1");
          doc->getDocumentElement()->insertBefore(instr, doc->getDocumentElement()->getFirstChild());
        }
        else {
          string count = X()%instr->getData();
          instr->setData(X()%toStr(boost::lexical_cast<int>(count)+1));
        }
      }
      if((not href->isActive()) and (not count->isActive()) and (not counterName->isActive()) and (not parameterHref->isActive()) and (not item->getNumberOfParameters())) {
        embedNode->getParentNode()->insertBefore(node,embedNode);
        embedNode->getParentNode()->removeChild(embedNode);
      }
    }
    return NULL;
  }

  void EmbeddingPropertyDialog::toWidget(TreeItemData *item) {
    initializeUsingXML(item->getXMLElement());
  }

  void EmbeddingPropertyDialog::fromWidget(TreeItemData *item) {
    writeXMLFile(item->getXMLElement());
  }

}
