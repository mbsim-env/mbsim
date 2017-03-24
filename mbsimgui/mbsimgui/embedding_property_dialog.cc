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

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  EmbeddingPropertyDialog::EmbeddingPropertyDialog(Element *element_, bool embedding, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), element(element_), embed(0) {
    addTab("Embedding");
    if(embedding) {
      embed = new ExtWidget("Embed", new EmbedWidget, true);
      addToTab("Embedding",embed);
      name = new ExtWidget("Name",new TextWidget);
      addToTab("Embedding",name);
    }
  }

  DOMElement* EmbeddingPropertyDialog::initializeUsingXML(DOMElement *ele) {
    if(embed) {
      static_cast<TextWidget*>(name->getWidget())->setText(element->getName());
      DOMElement *parent = static_cast<DOMElement*>(ele->getParentNode());
      if(E(parent)->getTagName()==PV%"Embed")
        embed->initializeUsingXML(parent);
    }
    return NULL;
  }

  DOMElement* EmbeddingPropertyDialog::writeXMLFile(DOMNode *node, DOMNode *ref) {
    if(embed) {
      element->setName(static_cast<TextWidget*>(name->getWidget())->getText());
      element->setCounterName(static_cast<EmbedWidget*>(embed->getWidget())->getCounterName().toStdString());
      element->setValue(static_cast<EmbedWidget*>(embed->getWidget())->getCount());
      DOMNode* embedNode = node->getParentNode();
      if(embed->isActive()) {
        if(X()%embedNode->getNodeName()!="Embed") {
          DOMDocument *doc=node->getOwnerDocument();
          DOMNode *ele=D(doc)->createElement(PV%"Embed");
          embedNode->insertBefore(ele,node);
          embedNode = ele;
          embedNode->insertBefore(node,NULL);
        }
        embed->writeXMLFile(embedNode,ref);
      }
      else {
        if(X()%embedNode->getNodeName()=="Embed") {
          if(element->getNumberOfParameters()) {
            E(static_cast<DOMElement*>(embedNode))->removeAttribute("href");
            E(static_cast<DOMElement*>(embedNode))->removeAttribute("count");
            E(static_cast<DOMElement*>(embedNode))->removeAttribute("counterName");
            E(static_cast<DOMElement*>(embedNode))->removeAttribute("parameterHref");
          }
          else {
            embedNode->getParentNode()->insertBefore(node,embedNode);
            embedNode->getParentNode()->removeChild(embedNode);
          }
        }
      }
    }
    return NULL;
  }

  void EmbeddingPropertyDialog::toWidget(Element *element) {
    initializeUsingXML(element->getXMLElement());
  }

  void EmbeddingPropertyDialog::fromWidget(Element *element) {
    writeXMLFile(element->getXMLElement());
  }

}
