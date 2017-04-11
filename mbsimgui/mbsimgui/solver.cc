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
#include "solver.h"
#include "dynamic_system_solver.h"
#include "objectfactory.h"
#include <QDir>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern DOMImplementation *impl;
  extern DOMLSSerializer *serializer;

  DOMElement* Solver::processHref(DOMElement *element) {
    DOMElement *parent = static_cast<DOMElement*>(element->getParentNode());
    DOMProcessingInstruction *instr = E(parent)->getFirstProcessingInstructionChildNamed("parameterHref");
    if(instr) {
      parent->removeChild(instr);
      DOMElement *parameter = E(parent)->getFirstElementChildNamed(PV%"Parameter");
      if(parameter) {
        DOMDocument *doc = impl->createDocument();
        DOMNode *node = doc->importNode(parameter,true);
        doc->insertBefore(node,NULL);
        serializer->writeToURI(doc,instr->getData());
        E(parent)->setAttribute("parameterHref",X()%instr->getData());
        DOMNode *ps = parameter->getPreviousSibling();
        if(ps and X()%ps->getNodeName()=="#text")
          parameter->getParentNode()->removeChild(ps);
        parameter->getParentNode()->removeChild(parameter);
      }
    }
    instr = E(parent)->getFirstProcessingInstructionChildNamed("href");
    if(instr) {
      parent->removeChild(instr);
      DOMDocument *doc = impl->createDocument();
      DOMNode *node = doc->importNode(element,true);
      doc->insertBefore(node,NULL);
      serializer->writeToURI(doc, instr->getData());
      E(parent)->setAttribute("href",X()%instr->getData());
      DOMNode *ps = element->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        element->getParentNode()->removeChild(ps);
      element->getParentNode()->removeChild(element);
    }
    return element;
  }

  void Solver::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
  }

  DOMElement* Solver::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    element=D(doc)->createElement(getNameSpace()%getType().toStdString());
    parent->insertBefore(element, NULL);
    return element;
  }

  void Solver::initializeUsingXML(DOMElement *element) {
    this->element = element;
  }

}
