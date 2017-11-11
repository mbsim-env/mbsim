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
#include "element.h"
#include <cmath>
#include "frame.h"
#include "contour.h"
#include "dynamic_system_solver.h"
#include "object.h"
#include "link.h"
#include "constraint.h"
#include "observer.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>
#include <QDir>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern bool absolutePath;
  extern QDir mbsDir;
  extern DOMImplementation *impl;
  extern DOMLSSerializer *serializer;

  int Element::IDcounter=0;

  void Element::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
  }

  DOMElement* Element::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    element=D(doc)->createElement(getNameSpace()%getType().toStdString());
    E(element)->setAttribute("name",getType().toStdString());
    parent->insertBefore(element, NULL);
    return element;
  }

  DOMElement* Element::initializeUsingXML(DOMElement *element) {
    this->element = element;
    config = true;
    return element;
  }

  QString Element::getXMLPath(Element *ref, bool rel) {
    if(rel) {
      vector<Element*> e0, e1;
      Element* element = ref;
      e0.push_back(element);
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        element = element->getParent();
        e0.push_back(element);
      }
      element = parent;
      e1.push_back(element);
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        element = element->getParent();
        e1.push_back(element);
      }
      int imatch=0;
      for(vector<Element*>::iterator i0 = e0.end()-1, i1 = e1.end()-1 ; (i0 != e0.begin()-1) && (i1 != e1.begin()-1) ; i0--, i1--) 
        if(*i0 == *i1) imatch++;
      QString type;
      if(dynamic_cast<Frame*>(this))
        type = "Frame";
      else if(dynamic_cast<Contour*>(this))
        type = "Contour";
      else if(dynamic_cast<Group*>(this))
        type = "Group";
      else if(dynamic_cast<Object*>(this))
        type = "Object";
      else if(dynamic_cast<Link*>(this))
        type = "Link";
      else if(dynamic_cast<Constraint*>(this))
        type = "Constraint";
      else if(dynamic_cast<Observer*>(this))
        type = "Observer";
      else 
        type = getType();
      QString str = type + "[" + getName() + "]";
      for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
        if(dynamic_cast<Group*>(*i1))
          str = QString("Group[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(*i1))
          str = QString("Object[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Link*>(*i1))
          str = QString("Link[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Constraint*>(*i1))
          str = QString("Constraint[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Observer*>(*i1))
          str = QString("Observer[") + (*i1)->getName() + "]/" + str;
        else
          str = "";
      }
      for(int i=0; i<int(e0.size())-imatch; i++)
        str = "../" + str;
      return str;
    } else {
      QString type;
      if(dynamic_cast<Frame*>(this))
        type = "Frame";
      else if(dynamic_cast<Contour*>(this))
        type = "Contour";
      else if(dynamic_cast<Group*>(this))
        type = "Group";
      else if(dynamic_cast<Object*>(this))
        type = "Object";
      else if(dynamic_cast<Link*>(this))
        type = "Link";
      else if(dynamic_cast<Constraint*>(this))
        type = "Constraint";
      else if(dynamic_cast<Observer*>(this))
        type = "Observer";
      else 
        type = getType();
      QString str = type + "[" + getName() + "]";
      Element* element = parent;
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        if(dynamic_cast<Group*>(element))
          str = QString("Group[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(element))
          str = QString("Object[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Link*>(element))
          str = QString("Link[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Constraint*>(element))
          str = QString("Constraint[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Observer*>(element))
          str = QString("Observer[") + element->getName() + "]/" + str;
        else
          str = "";
        element = element->getParent();
      }
      str = "/" + str;
      return str;
    }
  }

  vector<EmbedItemData*> Element::getParents() {
    vector<EmbedItemData*> parents;
    if(getParent()) {
      parents = getParent()->getParents();
      parents.push_back(getParent());
    }
    return parents;
  }

}
