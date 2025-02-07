/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "element.h"
#include <cmath>
#include "element_view.h"
#include "parameter_view.h"
#include "frame.h"
#include "contour.h"
#include "dynamic_system_solver.h"
#include "object.h"
#include "link_.h"
#include "constraint.h"
#include "observer.h"
#include "mainwindow.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>
#include <QDir>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Element::Element() {
    ID = mw->getID(this);
  }

  DOMElement* Element::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    E(element)->setAttribute("name",getType().toStdString());
    parent->insertBefore(element, nullptr);
    return element;
  }

  QString Element::getXMLName() {
    return QString::fromStdString(E(element)->getAttribute("name"));
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
      for(auto i0 = e0.end()-1, i1 = e1.end()-1 ; (i0 != e0.begin()-1) && (i1 != e1.begin()-1) ; i0--, i1--) 
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
        type = "<unknown container>";
      QString str = type + "[" + getXMLName() + "]";
      for(auto i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
        if(dynamic_cast<Group*>(*i1))
          str = QString("Group[") + (*i1)->getXMLName() + "]/" + str;
        else if(dynamic_cast<Object*>(*i1))
          str = QString("Object[") + (*i1)->getXMLName() + "]/" + str;
        else if(dynamic_cast<Link*>(*i1))
          str = QString("Link[") + (*i1)->getXMLName() + "]/" + str;
        else if(dynamic_cast<Constraint*>(*i1))
          str = QString("Constraint[") + (*i1)->getXMLName() + "]/" + str;
        else if(dynamic_cast<Observer*>(*i1))
          str = QString("Observer[") + (*i1)->getXMLName() + "]/" + str;
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
        type = "<unknown container>";
      QString str = type + "[" + getXMLName() + "]";
      Element* element = parent;
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        if(dynamic_cast<Group*>(element))
          str = QString("Group[") + element->getXMLName() + "]/" + str;
        else if(dynamic_cast<Object*>(element))
          str = QString("Object[") + element->getXMLName() + "]/" + str;
        else if(dynamic_cast<Link*>(element))
          str = QString("Link[") + element->getXMLName() + "]/" + str;
        else if(dynamic_cast<Constraint*>(element))
          str = QString("Constraint[") + element->getXMLName() + "]/" + str;
        else if(dynamic_cast<Observer*>(element))
          str = QString("Observer[") + element->getXMLName() + "]/" + str;
        else
          str = "";
        element = element->getParent();
      }
      str = "/" + str;
      return str;
    }
  }

  DOMElement* Element::processIDAndHref(DOMElement* element) {
    element = EmbedItemData::processIDAndHref(element);
    return element;
  }

  void Element::updateStatus() {
    enabled = (not parent or parent->getEnabled()) and isActive();
    if(getModelIndex().model()==mw->getElementView()->model())
      emit mw->getElementView()->model()->dataChanged( getModelIndex(), getModelIndex(), { Qt::UserRole });
    if(getModelIndex().model()==mw->getParameterView()->model())
      emit mw->getParameterView()->model()->dataChanged( getModelIndex(), getModelIndex(), { Qt::UserRole });
  }

  void Element::emitDataChangedOnChildren() {
    if(getModelIndex().isValid()) {
      int row=0;
      while(true) {
        auto containerIndex=getModelIndex().model()->index(row++,0,getModelIndex());
        if(!containerIndex.isValid())
          break;
        emit mw->getElementView()->model()->dataChanged(containerIndex, containerIndex, { Qt::UserRole });
      }
    }
  }

}
