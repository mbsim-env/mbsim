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
#include "group.h"
#include "frame.h"
#include "contour.h"
#include "object.h"
#include "link.h"
#include "constraint.h"
#include "observer.h"
#include "objectfactory.h"
#include "utils.h"
#include "embed.h"
#include "mbxmlutilshelper/dom.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std; using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Group::Group() : constraints(nullptr), observers(nullptr) {

    InternalFrame *I = new InternalFrame("I",MBSIM%"enableOpenMBVFrameI","plotFeatureFrameI");
    addFrame(I);
  }

  Group::~Group() {
    for(auto & i : frame)
      delete i;
    for(auto & i : contour)
      delete i;
    for(auto & i : group)
      delete i;
    for(auto & i : object)
      delete i;
    for(auto & i : link)
      delete i;
    for(auto & i : constraint)
      delete i;
    for(auto & i : observer)
      delete i;
    for(auto & i : removedElement)
      delete i;
  }

  void Group::addFrame(Frame* frame_) {
    frame.push_back(frame_);
    frame_->setParent(this);
  }

  void Group::addContour(Contour* contour_) {
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  void Group::addGroup(Group* group_) {
    group.push_back(group_);
    group_->setParent(this);
  }

  void Group::addObject(Object* object_) {
    object.push_back(object_);
    object_->setParent(this);
  }

  void Group::addLink(Link* link_) {
    link.push_back(link_);
    link_->setParent(this);
  }

  void Group::addConstraint(Constraint* constraint_) {
    constraint.push_back(constraint_);
    constraint_->setParent(this);
  }

  void Group::addObserver(Observer* observer_) {
    observer.push_back(observer_);
    observer_->setParent(this);
  }

  void Group::removeElement(Element* element) {
    if(dynamic_cast<Frame*>(element)) {
      for (auto it = frame.begin() ; it != frame.end(); ++it)
        if(*it==element) {
          frame.erase(it);
          break;
        }
    }
    else if(dynamic_cast<Contour*>(element)) {
      for (auto it = contour.begin() ; it != contour.end(); ++it)
        if(*it==element) {
          contour.erase(it);
          break;
        }
    }
    else if(dynamic_cast<Group*>(element)) {
      for (auto it = group.begin() ; it != group.end(); ++it)
        if(*it==element) {
          group.erase(it);
          break;
        }
    }
    else if(dynamic_cast<Object*>(element)) {
      for (auto it = object.begin() ; it != object.end(); ++it)
        if(*it==element) {
          object.erase(it);
          break;
        }
    }
    else if(dynamic_cast<Link*>(element)) {
      for (auto it = link.begin() ; it != link.end(); ++it)
        if(*it==element) {
          link.erase(it);
          break;
        }
    }
    else if(dynamic_cast<Constraint*>(element)) {
      for (auto it = constraint.begin() ; it != constraint.end(); ++it)
        if(*it==element) {
          constraint.erase(it);
          break;
        }
    }
    else if(dynamic_cast<Observer*>(element)) {
      for (auto it = observer.begin() ; it != observer.end(); ++it)
        if(*it==element) {
          observer.erase(it);
          break;
        }
    }
    removedElement.push_back(element);
  }

  void Group::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (e != groups) and (e != objects) and (e != links) and (e != constraints) and (e != observers) and (E(e)->getTagName() != MBSIM%"enableOpenMBVFrameI") and (E(e)->getTagName() != MBSIM%"plotFeatureFrameI"))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* Group::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Element::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();
    frames = D(doc)->createElement( MBSIM%"frames" );
    ele0->insertBefore( frames, nullptr );
    contours = D(doc)->createElement( MBSIM%"contours" );
    ele0->insertBefore( contours, nullptr );
    groups = D(doc)->createElement( MBSIM%"groups" );
    ele0->insertBefore( groups, nullptr );
    objects = D(doc)->createElement( MBSIM%"objects" );
    ele0->insertBefore( objects, nullptr );
    links = D(doc)->createElement( MBSIM%"links" );
    ele0->insertBefore( links, nullptr );
    constraints = D(doc)->createElement( MBSIM%"constraints" );
    ele0->insertBefore( constraints, nullptr );
    observers = D(doc)->createElement( MBSIM%"observers" );
    ele0->insertBefore( observers, nullptr );

    DOMElement *ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameI" );
    ele0->insertBefore( ele1, nullptr );

    return ele0;
  }

  DOMElement* Group::processFileID(DOMElement *element) {
    element = Element::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    for(size_t i=1; i<frame.size(); i++) {
      frame[i]->processFileID(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    for(auto & i : contour) {
      i->processFileID(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"groups")->getFirstElementChild();
    for(auto & i : group) {
      i->processFileID(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"objects")->getFirstElementChild();
    for(auto & i : object) {
      i->processFileID(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"links")->getFirstElementChild();
    for(auto & i : link) {
      i->processFileID(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"constraints");
    if(ELE) {
      ELE=ELE->getFirstElementChild();
      for(auto & i : constraint) {
        i->processFileID(ELE);
        ELE=ELE->getNextElementSibling();
      }
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"observers");
    if(ELE) {
      ELE=ELE->getFirstElementChild();
      for(auto & i : observer) {
        i->processFileID(ELE);
        ELE=ELE->getNextElementSibling();
      }
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getFrame(0)->getID().toStdString());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* Group::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);

    frames = E(element)->getFirstElementChildNamed(MBSIM%"frames");
    DOMElement *ELE=frames->getFirstElementChild();
    Frame *f;
    while(ELE) {
      f = Embed<Frame>::createAndInit(ELE);
      if(f) addFrame(f);
      ELE=ELE->getNextElementSibling();
    }

    contours = E(element)->getFirstElementChildNamed(MBSIM%"contours");
    ELE=contours->getFirstElementChild();
    Contour *c;
    while(ELE) {
      c = Embed<Contour>::createAndInit(ELE);
      if(c) addContour(c);
      ELE=ELE->getNextElementSibling();
    }

    groups = E(element)->getFirstElementChildNamed(MBSIM%"groups");
    ELE=groups->getFirstElementChild();
    Group *g;
    while(ELE) {
      g = Embed<Group>::createAndInit(ELE);
      if(g) addGroup(g);
      ELE=ELE->getNextElementSibling();
    }

    objects = E(element)->getFirstElementChildNamed(MBSIM%"objects");
    ELE=objects->getFirstElementChild();
    Object *o;
    while(ELE) {
      o = Embed<Object>::createAndInit(ELE);
      if(o) addObject(o);
      ELE=ELE->getNextElementSibling();
    }

    links = E(element)->getFirstElementChildNamed(MBSIM%"links");
    ELE=links->getFirstElementChild();
    Link *l;
    while(ELE) {
      l = Embed<Link>::createAndInit(ELE);
      if(l) addLink(l);
      ELE=ELE->getNextElementSibling();
    }

    constraints = E(element)->getFirstElementChildNamed(MBSIM%"constraints");
    ELE=constraints->getFirstElementChild();
    Constraint *constraint;
    while(ELE) {
      constraint = Embed<Constraint>::createAndInit(ELE);
      if(constraint) addConstraint(constraint);
      ELE=ELE->getNextElementSibling();
    }

    observers = E(element)->getFirstElementChildNamed(MBSIM%"observers");
    ELE=observers->getFirstElementChild();
    Observer *obsrv;
    while(ELE) {
      obsrv = Embed<Observer>::createAndInit(ELE);
      if(obsrv) addObserver(obsrv);
      ELE=ELE->getNextElementSibling();
    }

    return element;
  }

  Frame* Group::getFrame(const QString &name) const {
    size_t i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name)
        return frame[i];
    }
    return nullptr;
  }

  Contour* Group::getContour(const QString &name) const {
    size_t i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
        return contour[i];
    }
    return nullptr;
  }

  Group* Group::getGroup(const QString &name) const {
    size_t i;
    for(i=0; i<group.size(); i++) {
      if(group[i]->getName() == name)
        return group[i];
    }
    return nullptr;
  }

  Object* Group::getObject(const QString &name) const {
    size_t i;
    for(i=0; i<object.size(); i++) {
      if(object[i]->getName() == name)
        return object[i];
    }
    return nullptr;
  }

  Link* Group::getLink(const QString &name) const {
    size_t i;
    for(i=0; i<link.size(); i++) {
      if(link[i]->getName() == name)
        return link[i];
    }
    return nullptr;
  }

  Constraint* Group::getConstraint(const QString &name) const {
    size_t i;
    for(i=0; i<constraint.size(); i++) {
      if(constraint[i]->getName() == name)
        return constraint[i];
    }
    return nullptr;
  }

  Observer* Group::getObserver(const QString &name) const {
    size_t i;
    for(i=0; i<observer.size(); i++) {
      if(observer[i]->getName() == name)
        return observer[i];
    }
    return nullptr;
  }

  Element * Group::getChildByContainerAndName(const QString &container, const QString &name) const {
    if (container == "Object")
      return getObject(name);
    else if (container == "Link")
      return getLink(name);
    else if (container == "Constraint")
      return getConstraint(name);
    else if (container == "Group")
      return getGroup(name);
    else if (container == "Frame")
      return getFrame(name);
    else if (container == "Contour")
      return getContour(name);
    else if (container == "Observer")
      return getObserver(name);
    else
      return nullptr;
  }

  int Group::getIndexOfFrame(Frame *frame_) {
    for(size_t i=0; i<frame.size(); i++)
      if(frame[i] == frame_)
        return i;
    return -1;
  }

  int Group::getIndexOfContour(Contour *contour_) {
    for(size_t i=0; i<contour.size(); i++)
      if(contour[i] == contour_)
        return i;
    return -1;
  }

  int Group::getIndexOfGroup(Group *group_) {
    for(size_t i=0; i<group.size(); i++)
      if(group[i] == group_)
        return i;
    return -1;
  }

  int Group::getIndexOfObject(Object *object_) {
    for(size_t i=0; i<object.size(); i++)
      if(object[i] == object_)
        return i;
    return -1;
  }

  int Group::getIndexOfLink(Link *link_) {
    for(size_t i=0; i<link.size(); i++)
      if(link[i] == link_)
        return i;
    return -1;
  }

  int Group::getIndexOfConstraint(Constraint *constraint_) {
    for(size_t i=0; i<constraint.size(); i++)
      if(constraint[i] == constraint_)
        return i;
    return -1;
  }

  int Group::getIndexOfObserver(Observer *observer_) {
    for(size_t i=0; i<observer.size(); i++)
      if(observer[i] == observer_)
        return i;
    return -1;
  }

  void Group::setEmbeded(bool embeded) {
    Element::setEmbeded(embeded);
    for(auto & i : frame)
      i->setEmbeded(embeded);
    for(auto & i : contour)
      i->setEmbeded(embeded);
    for(auto & i : group)
      i->setEmbeded(embeded);
    for(auto & i : object)
      i->setEmbeded(embeded);
    for(auto & i : link)
      i->setEmbeded(embeded);
    for(auto & i : constraint)
      i->setEmbeded(embeded);
    for(auto & i : observer)
      i->setEmbeded(embeded);
  }

}
