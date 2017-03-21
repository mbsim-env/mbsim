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
#include "mainwindow.h"
#include "utils.h"
#include "embed.h"
#include "mbxmlutilshelper/dom.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Group::Group(const string &str, Element *parent) : Element(str,parent), constraints(NULL), observers(NULL) {

    InternalFrame *I = new InternalFrame("I",this,MBSIM%"enableOpenMBVFrameI","plotFeatureFrameI");
    addFrame(I);

//    if(parent) {
//      position.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"position"),"",4));
//
//      orientation.setProperty(new ChoiceProperty2(new RotMatPropertyFactory(MBSIM%"orientation"),"",4));
//
//      frameOfReference.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIM%"frameOfReference"));
//    }
  }

  Group::Group(const Group &g) : Element(g) {
    for(unsigned int i=0; i<g.frame.size(); i++)
      frame.push_back(static_cast<Frame*>(g.frame[i]->clone()));;
    for(unsigned int i=0; i<g.contour.size(); i++)
      contour.push_back(static_cast<Contour*>(g.contour[i]->clone()));;
    for(unsigned int i=0; i<g.group.size(); i++)
      group.push_back(static_cast<Group*>(g.group[i]->clone()));;
    for(unsigned int i=0; i<g.object.size(); i++)
      object.push_back(static_cast<Object*>(g.object[i]->clone()));;
    for(unsigned int i=0; i<g.link.size(); i++)
      link.push_back(static_cast<Link*>(g.link[i]->clone()));;
    for(unsigned int i=0; i<g.constraint.size(); i++)
      constraint.push_back(static_cast<Constraint*>(g.constraint[i]->clone()));;
    for(unsigned int i=0; i<g.observer.size(); i++)
      observer.push_back(static_cast<Observer*>(g.observer[i]->clone()));;
  }

  Group::~Group() {
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
      delete *i;
    for(vector<Group*>::iterator i = group.begin(); i != group.end(); ++i) 
      delete *i;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      delete *i;
    for(vector<Observer*>::iterator i = observer.begin(); i != observer.end(); ++i)
      delete *i;
    for(vector<Element*>::iterator i = removedElement.begin(); i != removedElement.end(); ++i) 
      delete *i;
  }

  Group& Group::operator=(const Group &g) {
    Element::operator=(g);
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
      delete *i;
    for(vector<Group*>::iterator i = group.begin(); i != group.end(); ++i) 
      delete *i;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      delete *i;
    for(vector<Observer*>::iterator i = observer.begin(); i != observer.end(); ++i)
      delete *i;
    for(vector<Element*>::iterator i = removedElement.begin(); i != removedElement.end(); ++i) 
      delete *i;
    frame.clear();
    contour.clear();
    group.clear();
    object.clear();
    link.clear();
    constraint.clear();
    observer.clear();
    removedElement.clear();
    for(unsigned int i=0; i<g.frame.size(); i++)
      frame.push_back(static_cast<Frame*>(g.frame[i]->clone()));;
    for(unsigned int i=0; i<g.contour.size(); i++)
      contour.push_back(static_cast<Contour*>(g.contour[i]->clone()));;
    for(unsigned int i=0; i<g.group.size(); i++)
      group.push_back(static_cast<Group*>(g.group[i]->clone()));;
    for(unsigned int i=0; i<g.object.size(); i++)
      object.push_back(static_cast<Object*>(g.object[i]->clone()));;
    for(unsigned int i=0; i<g.link.size(); i++)
      link.push_back(static_cast<Link*>(g.link[i]->clone()));;
    for(unsigned int i=0; i<g.constraint.size(); i++)
      constraint.push_back(static_cast<Constraint*>(g.constraint[i]->clone()));;
    for(unsigned int i=0; i<g.observer.size(); i++)
      observer.push_back(static_cast<Observer*>(g.observer[i]->clone()));;
    return *this;
  }

  void Group::initialize() {
    Element::initialize();

    for(size_t i=0; i<frame.size(); i++)
      frame[i]->initialize();

    for(size_t i=0; i<contour.size(); i++)
      contour[i]->initialize();

    for(size_t i=0; i<group.size(); i++)
      group[i]->initialize();

    for(size_t i=0; i<object.size(); i++)
      object[i]->initialize();

    for(size_t i=0; i<link.size(); i++)
      link[i]->initialize();

    for(size_t i=0; i<constraint.size(); i++)
      constraint[i]->initialize();

    for(size_t i=0; i<observer.size(); i++)
      observer[i]->initialize();
  }

  void Group::addFrame(Frame* frame_) {
    frame.push_back(frame_);
  }

  void Group::addContour(Contour* contour_) {
    contour.push_back(contour_);
  }

  void Group::addGroup(Group* group_) {
    group.push_back(group_);
  }

  void Group::addObject(Object* object_) {
    object.push_back(object_);
  }

  void Group::addLink(Link* link_) {
    link.push_back(link_);
  }

  void Group::addConstraint(Constraint* constraint_) {
    constraint.push_back(constraint_);
  }

  void Group::addObserver(Observer* observer_) {
    observer.push_back(observer_);
  }

  void Group::removeElement(Element* element) {
    if(dynamic_cast<Frame*>(element)) {
      for (vector<Frame*>::iterator it = frame.begin() ; it != frame.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          frame.erase(it);
          //delete (*it);
          break;
        }
    }
    else if(dynamic_cast<Contour*>(element)) {
      for (vector<Contour*>::iterator it = contour.begin() ; it != contour.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          contour.erase(it);
          //delete (*it);
          break;
        }
    }
    else if(dynamic_cast<Group*>(element)) {
      for (vector<Group*>::iterator it = group.begin() ; it != group.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          group.erase(it);
          //delete (*it);
          break;
        }
    }
    else if(dynamic_cast<Object*>(element)) {
      for (vector<Object*>::iterator it = object.begin() ; it != object.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          object.erase(it);
          //delete (*it);
          break;
        }
    }
    else if(dynamic_cast<Link*>(element)) {
      for (vector<Link*>::iterator it = link.begin() ; it != link.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          link.erase(it);
          //delete (*it);
          break;
        }
    }
    else if(dynamic_cast<Constraint*>(element)) {
      for (vector<Constraint*>::iterator it = constraint.begin() ; it != constraint.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          constraint.erase(it);
          //delete (*it);
          break;
        }
    }
    else if(dynamic_cast<Observer*>(element)) {
      for (vector<Observer*>::iterator it = observer.begin() ; it != observer.end(); ++it)
        if(*it==element) {
          //cout << "erase " << (*it)->getName() << endl;
          observer.erase(it);
          //delete (*it);
          break;
        }
    }
    removedElement.push_back(element);
//    element->deinitialize();
  }

  Group* Group::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
//    Group *group=ObjectFactory::getInstance()->createGroup(e, parent);
    Group *group=Embed<Group>::createAndInit(e,parent);
    if(group) {
//      group->initializeUsingXML(e);
      group->initialize();
    }
    return group;
  }

  void Group::createXMLConstraints() {
    DOMDocument *doc=element->getOwnerDocument();
    constraints = D(doc)->createElement( MBSIM%"constraints" );
    element->insertBefore( constraints, getXMLLinks()->getNextElementSibling() );
  }

  void Group::createXMLObservers() {
    DOMDocument *doc=element->getOwnerDocument();
    observers = D(doc)->createElement( MBSIM%"observers" );
    element->insertBefore( observers, getXMLConstraints()->getNextElementSibling() );
  }

  void Group::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    DOMElement *ombvFrame=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (e != groups) and (e != objects) and (e != links) and (e != constraints) and (e != observers) and (e != ombvFrame))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* Group::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Element::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();
    frames = D(doc)->createElement( MBSIM%"frames" );
    ele0->insertBefore( frames, NULL );
    contours = D(doc)->createElement( MBSIM%"contours" );
    ele0->insertBefore( contours, NULL );
    groups = D(doc)->createElement( MBSIM%"groups" );
    ele0->insertBefore( groups, NULL );
    objects = D(doc)->createElement( MBSIM%"objects" );
    ele0->insertBefore( objects, NULL );
    links = D(doc)->createElement( MBSIM%"links" );
    ele0->insertBefore( links, NULL );

    DOMElement *ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameI" );
//    DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getFrame(0)->getID());
//    ele1->insertBefore(id, NULL);
    ele0->insertBefore( ele1, NULL );

    return ele0;
  }

  DOMElement* Group::processFileID(DOMElement *element) {
    Element::processFileID(element);

    // frames
    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    for(size_t i=1; i<frame.size(); i++) {
      frame[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    // contours
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    for(size_t i=0; i<contour.size(); i++) {
      contour[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    // objects
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"objects")->getFirstElementChild();
    for(size_t i=0; i<object.size(); i++) {
      object[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getFrame(0)->getID());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  DOMElement* Group::initializeUsingXML(DOMElement *element) {
 //   DOMElement *e;
    Element::initializeUsingXML(element);
//    e=element->getFirstElementChild();

//    if(frameOfReference.getProperty())
//      frameOfReference.initializeUsingXML(element);
//
//    // search first element known by Group
//    while(e &&
//        E(e)->getTagName()!=MBSIM%"position" &&
//        E(e)->getTagName()!=MBSIM%"orientation" &&
//        E(e)->getTagName()!=MBSIM%"frames")
//      e=e->getNextElementSibling();
//
//    if(position.getProperty())
//      position.initializeUsingXML(element);
//
//    if(orientation.getProperty())
//      orientation.initializeUsingXML(element);

    // frames
    frames = E(element)->getFirstElementChildNamed(MBSIM%"frames");
    DOMElement *ELE=frames->getFirstElementChild();
    Frame *f;
    while(ELE) {
      f = Embed<Frame>::createAndInit(ELE,this);
      if(f) addFrame(f);
      ELE=ELE->getNextElementSibling();
    }

    // contours
    contours = E(element)->getFirstElementChildNamed(MBSIM%"contours");
    ELE=contours->getFirstElementChild();
    Contour *c;
    while(ELE) {
      c = Embed<Contour>::createAndInit(ELE,this);
      if(c) addContour(c);
      ELE=ELE->getNextElementSibling();
    }

    // groups
    groups = E(element)->getFirstElementChildNamed(MBSIM%"groups");
    ELE=groups->getFirstElementChild();
    Group *g;
    while(ELE) {
      g = Embed<Group>::createAndInit(ELE,this);
      if(g) addGroup(g);
      ELE=ELE->getNextElementSibling();
    }

    // objects
    objects = E(element)->getFirstElementChildNamed(MBSIM%"objects");
    ELE=objects->getFirstElementChild();
    Object *o;
    while(ELE) {
      o = Embed<Object>::createAndInit(ELE,this);
      if(o) addObject(o);
      ELE=ELE->getNextElementSibling();
    }

    // links
    links = E(element)->getFirstElementChildNamed(MBSIM%"links");
    ELE=links->getFirstElementChild();
    Link *l;
    while(ELE) {
      l = Embed<Link>::createAndInit(ELE,this);
      if(l) addLink(l);
      ELE=ELE->getNextElementSibling();
    }

    // constraints
    constraints = E(element)->getFirstElementChildNamed(MBSIM%"constraints");
    if(constraints) {
      ELE=constraints->getFirstElementChild();
      Constraint *constraint;
      while(ELE) {
        constraint = Embed<Constraint>::createAndInit(ELE,this);
        if(constraint) addConstraint(constraint);
        ELE=ELE->getNextElementSibling();
      }
    }

    // observers
    observers = E(element)->getFirstElementChildNamed(MBSIM%"observers");
    if(observers) {
      ELE=observers->getFirstElementChild();
      Observer *obsrv;
      while(ELE) {
        obsrv = Embed<Observer>::createAndInit(ELE,this);
        if(obsrv) addObserver(obsrv);
        ELE=ELE->getNextElementSibling();
      }
    }

    return element;
  }

  Frame* Group::getFrame(const string &name) const {
    size_t i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name)
        return frame[i];
    }
    return NULL;
  }

  Contour* Group::getContour(const string &name) const {
    size_t i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
        return contour[i];
    }
    return NULL;
  }

  Group* Group::getGroup(const string &name) const {
    size_t i;
    for(i=0; i<group.size(); i++) {
      if(group[i]->getName() == name)
        return group[i];
    }
    return NULL;
  }

  Object* Group::getObject(const string &name) const {
    size_t i;
    for(i=0; i<object.size(); i++) {
      if(object[i]->getName() == name)
        return object[i];
    }
    return NULL;
  }

  Link* Group::getLink(const string &name) const {
    size_t i;
    for(i=0; i<link.size(); i++) {
      if(link[i]->getName() == name)
        return link[i];
    }
    return NULL;
  }

  Constraint* Group::getConstraint(const string &name) const {
    size_t i;
    for(i=0; i<constraint.size(); i++) {
      if(constraint[i]->getName() == name)
        return constraint[i];
    }
    return NULL;
  }

  Observer* Group::getObserver(const string &name) const {
    size_t i;
    for(i=0; i<observer.size(); i++) {
      if(observer[i]->getName() == name)
        return observer[i];
    }
    return NULL;
  }

  Element * Group::getChildByContainerAndName(const std::string &container, const std::string &name) const {
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
      return 0;
  }

}
