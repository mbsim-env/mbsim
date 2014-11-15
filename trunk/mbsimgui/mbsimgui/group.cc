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
#include <QtGui/QMenu>
#include <QtGui/QInputDialog>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QVBoxLayout>
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "object.h"
#include "link.h"
#include "mainwindow.h"
#include "observer.h"
#include "basic_properties.h"
#include "utils.h"
#include "embed.h"
#include "mbxmlutilshelper/dom.h"

using namespace std;
using namespace MBXMLUtils;
using namespace boost;
using namespace xercesc;

namespace MBSimGUI {

  Group::Group(const string &str, Element *parent) : Element(str,parent), position(0,false), orientation(0,false), frameOfReference(0,false) {

    Frame *I = new Frame("I",this);
    addFrame(I);

    if(parent) {
      position.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"position"),"",4));

      orientation.setProperty(new ChoiceProperty2(new RotMatPropertyFactory(MBSIM%"orientation"),"",4));

      frameOfReference.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIM%"frameOfReference"));
    }
  }

  Group::Group(const Group &g) : Element(g), position(g.position), orientation(g.orientation), frameOfReference(g.frameOfReference) {
    for(unsigned int i=0; i<g.group.size(); i++)
      group.push_back(static_cast<Group*>(g.group[i]->clone()));;
    for(unsigned int i=0; i<g.object.size(); i++)
      object.push_back(static_cast<Object*>(g.object[i]->clone()));;
    for(unsigned int i=0; i<g.link.size(); i++)
      link.push_back(static_cast<Link*>(g.link[i]->clone()));;
    for(unsigned int i=0; i<g.frame.size(); i++)
      frame.push_back(static_cast<Frame*>(g.frame[i]->clone()));;
    for(unsigned int i=0; i<g.contour.size(); i++)
      contour.push_back(static_cast<Contour*>(g.contour[i]->clone()));;
    for(unsigned int i=0; i<g.observer.size(); i++)
      observer.push_back(static_cast<Observer*>(g.observer[i]->clone()));;
  }

  Group::~Group() {
    for(vector<Group*>::iterator i = group.begin(); i != group.end(); ++i) 
      delete *i;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
      delete *i;
    for(vector<Observer*>::iterator i = observer.begin(); i != observer.end(); ++i)
      delete *i;
    for(vector<Element*>::iterator i = removedElement.begin(); i != removedElement.end(); ++i) 
      delete *i;
  }

  Group& Group::operator=(const Group &g) {
    Element::operator=(g);
    for(vector<Group*>::iterator i = group.begin(); i != group.end(); ++i) 
      delete *i;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
      delete *i;
    for(vector<Observer*>::iterator i = observer.begin(); i != observer.end(); ++i)
      delete *i;
    for(vector<Element*>::iterator i = removedElement.begin(); i != removedElement.end(); ++i) 
      delete *i;
    group.clear();
    object.clear();
    link.clear();
    frame.clear();
    contour.clear();
    observer.clear();
    removedElement.clear();
    position=g.position; 
    orientation=g.orientation; 
    frameOfReference=g.frameOfReference;
    for(unsigned int i=0; i<g.group.size(); i++)
      group.push_back(static_cast<Group*>(g.group[i]->clone()));;
    for(unsigned int i=0; i<g.object.size(); i++)
      object.push_back(static_cast<Object*>(g.object[i]->clone()));;
    for(unsigned int i=0; i<g.link.size(); i++)
      link.push_back(static_cast<Link*>(g.link[i]->clone()));;
    for(unsigned int i=0; i<g.frame.size(); i++)
      frame.push_back(static_cast<Frame*>(g.frame[i]->clone()));;
    for(unsigned int i=0; i<g.contour.size(); i++)
      contour.push_back(static_cast<Contour*>(g.contour[i]->clone()));;
    for(unsigned int i=0; i<g.observer.size(); i++)
      observer.push_back(static_cast<Observer*>(g.observer[i]->clone()));;
  }

  void Group::initialize() {
    Element::initialize();

    for(int i=0; i<frame.size(); i++)
      frame[i]->initialize();

    for(int i=0; i<contour.size(); i++)
      contour[i]->initialize();

    for(int i=0; i<group.size(); i++)
      group[i]->initialize();

    for(int i=0; i<object.size(); i++)
      object[i]->initialize();

    for(int i=0; i<link.size(); i++)
      link[i]->initialize();

    for(int i=0; i<observer.size(); i++)
      observer[i]->initialize();

    if(frameOfReference.getProperty())
      frameOfReference.initialize();
  }

  int Group::getqSize() {
    int qSize = 0;
    //  if(getContainerGroup()) {
    //    for(int i=0; i<getContainerGroup()->childCount(); i++)
    //      qSize += getGroup(i)->getqSize();
    //  }
    //  if(getContainerObject()) {
    //    for(int i=0; i<getContainerObject()->childCount(); i++)
    //      qSize += getObject(i)->getqSize();
    //  }
    return qSize;
  }

  int Group::getuSize() {
    int uSize = 0;
    //  if(getContainerGroup()) {
    //    for(int i=0; i<getContainerGroup()->childCount(); i++)
    //      uSize += getGroup(i)->getuSize();
    //  }
    //  if(getContainerObject()) {
    //    for(int i=0; i<getContainerObject()->childCount(); i++)
    //      uSize += getObject(i)->getuSize();
    //  }
    return uSize;
  }

  int Group::getxSize() {
    int xSize = 0;
    //  if(getContainerGroup()) {
    //    for(int i=0; i<getContainerGroup()->childCount(); i++)
    //      xSize += getGroup(i)->getxSize();
    //  }
    //  if(getContainerLink()) {
    //    for(int i=0; i<getContainerLink()->childCount(); i++)
    //      xSize += getLink(i)->getxSize();
    //  }
    return xSize;
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
    element->deinitialize();
  }

  Group* Group::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
//    Group *group=ObjectFactory::getInstance()->createGroup(e, parent);
    Group *group=Embed<Group>::createAndInit(e,parent);
    if(group) {
//      group->initializeUsingXML(e);
      group->initialize();
    }
    return group;
  }

  DOMElement* Group::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Element::initializeUsingXML(element);
    e=element->getFirstElementChild();

    if(frameOfReference.getProperty())
      frameOfReference.initializeUsingXML(element);

    // search first element known by Group
    while(e && 
        E(e)->getTagName()!=MBSIM%"position" &&
        E(e)->getTagName()!=MBSIM%"orientation" &&
        E(e)->getTagName()!=MBSIM%"frames")
      e=e->getNextElementSibling();

    if(position.getProperty())
      position.initializeUsingXML(element);

    if(orientation.getProperty())
      orientation.initializeUsingXML(element);

    // frames
    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    Frame *f;
    while(ELE) {
      f = Embed<Frame>::createAndInit(ELE,this);
      if(f) addFrame(f);
      ELE=ELE->getNextElementSibling();
    }

    // contours
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    Contour *c;
    while(ELE) {
      c = Embed<Contour>::createAndInit(ELE,this);
      if(c) addContour(c);
      ELE=ELE->getNextElementSibling();
    }

    // groups
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"groups")->getFirstElementChild();
    Group *g;
    while(ELE) {
      g = Embed<Group>::createAndInit(ELE,this);
      if(g) addGroup(g);
      ELE=ELE->getNextElementSibling();
    }

    // objects
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"objects")->getFirstElementChild();
    Object *o;
    while(ELE) {
      o = Embed<Object>::createAndInit(ELE,this);
      if(o) addObject(o);
      ELE=ELE->getNextElementSibling();
    }

    // links
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"links")->getFirstElementChild();
    Link *l;
    while(ELE) {
      l = Embed<Link>::createAndInit(ELE,this);
      if(l) addLink(l);
      ELE=ELE->getNextElementSibling();
    }

    // observers
    if(E(element)->getFirstElementChildNamed(MBSIM%"observers")) {
      ELE=E(element)->getFirstElementChildNamed(MBSIM%"observers")->getFirstElementChild();
      Observer *obsrv;
      while(ELE) {
        obsrv = Embed<Observer>::createAndInit(ELE,this);
        if(obsrv) addObserver(obsrv);
        ELE=ELE->getNextElementSibling();
      }
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    if(e)
      getFrame(0)->initializeUsingXML2(e);
    else
      getFrame(0)->setOpenMBVFrame(false);

    return element;
  }

  DOMElement* Group::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Element::writeXMLFile(parent);

    DOMElement *ele1;

    if(position.getProperty()) {
      frameOfReference.writeXMLFile(ele0);
      position.writeXMLFile(ele0);
      orientation.writeXMLFile(ele0);
    }

    DOMDocument *doc=ele0->getOwnerDocument();
    //  DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    ele1 = D(doc)->createElement( MBSIM%"frames" );
    for(int i=1; i<frame.size(); i++)
      Embed<Frame>::writeXML(frame[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ele1 = D(doc)->createElement( MBSIM%"contours" );
    for(int i=0; i<contour.size(); i++)
      Embed<Contour>::writeXML(contour[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ele1 = D(doc)->createElement( MBSIM%"groups" );
    for(int i=0; i<group.size(); i++)
      Embed<Group>::writeXML(group[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ele1 = D(doc)->createElement( MBSIM%"objects" );
    for(int i=0; i<object.size(); i++)
      Embed<Object>::writeXML(object[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ele1 = D(doc)->createElement( MBSIM%"links" );
    for(int i=0; i<link.size(); i++)
      Embed<Link>::writeXML(link[i],ele1);
    ele0->insertBefore( ele1, NULL );

    if(observer.size()) { 
      ele1 = D(doc)->createElement( MBSIM%"observers" );
      for(int i=0; i<observer.size(); i++)
        Embed<Observer>::writeXML(observer[i],ele1);
      ele0->insertBefore( ele1, NULL );
    }

    Frame *I = getFrame(0);
    if(I->openMBVFrame()) {
      ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameI" );
      I->writeXMLFile2(ele1);
      ele0->insertBefore(ele1, NULL);
    }

    return ele0;
  }

  Frame* Group::getFrame(const string &name) const {
    int i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name)
        return frame[i];
    }
    return NULL;
  }

  Contour* Group::getContour(const string &name) const {
    int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
        return contour[i];
    }
    return NULL;
  }

  Group* Group::getGroup(const string &name) const {
    int i;
    for(i=0; i<group.size(); i++) {
      if(group[i]->getName() == name)
        return group[i];
    }
    return NULL;
  }

  Object* Group::getObject(const string &name) const {
    int i;
    for(i=0; i<object.size(); i++) {
      if(object[i]->getName() == name)
        return object[i];
    }
    return NULL;
  }

  Link* Group::getLink(const string &name) const {
    int i;
    for(i=0; i<link.size(); i++) {
      if(link[i]->getName() == name)
        return link[i];
    }
    return NULL;
  }

  Observer* Group::getObserver(const string &name) const {
    int i;
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
