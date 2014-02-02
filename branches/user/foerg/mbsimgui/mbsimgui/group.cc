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
#include "mbxmlutilshelper/dom.h"

using namespace std;
using namespace MBXMLUtils;
using namespace boost;
using namespace xercesc;

Group::Group(const string &str, Element *parent) : Element(str,parent), position(0,false), orientation(0,false), frameOfReference(0,false) {

  Frame *I = new Frame("I",this);
  addFrame(I);

  if(parent) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(3),"m",MBSIM%"position"));
    position.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatProperty(getEye<string>(3,3,"1","0")),"-",MBSIM%"orientation"));
    orientation.setProperty(new ExtPhysicalVarProperty(input));

    frameOfReference.setProperty(new ParentFrameOfReferenceProperty("",getParent()->getFrame(0)->getXMLPath(this,true),this));
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
  Group *group=ObjectFactory::getInstance()->createGroup(e, parent);
  if(group) {
    group->initializeUsingXML(e);
    group->initialize();
  }
  return group;
}

void Group::initializeUsingXML(DOMElement *element) {
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
    if(E(ELE)->getTagName()==PV%"embed") {
      DOMElement *ELE2 = 0;
      if(E(ELE)->hasAttribute("href"))
        f=Frame::readXMLFile(E(ELE)->getAttribute("href"),this);
      else {
        ELE2 = ELE->getFirstElementChild();
        if(E(ELE2)->getTagName() == PV%"localParameter")
          ELE2 = ELE2->getNextElementSibling();
        f=ObjectFactory::getInstance()->createFrame(ELE2,this);
      }
      if(f) {
        addFrame(f);
        f->initializeUsingXMLEmbed(ELE);
        if(ELE2)
          f->initializeUsingXML(ELE2);
      }
    }
    else {
      f=ObjectFactory::getInstance()->createFrame(ELE,this);
      addFrame(f);
      f->initializeUsingXML(ELE);
    }
    ELE=ELE->getNextElementSibling();
  }

  // contours
  ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
  Contour *c;
  while(ELE) {
    if(E(ELE)->getTagName()==PV%"embed") {
      DOMElement *ELE2 = 0;
      if(E(ELE)->hasAttribute("href"))
        c=Contour::readXMLFile(E(ELE)->getAttribute("href"),this);
      else {
        ELE2 = ELE->getFirstElementChild();
        if(E(ELE2)->getTagName() == PV%"localParameter")
          ELE2 = ELE2->getNextElementSibling();
        c=ObjectFactory::getInstance()->createContour(ELE2,this);
      }
      if(c) {
        addContour(c);
        c->initializeUsingXMLEmbed(ELE);
        if(ELE2)
          c->initializeUsingXML(ELE2);
      }
    }
    else {
      c=ObjectFactory::getInstance()->createContour(ELE,this);
      if(c) {
        addContour(c);
        c->initializeUsingXML(ELE);
      }
    }
    ELE=ELE->getNextElementSibling();
  }

  // groups
  ELE=E(element)->getFirstElementChildNamed(MBSIM%"groups")->getFirstElementChild();
  Group *g;
  while(ELE) {
    if(E(ELE)->getTagName()==PV%"embed") {
      DOMElement *ELE2 = 0;
      if(E(ELE)->hasAttribute("href"))
        g=Group::readXMLFile(E(ELE)->getAttribute("href"),this);
      else {
        ELE2 = ELE->getFirstElementChild();
        if(E(ELE2)->getTagName() == PV%"localParameter")
          ELE2 = ELE2->getNextElementSibling();
        g=ObjectFactory::getInstance()->createGroup(ELE2,this);
      }
      if(g) {
        addGroup(g);
        g->initializeUsingXMLEmbed(ELE);
        if(ELE2)
          g->initializeUsingXML(ELE2);
      }
    }
    else {
      g=ObjectFactory::getInstance()->createGroup(ELE,this);
      if(g) {
        addGroup(g);
        g->initializeUsingXML(ELE);
      }
    }
    ELE=ELE->getNextElementSibling();
  }

  // objects
  ELE=E(element)->getFirstElementChildNamed(MBSIM%"objects")->getFirstElementChild();
  Object *o;
  while(ELE) {
    if(E(ELE)->getTagName()==PV%"embed") {
      DOMElement *ELE2 = 0;
      if(E(ELE)->hasAttribute("href"))
        o=Object::readXMLFile(E(ELE)->getAttribute("href"),this);
      else {
        ELE2 = ELE->getFirstElementChild();
        if(E(ELE2)->getTagName() == PV%"localParameter")
          ELE2 = ELE2->getNextElementSibling();
        o=ObjectFactory::getInstance()->createObject(ELE2,this);
      }
      if(o) {
        addObject(o);
        o->initializeUsingXMLEmbed(ELE);
        if(ELE2)
          o->initializeUsingXML(ELE2);
      }
    }
    else {
      o=ObjectFactory::getInstance()->createObject(ELE,this);
      if(o) {
        addObject(o);
        o->initializeUsingXML(ELE);
      }
    }
    ELE=ELE->getNextElementSibling();
  }

  // links
  ELE=E(element)->getFirstElementChildNamed(MBSIM%"links")->getFirstElementChild();
  Link *l;
  while(ELE) {
    if(E(ELE)->getTagName()==PV%"embed") {
      DOMElement *ELE2 = 0;
      if(E(ELE)->hasAttribute("href"))
        l=Link::readXMLFile(E(ELE)->getAttribute("href"),this);
      else {
        ELE2 = ELE->getFirstElementChild();
        if(E(ELE2)->getTagName() == PV%"localParameter")
          ELE2 = ELE2->getNextElementSibling();
        l=ObjectFactory::getInstance()->createLink(ELE2,this);
      }
      if(l) {
        addLink(l);
        l->initializeUsingXMLEmbed(ELE);
        if(ELE2)
          l->initializeUsingXML(ELE2);
      }
    }
    else {
      l=ObjectFactory::getInstance()->createLink(ELE,this);
      if(l) {
        addLink(l);
        l->initializeUsingXML(ELE);
      }
    }
    ELE=ELE->getNextElementSibling();
  }

  // observers
  if(E(element)->getFirstElementChildNamed(MBSIM%"observers")) {
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"observers")->getFirstElementChild();
    Observer *obsrv;
    while(ELE) {
      if(E(ELE)->getTagName()==PV%"embed") {
        DOMElement *ELE2 = 0;
        if(E(ELE)->hasAttribute("href"))
          obsrv=Observer::readXMLFile(E(ELE)->getAttribute("href"),this);
        else {
          ELE2 = ELE->getFirstElementChild();
          if(E(ELE2)->getTagName() == PV%"localParameter")
            ELE2 = ELE2->getNextElementSibling();
          obsrv=ObjectFactory::getInstance()->createObserver(ELE2,this);
        }
        if(obsrv) {
          addObserver(obsrv);
          obsrv->initializeUsingXMLEmbed(ELE);
          if(ELE2)
            obsrv->initializeUsingXML(ELE2);
        }
      }
      else {
        obsrv=ObjectFactory::getInstance()->createObserver(ELE,this);
        if(obsrv) {
          addObserver(obsrv);
          obsrv->initializeUsingXML(ELE);
        }
      }
      ELE=ELE->getNextElementSibling();
    }
  }

  e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
  if(e)
    getFrame(0)->initializeUsingXML2(e);
  else
    getFrame(0)->setOpenMBVFrame(false);
}

DOMElement* Group::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Element::writeXMLFile(parent);

  DOMElement *ele1;

  if(position.getProperty()) {
    frameOfReference.writeXMLFile(ele0);
    position.writeXMLFile(ele0);
    orientation.writeXMLFile(ele0);
  }

  DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
  ele1 = D(doc)->createElement( MBSIM%"frames" );
  for(int i=1; i<frame.size(); i++)
    if(frame[i]->isEmbedded())
      frame[i]->writeXMLFileEmbed(ele1);
    else
      frame[i]->writeXMLFile(ele1);
  ele0->insertBefore( ele1, NULL );

  ele1 = D(doc)->createElement( MBSIM%"contours" );
  for(int i=0; i<contour.size(); i++)
    if(contour[i]->isEmbedded())
      contour[i]->writeXMLFileEmbed(ele1);
    else
      contour[i]->writeXMLFile(ele1);
  ele0->insertBefore( ele1, NULL );

  ele1 = D(doc)->createElement( MBSIM%"groups" );
  for(int i=0; i<group.size(); i++)
    if(group[i]->isEmbedded())
      group[i]->writeXMLFileEmbed(ele1);
    else
      group[i]->writeXMLFile(ele1);
  ele0->insertBefore( ele1, NULL );

  ele1 = D(doc)->createElement( MBSIM%"objects" );
  for(int i=0; i<object.size(); i++)
    if(object[i]->isEmbedded())
      object[i]->writeXMLFileEmbed(ele1);
    else
      object[i]->writeXMLFile(ele1);
  ele0->insertBefore( ele1, NULL );
  
  ele1 = D(doc)->createElement( MBSIM%"links" );
  for(int i=0; i<link.size(); i++)
    if(link[i]->isEmbedded())
      link[i]->writeXMLFileEmbed(ele1);
    else
      link[i]->writeXMLFile(ele1);
  ele0->insertBefore( ele1, NULL );

  if(observer.size()) { 
    ele1 = D(doc)->createElement( MBSIM%"observers" );
    for(int i=0; i<observer.size(); i++)
      if(observer[i]->isEmbedded())
        observer[i]->writeXMLFileEmbed(ele1);
      else
        observer[i]->writeXMLFile(ele1);
    ele0->insertBefore( ele1, NULL );
  }

  Frame *I = getFrame(0);
  cout << I->openMBVFrame() << endl;
  if(I->openMBVFrame()) {
    ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameI" );
    I->writeXMLFile2(ele1);
    ele0->insertBefore(ele1, NULL);
  }

  return ele0;
}

Element* Group::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") { // absolut path
    if(parent)
      return parent->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  }
  else if (path.substr(0, 3)=="../") // relative path
    return parent->getByPathSearch(path.substr(3));
  else { // local path
    size_t pos0=path.find_first_of("[");
    string container=path.substr(0, pos0);
    size_t pos1=path.find_first_of("]", pos0);
    string searched_name=path.substr(pos0+1, pos1-pos0-1);
    if(path.length()>pos1+1) { // weiter absteigen
      string rest=path.substr(pos1+2);
      if (container=="Group") {
        Group *group = getGroup(searched_name);
        return group?group->getByPathSearch(rest):NULL;
      }
      else if (container=="Object") {
        Object *object = getObject(searched_name);
        return object?object->getByPathSearch(rest):NULL;
      }
      else if (container=="Link") {
        Link *link = getLink(searched_name);
        return link?link->getByPathSearch(rest):NULL;
      }
      else if (container=="Observer") {
        Observer *observer = getObserver(searched_name);
        return observer?observer->getByPathSearch(rest):NULL;
      }
    }
    else {
      if (container=="Frame")
        return getFrame(searched_name);
      else if (container=="Contour")
        return getContour(searched_name);
      else if (container=="Group")
        return getGroup(searched_name);
      else if (container=="Object")
        return getObject(searched_name);
      else if (container=="Link")
        return getLink(searched_name);
      else if (container=="Observer")
        return getObserver(searched_name);
    }
  }
  return NULL;
}

Frame* Group::getFrame(const string &name) {
  int i;
  for(i=0; i<frame.size(); i++) {
    if(frame[i]->getValue() == name)
      return frame[i];
  }
  return NULL;
}

Contour* Group::getContour(const string &name) {
  int i;
  for(i=0; i<contour.size(); i++) {
    if(contour[i]->getValue() == name)
      return contour[i];
  }
  return NULL;
}

Group* Group::getGroup(const string &name) {
  int i;
  for(i=0; i<group.size(); i++) {
    if(group[i]->getValue() == name)
      return group[i];
  }
  return NULL;
}

Object* Group::getObject(const string &name) {
  int i;
  for(i=0; i<object.size(); i++) {
    if(object[i]->getValue() == name)
      return object[i];
  }
  return NULL;
}

Link* Group::getLink(const string &name) {
  int i;
  for(i=0; i<link.size(); i++) {
    if(link[i]->getValue() == name)
      return link[i];
  }
  return NULL;
}

Observer* Group::getObserver(const string &name) {
  int i;
  for(i=0; i<observer.size(); i++) {
    if(observer[i]->getValue() == name)
      return observer[i];
  }
  return NULL;
}




