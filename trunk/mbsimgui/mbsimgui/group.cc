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
#include "rigidbody.h"
#include "constraint.h"
#include "spring_damper.h"
#include "joint.h"
#include "kinetic_excitation.h"
#include "contact.h"
#include "signal_.h"
#include "frame.h"
#include "contour.h"
#include "basic_properties.h"
#include <string>
#include "utils.h"
#include "observer.h"

using namespace std;
using namespace MBXMLUtils;

Group::Group(const string &str, Element *parent) : Element(str,parent), position(0,false), orientation(0,false), frameOfReference(0,false) {

  Frame *I = new Frame("I",this);
  addFrame(I);

  if(parent) {
    vector<PhysicalVariableProperty*> input;
    input.push_back(new PhysicalVariableProperty(new VecProperty(3),"m",MBSIMNS"position"));
    position.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(new PhysicalVariableProperty(new MatProperty(getEye<string>(3,3,"1","0")),"-",MBSIMNS"orientation"));
    orientation.setProperty(new ExtPhysicalVarProperty(input));

    frameOfReference.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0),this,MBSIMNS"frameOfReference"));
  }
}

Group::~Group() {
//  for(vector<Group*>::iterator i = group.begin(); i != group.end(); ++i) 
//    delete *i;
//  for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
//    delete *i;
//  for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
//    delete *i;
//  // for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i != extraDynamic.end(); ++i)
//  //   delete *i;
//  for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
//    delete *i;
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
}

Group* Group::readXMLFile(const string &filename, Element *parent) {
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename);
  assert(ret==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  map<string,string> dummy;
  incorporateNamespace(doc.FirstChildElement(), dummy);
  Group *group=ObjectFactory::getInstance()->createGroup(e,parent);
  if(group)
    group->initializeUsingXML(e);
  return group;
}

void Group::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Element::initializeUsingXML(element);
  e=element->FirstChildElement();

  if(frameOfReference.getProperty())
    frameOfReference.initializeUsingXML(element);

  // search first element known by Group
  while(e && 
      e->ValueStr()!=MBSIMNS"position" &&
      e->ValueStr()!=MBSIMNS"orientation" &&
      e->ValueStr()!=MBSIMNS"frames")
    e=e->NextSiblingElement();

  if(position.getProperty())
    position.initializeUsingXML(element);

  if(orientation.getProperty())
    orientation.initializeUsingXML(element);

  // frames
  TiXmlElement *E=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
  while(E && E->ValueStr()==MBSIMNS"frame") {
    TiXmlElement *ec=E->FirstChildElement();
    FixedRelativeFrame *f=new FixedRelativeFrame(ec->Attribute("name"),this);
    addFrame(f);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(E);
    E=E->NextSiblingElement();
  }
  while(E) {
    if(E->ValueStr()==PVNS"embed") {
      Frame *f=Frame::readXMLFile(E->Attribute("href"),this);
      if(f) {
        addFrame(f);
        f->initializeUsingXMLEmbed(E);
      }
    }
    else {
      FixedRelativeFrame *f=new FixedRelativeFrame(E->Attribute("name"),this);
      addFrame(f);
      f->initializeUsingXML(E);
    }
    E=E->NextSiblingElement();
  }

  // contours
  E=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
  Contour *c;
  while(E && E->ValueStr()==MBSIMNS"contour") {
    TiXmlElement *ec=E->FirstChildElement();
    c=ObjectFactory::getInstance()->createContour(ec,this);
    if(c) {
      addContour(c);
      c->initializeUsingXML(ec);
    }
    FixedRelativeFrame *f=new FixedRelativeFrame("ContourFrame"+toStr(int(contour.size())),this);
    addFrame(f);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(E);
    c->setSavedFrameOfReference(string("../Frame[")+f->getName()+"]");
    E=E->NextSiblingElement();
  }
  while(E) {
    if(E->ValueStr()==PVNS"embed") {
      c=Contour::readXMLFile(E->Attribute("href"),this);
      if(c) {
        addContour(c);
        c->initializeUsingXMLEmbed(E);
      }
    }
    else {
      c=ObjectFactory::getInstance()->createContour(E,this);
      if(c) {
        addContour(c);
        c->initializeUsingXML(E);
      }
    }
    E=E->NextSiblingElement();
  }

  // groups
  E=element->FirstChildElement(MBSIMNS"groups")->FirstChildElement();
  Group *g;
  while(E) {
    if(E->ValueStr()==PVNS"embed") {
      g=Group::readXMLFile(E->Attribute("href"),this);
      if(g) {
        addGroup(g);
        g->initializeUsingXMLEmbed(E);
      }
    }
    else {
      g=ObjectFactory::getInstance()->createGroup(E,this);
      if(g) {
        addGroup(g);
        g->initializeUsingXML(E);
      }
    }
    E=E->NextSiblingElement();
  }

  // objects
  E=element->FirstChildElement(MBSIMNS"objects")->FirstChildElement();
  Object *o;
  while(E) {
    if(E->ValueStr()==PVNS"embed") {
      o=Object::readXMLFile(E->Attribute("href"),this);
      if(o) {
        addObject(o);
        o->initializeUsingXMLEmbed(E);
      }
    }
    else {
      o=ObjectFactory::getInstance()->createObject(E,this);
      if(o) {
        addObject(o);
        o->initializeUsingXML(E);
      }
    }
    E=E->NextSiblingElement();
  }

  // extraDynamics
  if(element->FirstChildElement(MBSIMNS"extraDynamics")) {
    E=element->FirstChildElement(MBSIMNS"extraDynamics")->FirstChildElement();
    //ExtraDynamic *ed;
    while(E) {
      //        ed=ObjectFactory::getInstance()->createExtraDynamic(E);
      //        addExtraDynamic(ed);
      //        ed->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
  }

  // links
  E=element->FirstChildElement(MBSIMNS"links")->FirstChildElement();
  Link *l;
  while(E) {
    if(E->ValueStr()==PVNS"embed") {
      l=Link::readXMLFile(E->Attribute("href"),this);
      if(l) {
        addLink(l);
        l->initializeUsingXMLEmbed(E);
      }
    }
    else {
      l=ObjectFactory::getInstance()->createLink(E,this);
      if(l) {
        addLink(l);
        l->initializeUsingXML(E);
      }
    }
    E=E->NextSiblingElement();
  }

  // observers
  if(element->FirstChildElement(MBSIMNS"observers")) {
    E=element->FirstChildElement(MBSIMNS"observers")->FirstChildElement();
    Observer *obsrv;
    while(E) {
      if(E->ValueStr()==PVNS"embed") {
        obsrv=Observer::readXMLFile(E->Attribute("href"),this);
        if(obsrv) {
          addObserver(obsrv);
          obsrv->initializeUsingXMLEmbed(E);
        }
      }
      else {
        obsrv=ObjectFactory::getInstance()->createObserver(E,this);
        if(obsrv) {
          addObserver(obsrv);
          obsrv->initializeUsingXML(E);
        }
      }
      E=E->NextSiblingElement();
    }
  }

  e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameI");
  if(e)
    getFrame(0)->initializeUsingXML2(e);
  else
    getFrame(0)->setOpenMBVFrame(false);
}

TiXmlElement* Group::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Element::writeXMLFile(parent);

  TiXmlElement *ele1;

  if(position.getProperty()) {
    frameOfReference.writeXMLFile(ele0);
    position.writeXMLFile(ele0);
    orientation.writeXMLFile(ele0);
  }

  ele1 = new TiXmlElement( MBSIMNS"frames" );
  for(int i=1; i<frame.size(); i++)
    if(frame[i]->isEmbedded())
      frame[i]->writeXMLFileEmbed(ele1);
    else
      frame[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  for(int i=0; i<contour.size(); i++)
    if(contour[i]->isEmbedded())
      contour[i]->writeXMLFileEmbed(ele1);
    else
      contour[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"groups" );
  for(int i=0; i<group.size(); i++)
    if(group[i]->isEmbedded())
      group[i]->writeXMLFileEmbed(ele1);
    else
      group[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"objects" );
  for(int i=0; i<object.size(); i++)
    if(object[i]->isEmbedded())
      object[i]->writeXMLFileEmbed(ele1);
    else
      object[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"extraDynamics" );
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"links" );
  for(int i=0; i<link.size(); i++)
    if(link[i]->isEmbedded())
      link[i]->writeXMLFileEmbed(ele1);
    else
      link[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"observers" );
  for(int i=0; i<observer.size(); i++)
    if(observer[i]->isEmbedded())
      observer[i]->writeXMLFileEmbed(ele1);
    else
      observer[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  Frame *I = getFrame(0);
  if(I->openMBVFrame()) {
    ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameI" );
    I->writeXMLFile2(ele1);
    ele0->LinkEndChild(ele1);
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
      if (container=="Object")
        return getObject(searched_name)->getByPathSearch(rest);
      else if (container=="Link")
        return getLink(searched_name)->getByPathSearch(rest);
      //else if (container=="ExtraDynamic")
      //return getExtraDynamic(searched_name)->getByPathSearch(rest);
      else if (container=="Group")
        return getGroup(searched_name)->getByPathSearch(rest);
      else {
        cout << "Unknown name of container" << endl;
        throw;
      }
    }
    else {
      if (container=="Object")
        return getObject(searched_name);
      else if (container=="Link")
        return getLink(searched_name);
      //else if (container=="ExtraDynamic")
      //return getExtraDynamic(searched_name);
      else if (container=="Group")
        return getGroup(searched_name);
      else if (container=="Frame")
        return getFrame(searched_name);
      else if (container=="Contour")
        return getContour(searched_name);
      else {
        cout << "Unknown name of container" << endl;
        throw;
      }
    }
  }
}

Link* Group::getLink(const string &name, bool check) {
  int i;
  for(i=0; i<link.size(); i++) {
    if(link[i]->getName() == name)
      return link[i];
  }
  if(check) {
    if(!(i<link.size()))
      throw MBSimError("The link \""+link[i]->getName()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerLink()->childCount());
  }
  return NULL;
}

Observer* Group::getObserver(const string &name, bool check) {
  int i;
  for(i=0; i<observer.size(); i++) {
    if(observer[i]->getName() == name)
      return observer[i];
  }
  if(check) {
    if(!(i<observer.size()))
      throw MBSimError("The observer \""+observer[i]->getName()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerObserver()->childCount());
  }
  return NULL;
}

Group* Group::getGroup(const string &name, bool check) {
  int i;
  for(i=0; i<group.size(); i++) {
    if(group[i]->getName() == name)
      return group[i];
  }
  if(check) {
    if(!(i<group.size()))
      throw MBSimError("The group \""+group[i]->getName()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerGroup()->childCount());
  }
  return NULL;
}

Object* Group::getObject(const string &name, bool check) {
  int i;
  for(i=0; i<object.size(); i++) {
    if(object[i]->getName() == name)
      return object[i];
  }
  if(check) {
    if(!(i<object.size()))
      throw MBSimError("The object \""+object[i]->getName()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerObject()->childCount());
  }
  return NULL;
}

Frame* Group::getFrame(const string &name, bool check) {
  int i;
  for(i=0; i<frame.size(); i++) {
    if(frame[i]->getName() == name)
      return frame[i];
  }
  if(check) {
    if(!(i<frame.size()))
      throw MBSimError("The frames \""+frame[i]->getName()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerFrame()->childCount());
  }
  return NULL;
}

Contour* Group::getContour(const string &name, bool check) {
  int i;
  for(i=0; i<contour.size(); i++) {
    if(contour[i]->getName() == name)
      return contour[i];
  }
  if(check) {
    if(!(i<contour.size()))
      throw MBSimError("The contours \""+contour[i]->getName()+"\" comprises no contour \""+name+"\"!");
    assert(i<getContainerContour()->childCount());
  }
  return NULL;
}

