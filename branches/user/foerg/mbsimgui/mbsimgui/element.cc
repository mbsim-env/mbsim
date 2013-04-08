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
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QInputDialog>
#include <QtGui/QMessageBox>
#include <cmath>
#include "property_widget.h"
#include "basic_widgets.h"
#include "solver.h"
#include "object.h"
#include "frame.h"
#include "contour.h"
#include "link.h"
#include "observer.h"
#include "mainwindow.h"

using namespace std;

extern MainWindow *mw;

int Element::IDcounter=0;
map<string, Element*> Element::idEleMap;

Element::Element(const string &name_, Element *parent_) : name(name_), parent(parent_), drawThisPath(true), searchMatched(true), ns(MBSIMNS) {
  stringstream sstr;
  sstr<<IDcounter++;
  ID=sstr.str();
  idEleMap.insert(make_pair(ID, this));
}

Element::~Element() {
  idEleMap.erase(ID);
}

string Element::getPath() {
 return parent?(parent->getPath()+"."+getName()):getName();
}

void Element::writeXMLFile(const string &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile((name+".xml"));
}

void Element::initialize() {
}

void Element::initializeUsingXML(TiXmlElement *element) {
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->initializeUsingXML(element);
}

TiXmlElement* Element::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(ns+getType());
  //name->writeXMLFile(ele0);
  ele0->SetAttribute("name", getName());
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return ele0;
}

string Element::getXMLPath(Element *ref, bool rel) {
  if(rel) {
    vector<Element*> e0, e1;
    Element* element = ref;
    e0.push_back(element);
    while(!dynamic_cast<Solver*>(element)) {
      element = element->getParent();
      e0.push_back(element);
    }
    element = parent;
    e1.push_back(element);
    while(!dynamic_cast<Solver*>(element)) {
      element = element->getParent();
      e1.push_back(element);
    }
    int imatch=0;
    for(vector<Element*>::iterator i0 = e0.end()-1, i1 = e1.end()-1 ; (i0 != e0.begin()-1) && (i1 != e1.begin()-1) ; i0--, i1--) 
      if(*i0 == *i1) imatch++;
    string type;
    if(dynamic_cast<Group*>(this))
      type = "Group";
    else if(dynamic_cast<Object*>(this))
      type = "Object";
    else if(dynamic_cast<Contour*>(this))
      type = "Contour";
    else if(dynamic_cast<Frame*>(this))
      type = "Frame";
    else 
      type = getType();
    string str = type + "[" + getName() + "]";
    for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
      if(dynamic_cast<Group*>(*i1))
        str = string("Group[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Object*>(*i1))
        str = string("Object[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Frame*>(*i1))
        str = string("Frame[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Contour*>(*i1))
        str = string("Contour[") + (*i1)->getName() + "]/" + str;
      else
        throw;
    }
    for(int i=0; i<int(e0.size())-imatch; i++)
      str = "../" + str;
    return str;
  } else {
    string type;
    if(dynamic_cast<Group*>(this))
      type = "Group";
    else if(dynamic_cast<Object*>(this))
      type = "Object";
    else if(dynamic_cast<Frame*>(this))
      type = "Frame";
    else if(dynamic_cast<Contour*>(this))
      type = "Contour";
    else 
      type = getType();
    string str = type + "[" + getName() + "]";
    Element* element = parent;
    while(!dynamic_cast<Solver*>(element)) {
      if(dynamic_cast<Group*>(element))
        str = string("Group[") + element->getName() + "]/" + str;
      else if(dynamic_cast<Object*>(element))
        str = string("Object[") + element->getName() + "]/" + str;
      else if(dynamic_cast<Contour*>(element))
        str = string("Contour[") + element->getName() + "]/" + str;
      else
        throw;
      element = element->getParent();
    }
    str = "/" + str;
    return str;
  }
}

//Element* Element::getChild(TreeItem* container, const string &name, bool check) {
//  int i;
//  for(i=0; i<container->childCount(); i++) {
//    if(((Element*)container->child(i))->getName() == name)
//      return (Element*)container->child(i);
//  }
//  if(check) {
//    if(!(i<container->childCount()))
//      throw MBSimError("The object \""+((Element*)container->child(i))->getName()+"\" comprises no frame \""+name+"\"!");
//    assert(i<container->childCount());
//  }
//  return NULL;
//}

//Element* Container::getChild(int i) {
//  return (Element*)child(i);
//}

//Element* Container::getChild(const string &name, bool check) {
//  int i;
//  for(i=0; i<childCount(); i++) {
//    if(getChild(i)->getName() == name)
//      return (Element*)child(i);
//  }
//  if(check) {
//    if(!(i<childCount()))
//      throw MBSimError("The object \""+getChild(i)->getName()+"\" comprises no element \""+name+"\"!");
//    assert(i<childCount());
//  }
//  return NULL;
//}


//Frame* Element::getFrame(int i) {
//  return (Frame*)frames->child(i); 
//}
//
//Contour* Element::getContour(int i) {
//  return (Contour*)contours->child(i); 
//}
//
//Object* Element::getObject(int i) {
//  return (Object*)objects->child(i); 
//}
//
//Link* Element::getLink(int i) {
//  return (Link*)links->child(i); 
//}
//
//Observer* Element::getObserver(int i) {
//  return (Observer*)observers->child(i); 
//}
//
//Group* Element::getGroup(int i) {
//  return (Group*)groups->child(i); 
//}


