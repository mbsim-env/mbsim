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
#include "solver.h"
#include "object.h"
#include "frame.h"
#include "contour.h"
#include "link.h"
#include "observer.h"

using namespace std;

int Element::IDcounter=0;

Element::Element(const string &name_, Element *parent_) : parent(parent_), href(0,false), count(0,false), counterName(0,false), parameterList(0,false) {
  name.setProperty(new TextProperty(name_,""));
  href.setProperty(new FileProperty(""));
  static_cast<FileProperty*>(href.getProperty())->setFileName(name_+".xml");
  static_cast<FileProperty*>(href.getProperty())->setAbsoluteFilePath(name_+".xml");
  count.setProperty(new TextProperty("1",""));
  counterName.setProperty(new TextProperty("n",""));
  parameterList.setProperty(new FileProperty(""));
  stringstream sstr;
  sstr<<IDcounter++;
  ID=sstr.str();
}

Element::~Element() {
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
  doc.SaveFile((name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
}

void Element::initialize() {
}

void Element::initializeUsingXML(TiXmlElement *element) {
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->initializeUsingXML(element);
}

TiXmlElement* Element::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  //name->writeXMLFile(ele0);
  ele0->SetAttribute("name", getName());
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return ele0;
}

void Element::initializeUsingXMLEmbed(TiXmlElement *element) {
  href.setActive(true);
  string file = element->Attribute("href");
  static_cast<FileProperty*>(href.getProperty())->setFileName(file);
  static_cast<FileProperty*>(href.getProperty())->setAbsoluteFilePath(file);
  if(element->Attribute("count")) {
    count.setActive(true);
    static_cast<TextProperty*>(count.getProperty())->setText(element->Attribute("count"));
  }
  if(element->Attribute("counterName")) {
    counterName.setActive(true);
    static_cast<TextProperty*>(counterName.getProperty())->setText(element->Attribute("counterName"));
  }
  TiXmlElement *ele = element->FirstChildElement(PVNS+string("localParameter"));
  if(ele) {
    parameterList.setActive(true);
    string file = ele->Attribute("href");
    static_cast<FileProperty*>(parameterList.getProperty())->setFileName(file);
    static_cast<FileProperty*>(parameterList.getProperty())->setAbsoluteFilePath(file);
  }
}

TiXmlElement* Element::writeXMLFileEmbed(TiXmlNode *parent) {
  writeXMLFile(static_cast<FileProperty*>(href.getProperty())->getAbsoluteFilePath());
  TiXmlElement *ele0=new TiXmlElement(PVNS+string("embed"));
  ele0->SetAttribute("href", static_cast<FileProperty*>(href.getProperty())->getAbsoluteFilePath());
  if(count.isActive())
    ele0->SetAttribute("count", static_cast<TextProperty*>(count.getProperty())->getText());
  //if(static_cast<TextProperty*>(counterName.getProperty())->getText() != "")
  if(counterName.isActive())
    ele0->SetAttribute("counterName", static_cast<TextProperty*>(counterName.getProperty())->getText());
  if(parameterList.isActive()) {
    TiXmlElement *ele1=new TiXmlElement(PVNS+string("localParameter"));
    ele1->SetAttribute("href", static_cast<FileProperty*>(parameterList.getProperty())->getAbsoluteFilePath());
    ele0->LinkEndChild(ele1);
  }
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
