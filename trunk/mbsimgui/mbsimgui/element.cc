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
#include "mainwindow.h"

extern MainWindow *mw;
extern bool absolutePath;

using namespace std;

int Element::IDcounter=0;

Element::Element(const string &name_, Element *parent_) : parent(parent_), embed(0,false) {
  name.setProperty(new TextProperty(name_,""));
  embed.setProperty(new EmbedProperty(this));
  ID=toStr(IDcounter++);
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
  TiXmlElement *ele0=new TiXmlElement(getNameSpace()+getType());
  //name->writeXMLFile(ele0);
  ele0->SetAttribute("name", getName());
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return ele0;
}

void Element::initializeUsingXMLEmbed(TiXmlElement *element) {
  embed.setActive(true);
  embed.initializeUsingXML(element);
}

TiXmlElement* Element::writeXMLFileEmbed(TiXmlNode *parent) {
  writeXMLFile(absolutePath?(mw->getUniqueTempDir().toStdString()+"/"+static_cast<const EmbedProperty*>(embed.getProperty())->getFile()):(static_cast<const EmbedProperty*>(embed.getProperty())->getFile()));
  QFile::copy(QString::fromStdString(static_cast<const EmbedProperty*>(embed.getProperty())->getParameterFile()),mw->getUniqueTempDir()+"/"+QString::fromStdString(static_cast<const EmbedProperty*>(embed.getProperty())->getParameterFile()));
  return embed.writeXMLFile(parent);
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

ParameterList Element::getParameterList(bool addCounter) const {
  ParameterList list;
  const EmbedProperty *e = static_cast<const EmbedProperty*>(embed.getProperty());
  if(isEmbedded() && e->hasParameterFile())
    list.readXMLFile(e->getParameterFile());
  if(parent)
    list.addParameterList(parent->getParameterList(false));
  if(addCounter && e->hasCounter())
    list.addParameter(e->getCounterName(),"1"); 
  return list;
}
