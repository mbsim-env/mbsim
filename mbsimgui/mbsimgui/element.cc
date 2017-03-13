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
#include <QtGui/QFileDialog>
#include <cmath>
#include "frame.h"
#include "contour.h"
#include "dynamic_system_solver.h"
#include "object.h"
#include "link.h"
#include "constraint.h"
#include "observer.h"
#include "mainwindow.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;
  extern bool absolutePath;
  extern QDir mbsDir;

  int Element::IDcounter=0;

  Element::Element(const string &name__, Element *parent_, const vector<FQN> &plotFeatureTypes) : parent(parent_), embed(0,false), element(NULL), name_(name__) {
    name.setProperty(new TextProperty(name_,""));
    embed.setProperty(new EmbedProperty(name_));
    plotFeature.setProperty(new PlotFeatureStatusProperty(plotFeatureTypes));
    ID=toStr(IDcounter++);
    addPlotFeature("plotRecursive");
    addPlotFeature("separateFilePerGroup");
    addPlotFeature("openMBV");
    addPlotFeature("debug");
  }

  string Element::getPath() {
    return parent?(parent->getPath()+"."+getName()):getName();
  }

  std::string Element::getValue() const {
    return isEmbedded()?("counterName="+static_cast<const EmbedProperty*>(embed.getProperty())->getCounterName()+"; count="+static_cast<const EmbedProperty*>(embed.getProperty())->getCount()):"";
  }

  void Element::writeXMLFile(const string &name) {
    shared_ptr<DOMDocument> doc=mw->parser->createDocument();
    writeXMLFile(doc.get());
    QFileInfo info(QString::fromStdString(name));
    QDir dir;
    if(!dir.exists(info.absolutePath()))
      dir.mkpath(info.absolutePath());
    DOMParser::serialize(doc.get(), (name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
  }

  void Element::writeXMLFileEmbed(const string &name) {
    shared_ptr<DOMDocument> doc=mw->parser->createDocument();
    Embed<Element>::writeXML(this,doc.get());
    QFileInfo info(QString::fromStdString(name));
    QDir dir;
    if(!dir.exists(info.absolutePath()))
      dir.mkpath(info.absolutePath());
    DOMParser::serialize(doc.get(), (name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
  }

  DOMElement* Element::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    element=D(doc)->createElement(getNameSpace()%getType());
    E(element)->setAttribute("name", static_cast<TextProperty*>(name.getProperty())->getText());
    parent->insertBefore(element, NULL);
    return element;
  }

  DOMElement* Element::initializeUsingXML(DOMElement *element) {
    plotFeature.initializeUsingXML(element);
    this->element = element;
    return element;
  }

  DOMElement* Element::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType());
    E(ele0)->setAttribute("name", getName());
    plotFeature.writeXMLFile(ele0);
    parent->insertBefore(ele0, NULL);
    return ele0;
  }

  void Element::initializeUsingXMLEmbed(DOMElement *element) {
    embed.initializeUsingXML(element);
    embed.setActive(true);
  }

  DOMElement* Element::writeXMLFileEmbed(DOMNode *parent) {
    parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    //DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = embed.writeXMLFile(parent);

    if(not(absolutePath) and static_cast<const EmbedProperty*>(embed.getProperty())->hasParameterFile()) {
      string absFileName =  static_cast<const EmbedProperty*>(embed.getProperty())->getParameterFile();
      string relFileName =  mbsDir.relativeFilePath(QString::fromStdString(absFileName)).toStdString();
      string name=absolutePath?(mw->getUniqueTempDir().generic_string()+"/"+relFileName):absFileName;
      parameters.writeXMLFile(name);
    }
    else
      parameters.writeXMLFile(ele);

    if(not(absolutePath) and static_cast<const EmbedProperty*>(embed.getProperty())->hasFile()) {
      string absFileName =  static_cast<const EmbedProperty*>(embed.getProperty())->getFile();
      string relFileName =  mbsDir.relativeFilePath(QString::fromStdString(absFileName)).toStdString();
      string name=absolutePath?(mw->getUniqueTempDir().generic_string()+"/"+relFileName):absFileName;
      writeXMLFile(name);
    }
    else 
      writeXMLFile(ele);

    return ele;
  }

  string Element::getXMLPath(Element *ref, bool rel) {
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
      string type;
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
      string str = type + "[" + getName() + "]";
      for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
        if(dynamic_cast<Group*>(*i1))
          str = string("Group[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(*i1))
          str = string("Object[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Link*>(*i1))
          str = string("Link[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Constraint*>(*i1))
          str = string("Constraint[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Observer*>(*i1))
          str = string("Observer[") + (*i1)->getName() + "]/" + str;
        else
          str = "";
      }
      for(int i=0; i<int(e0.size())-imatch; i++)
        str = "../" + str;
      return str;
    } else {
      string type;
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
      string str = type + "[" + getName() + "]";
      Element* element = parent;
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        if(dynamic_cast<Group*>(element))
          str = string("Group[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(element))
          str = string("Object[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Link*>(element))
          str = string("Link[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Constraint*>(element))
          str = string("Constraint[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Observer*>(element))
          str = string("Observer[") + element->getName() + "]/" + str;
        else
          str = "";
        element = element->getParent();
      }
      str = "/" + str;
      return str;
    }
  }

  string Element::getCounterName() const {
    const EmbedProperty *e = static_cast<const EmbedProperty*>(embed.getProperty());
    if(e->hasCounter())
      return e->getCounterName();
    return "";
  }

  vector<Element*> Element::getParents() {
    vector<Element*> parents;
    if(getParent()) {
      parents = getParent()->getParents();
      parents.push_back(getParent());
    }
    return parents;
  }

  void Element::addPlotFeature(const string &pf) {
    for(unsigned int i=0; i<plotFeatures.size(); i++)
      if(plotFeatures[i]==pf)
        return;
    plotFeatures.push_back(pf);
    if(getParent())
      getParent()->addPlotFeature(pf);
  }

//  string Element::getName() const {
//    return name_;
//    return E(element)->getAttribute("name");
//    return static_cast<const TextProperty*>(name.getProperty())->getText();
//  }

  void Element::setName(const string &str) {
    name_ = str;
//    static_cast<TextProperty*>(name.getProperty())->setText(str);
    E(element)->setAttribute("name", str);
  }

}
