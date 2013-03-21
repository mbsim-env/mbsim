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

Element::Element(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true), frames(0), contours(0), groups(0), objects(0), links(0), extraDynamics(0), observers(0), ns(MBSIMNS), dialog(0) {
  stringstream sstr;
  sstr<<IDcounter++;
  ID=sstr.str();
  idEleMap.insert(make_pair(ID, this));

  if(parentItem) {
  if(ind==-1 || ind>=parentItem->childCount())
    parentItem->addChild(this); // insert as last element
  else
    parentItem->insertChild(ind, this); // insert at position ind
  parentElement = static_cast<Element*>(parentItem->parent());
  }

  setName(str);

  if(grey) {
    QColor color;
    color.setRgb(200,200,200);
    QBrush brush(color);
    //setForeground(0,brush);
    //setForeground(1,brush);
  }
  else {
  }

  contextMenu=new QMenu("Context Menu");

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Properties", this);
  connect(action,SIGNAL(triggered()),this,SLOT(openPropertyDialog()));
  contextMenu->addAction(action);

}

Element::~Element() {
  idEleMap.erase(ID);
  // delete scene graph
  delete dialog;
  //objects.erase(this);
}

void Element::openPropertyDialog() {
  if(!dialog) {
    dialog = new PropertyDialog;
    connect(dialog,SIGNAL(apply()),this,SLOT(updateElement()));
    initializeDialog();
  }
  toWidget();
  dialog->show();
  //if (dialog->exec())
  //  updateElement();
}

void Element::initializeDialog() {
  dialog->addTab("General");
  textWidget = new TextWidget;
  ExtWidget *name=new ExtWidget("Name",textWidget);
  dialog->addToTab("General",name);
}

void Element::toWidget() {
  textWidget->setName(getName());
}

void Element::fromWidget() {
  setName(textWidget->getName());
}

void Element::updateElement() {
  fromWidget();
  mw->mbsimxml(1);
}

void Element::setName(const QString &str) {
  setText(0,str);
  name = str;
}

QString Element::getPath() {
 return parentElement?(parentElement->getPath()+"."+text(0)):text(0);
}

QString Element::getInfo() {
  return QString("<b>Path:</b> %1<br/>").arg(getPath())+
         QString("<b>Class:</b> <img src=\"%1\" width=\"16\" height=\"16\"/> %2").arg(iconFile).arg(metaObject()->className());
}

void Element::updateTextColor() {
  QPalette palette;
  if(drawThisPath) // active
    if(searchMatched)
      setForeground(0, palette.brush(QPalette::Active, QPalette::Text));
    else
      setForeground(0, QBrush(QColor(255,0,0)));
  else // inactive
    if(searchMatched)
      setForeground(0, palette.brush(QPalette::Disabled, QPalette::Text));
    else
      setForeground(0, QBrush(QColor(128,0,0)));
}

void Element::saveAs() {
  file=QFileDialog::getSaveFileName(0, "XML model files",  QString("./")+getName()+getFileExtension(), QString("hdf5 Files (*)")+getFileExtension()+")");
  if(file!="") {
    if(file.contains(getFileExtension()))
      file.chop(getFileExtension().size());
    writeXMLFile(file);
    actionSave->setDisabled(false);
  }
}

void Element::save() {
  if(file.contains(getFileExtension()))
    file.chop(getFileExtension().size());
  writeXMLFile(file);
}

void Element::copy() {
  delete copiedElement;
  copiedElement = new TiXmlElement("Node");
  writeXMLFile(copiedElement);
  map<string, string> nsprefix;
  unIncorporateNamespace(copiedElement->FirstChildElement(), nsprefix);  
  ((Solver*)treeWidget()->topLevelItem(0))->setActionPasteDisabled(false);
}

void Element::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  map<string, string> nsprefix;
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".xml").toAscii().data());
}

void Element::updateWidget() {
  if(dialog && dialog->isVisible())
    dialog->updateWidget();
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      getGroup(i)->updateWidget();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      getObject(i)->updateWidget();
  }
  if(getContainerFrame()) {
    for(int i=0; i<getContainerFrame()->childCount(); i++)
      getFrame(i)->updateWidget();
  }
  if(getContainerContour()) {
    for(int i=0; i<getContainerContour()->childCount(); i++)
      getContour(i)->updateWidget();
  }
  if(getContainerLink()) {
    for(int i=0; i<getContainerLink()->childCount(); i++)
      getLink(i)->updateWidget();
  }
  if(getContainerObserver()) {
    for(int i=0; i<getContainerObserver()->childCount(); i++)
      getObserver(i)->updateWidget();
  }
}

void Element::initialize() {
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      getGroup(i)->initialize();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      getObject(i)->initialize();
  }
  if(getContainerFrame()) {
    for(int i=0; i<getContainerFrame()->childCount(); i++)
      getFrame(i)->initialize();
  }
  if(getContainerContour()) {
    for(int i=0; i<getContainerContour()->childCount(); i++)
      getContour(i)->initialize();
  }
  if(getContainerLink()) {
    for(int i=0; i<getContainerLink()->childCount(); i++)
      getLink(i)->initialize();
  }
  if(getContainerObserver()) {
    for(int i=0; i<getContainerObserver()->childCount(); i++)
      getObserver(i)->initialize();
  }
}

void Element::resizeVariables() {
  dialog->resizeVariables();
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      getGroup(i)->resizeVariables();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      getObject(i)->resizeVariables();
  }
  if(getContainerFrame()) {
    for(int i=0; i<getContainerFrame()->childCount(); i++)
      getFrame(i)->resizeVariables();
  }
  if(getContainerContour()) {
    for(int i=0; i<getContainerContour()->childCount(); i++)
      getContour(i)->resizeVariables();
  }
  if(getContainerLink()) {
    for(int i=0; i<getContainerLink()->childCount(); i++)
      getLink(i)->resizeVariables();
  }
  if(getContainerObserver()) {
    for(int i=0; i<getContainerObserver()->childCount(); i++)
      getObserver(i)->resizeVariables();
  }
}

void Element::initializeUsingXML(TiXmlElement *element) {
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->initializeUsingXML(element);
}

TiXmlElement* Element::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(ns+getType().toStdString());
  //name->writeXMLFile(ele0);
  ele0->SetAttribute("name", getName().toStdString());
//  for(unsigned int i=0; i<plotFeature.size(); i++)
//    plotFeature[i]->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return ele0;
}

QString Element::getXMLPath(Element *ref, bool rel) {
  if(rel) {
    vector<Element*> e0, e1;
    Element* element = ref;
    e0.push_back(element);
    while(!dynamic_cast<Solver*>(element)) {
      element = element->getParentElement();
      e0.push_back(element);
    }
    element = parentElement;
    e1.push_back(element);
    while(!dynamic_cast<Solver*>(element)) {
      element = element->getParentElement();
      e1.push_back(element);
    }
    int imatch=0;
    for(vector<Element*>::iterator i0 = e0.end()-1, i1 = e1.end()-1 ; (i0 != e0.begin()-1) && (i1 != e1.begin()-1) ; i0--, i1--) 
      if(*i0 == *i1) imatch++;
    QString type;
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
    QString str = type + "[" + getName() + "]";
    for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
      if(dynamic_cast<Group*>(*i1))
        str = QString("Group[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Object*>(*i1))
        str = QString("Object[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Frame*>(*i1))
        str = QString("Frame[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Contour*>(*i1))
        str = QString("Contour[") + (*i1)->getName() + "]/" + str;
      else
        throw;
    }
    for(int i=0; i<int(e0.size())-imatch; i++)
      str = "../" + str;
    return str;
  } else {
    QString type;
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
    QString str = type + "[" + getName() + "]";
    Element* element = parentElement;
    while(!dynamic_cast<Solver*>(element)) {
      if(dynamic_cast<Group*>(element))
        str = QString("Group[") + element->getName() + "]/" + str;
      else if(dynamic_cast<Object*>(element))
        str = QString("Object[") + element->getName() + "]/" + str;
      else if(dynamic_cast<Contour*>(element))
        str = QString("Contour[") + element->getName() + "]/" + str;
      else
        throw;
      element = element->getParentElement();
    }
    str = "/" + str;
    return str;
  }
}

Element* Element::getChild(QTreeWidgetItem* container, const QString &name, bool check) {
  int i;
  for(i=0; i<container->childCount(); i++) {
    if(((Element*)container->child(i))->getName() == name)
      return (Element*)container->child(i);
  }
  if(check) {
    if(!(i<container->childCount()))
      throw MBSimError("The object \""+((Element*)container->child(i))->getName().toStdString()+"\" comprises no frame \""+name.toStdString()+"\"!");
    assert(i<container->childCount());
  }
  return NULL;
}

QString Element::newName(QTreeWidgetItem* container, const QString &type) {
  bool askForName = false;
  QString str;
  for(int i=1; i<10000; i++) {
    str = type + QString::number(i);
    if(!getChild(container,str,false))
      break;
  }
  QString text=str;
  if(askForName) {
    do {
      text = QInputDialog::getText(0, tr("Add"), tr("Name:"), QLineEdit::Normal, str);

      if(getChild(container,text,false)) {
        QMessageBox msgBox;
        msgBox.setText(QString("The name ") + text + " does already exist.");
        msgBox.exec();
      } else
        break;
    } while(true);
  }
  return text;
}

Element* Container::getChild(int i) {
  return (Element*)child(i);
}

Element* Container::getChild(const QString &name, bool check) {
  int i;
  for(i=0; i<childCount(); i++) {
    if(getChild(i)->getName() == name)
      return (Element*)child(i);
  }
  if(check) {
    if(!(i<childCount()))
      throw MBSimError("The object \""+getChild(i)->getName().toStdString()+"\" comprises no element \""+name.toStdString()+"\"!");
    assert(i<childCount());
  }
  return NULL;
}

void Element::remove() {

  QTreeWidget *tree = treeWidget();
  QTreeWidgetItem::parent()->removeChild(this);
  ((Element*)tree->topLevelItem(0))->updateWidget();
}


Link* Element::getLink(const QString &name, bool check) {
  int i;
  for(i=0; i<getContainerLink()->childCount(); i++) {
    if(getLink(i)->getName() == name)
      return getLink(i);
  }
  if(check) {
    if(!(i<getContainerLink()->childCount()))
      throw MBSimError("The link \""+getLink(i)->getName().toStdString()+"\" comprises no frame \""+name.toStdString()+"\"!");
    assert(i<getContainerLink()->childCount());
  }
  return NULL;
}

Observer* Element::getObserver(const QString &name, bool check) {
  int i;
  for(i=0; i<getContainerObserver()->childCount(); i++) {
    if(getObserver(i)->getName() == name)
      return getObserver(i);
  }
  if(check) {
    if(!(i<getContainerObserver()->childCount()))
      throw MBSimError("The observer \""+getObserver(i)->getName().toStdString()+"\" comprises no frame \""+name.toStdString()+"\"!");
    assert(i<getContainerObserver()->childCount());
  }
  return NULL;
}

Group* Element::getGroup(const QString &name, bool check) {
  int i;
  for(i=0; i<getContainerGroup()->childCount(); i++) {
    if(getGroup(i)->getName() == name)
      return getGroup(i);
  }
  if(check) {
    if(!(i<getContainerGroup()->childCount()))
      throw MBSimError("The group \""+getGroup(i)->getName().toStdString()+"\" comprises no frame \""+name.toStdString()+"\"!");
    assert(i<getContainerGroup()->childCount());
  }
  return NULL;
}

Object* Element::getObject(const QString &name, bool check) {
  int i;
  for(i=0; i<getContainerObject()->childCount(); i++) {
    if(getObject(i)->getName() == name)
      return getObject(i);
  }
  if(check) {
    if(!(i<getContainerObject()->childCount()))
      throw MBSimError("The object \""+getObject(i)->getName().toStdString()+"\" comprises no frame \""+name.toStdString()+"\"!");
    assert(i<getContainerObject()->childCount());
  }
  return NULL;
}

Frame* Element::getFrame(const QString &name, bool check) {
  int i;
  for(i=0; i<getContainerFrame()->childCount(); i++) {
    if(getFrame(i)->getName() == name)
      return getFrame(i);
  }
  if(check) {
    if(!(i<getContainerFrame()->childCount()))
      throw MBSimError("The frames \""+getFrame(i)->getName().toStdString()+"\" comprises no frame \""+name.toStdString()+"\"!");
    assert(i<getContainerFrame()->childCount());
  }
  return NULL;
}

Contour* Element::getContour(const QString &name, bool check) {
  int i;
  for(i=0; i<getContainerContour()->childCount(); i++) {
    if(getContour(i)->getName() == name)
      return getContour(i);
  }
  if(check) {
    if(!(i<getContainerContour()->childCount()))
      throw MBSimError("The contours \""+getContour(i)->getName().toStdString()+"\" comprises no contour \""+name.toStdString()+"\"!");
    assert(i<getContainerContour()->childCount());
  }
  return NULL;
}

Frame* Element::getFrame(int i) {
  return (Frame*)frames->child(i); 
}

Contour* Element::getContour(int i) {
  return (Contour*)contours->child(i); 
}

Object* Element::getObject(int i) {
  return (Object*)objects->child(i); 
}

Link* Element::getLink(int i) {
  return (Link*)links->child(i); 
}

Observer* Element::getObserver(int i) {
  return (Observer*)observers->child(i); 
}

Group* Element::getGroup(int i) {
  return (Group*)groups->child(i); 
}


