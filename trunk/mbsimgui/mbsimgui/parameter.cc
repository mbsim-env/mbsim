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
#include "parameter.h"
#include "editors.h"
#include "element.h"
#include "objectfactory.h"
#include "octaveutils.h"
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>

using namespace std;

Parameter::Parameter(const QString &str, QTreeWidgetItem *parentItem, int ind) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true) {
  if(ind==-1 || ind>=parentItem->childCount())
    parentItem->addChild(this); // insert as last element
  else
    parentItem->insertChild(ind, this); // insert at position ind

  setText(0, str);

  properties=new PropertyDialog(this);
  properties->addTab("General");

  name = new ParameterNameWidget(this);
  ExtXMLWidget *widget = new ExtXMLWidget("Name","",name);
  properties->addToTab("General", widget);

  contextMenu=new QMenu("Context Menu");

  connect(this,SIGNAL(parameterChanged(const QString&)),this,SLOT(updateTreeWidgetItem(const QString&)));

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);
}

Parameter::~Parameter() {
  delete properties;
}

QString Parameter::getInfo() {
  return "";
}

void Parameter::saveAs() {
  QString file=QFileDialog::getSaveFileName(0, "OpenMBV Files",  QString("./")+getType()+".xml", "hdf5 Files (*.xml)");
  if(file!="") {
    if(file.contains(".xml"))
      file.chop(10);
    writeXMLFile(file.toAscii().data());
  }
}

void Parameter::initializeUsingXML(TiXmlElement *element) {
}

TiXmlElement* Parameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+getType().toStdString());
  name->writeXMLFile(ele0);
  TiXmlText* text= new TiXmlText(getValue());
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);
  return ele0;
}

Parameter* Parameter::readXMLFile(const QString &filename, QTreeWidgetItem* parent) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  assert(doc.LoadFile(filename.toAscii().data())==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);
  Parameter *parameter=ObjectFactory::getInstance()->createParameter(e, parent, 1);
  parameter->initializeUsingXML(doc.FirstChildElement());
  return parameter;
}

void Parameter::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  map<string, string> nsprefix;
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".mbsimparam.xml").toAscii().data());
}

void Parameter::updateTreeWidgetItem(const QString &str) {
  setText(1, str);
}

DoubleParameter::DoubleParameter(const QString &str, QTreeWidgetItem *parentItem, int ind) : Parameter(str,parentItem,ind) {

  value = new ParameterValueWidget(new PhysicalStringWidget(new SScalarWidget("1"),PVNS"scalarParameter",QStringList(),0));
  ExtXMLWidget *widget = new ExtXMLWidget("Value","",value);
  properties->addToTab("General", widget);
  connect(value,SIGNAL(parameterChanged(const QString&)),this,SIGNAL(parameterChanged(const QString&)));

  properties->addStretch();
}

void DoubleParameter::initializeUsingXML(TiXmlElement *element) {
  ParameterValueWidget *val = (ParameterValueWidget*)value;
  val->getExtPhysicalWidget()->initializeUsingXML(element);
  TiXmlText *text = dynamic_cast<TiXmlText*>(element->FirstChild());
    if(text) {
      val->getExtPhysicalWidget()->getPhysicalStringWidget(0)->setValue(text->Value());
      val->getExtPhysicalWidget()->getPhysicalStringWidget(1)->setValue(text->Value());
    } 
    else if(element) {
      val->getExtPhysicalWidget()->getPhysicalStringWidget(0)->initializeUsingXML(element);
      val->getExtPhysicalWidget()->getPhysicalStringWidget(1)->setValue(val->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getValue().c_str());
    } 
    emit parameterChanged(getValue().c_str());
}

void Parameter::remove() {
  QTreeWidget *tree = treeWidget();
  tree->invisibleRootItem()->removeChild(this);
  emit parameterChanged(getValue().c_str());
}
