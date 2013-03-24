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
#include "property_widget.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "objectfactory.h"
#include "octaveutils.h"
#include "mainwindow.h"
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>

using namespace std;

extern MainWindow *mw;

Parameter::Parameter(const QString &str, QTreeWidgetItem *parentItem, int ind) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true), dialog(0) {
  if(ind==-1 || ind>=parentItem->childCount())
    parentItem->addChild(this); // insert as last element
  else
    parentItem->insertChild(ind, this); // insert at position ind

  setName(str);

  contextMenu=new QMenu("Context Menu");

  connect(this,SIGNAL(parameterChanged(const QString&)),this,SLOT(updateTreeWidgetItem(const QString&)));

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Properties", this);
  connect(action,SIGNAL(triggered()),this,SLOT(openPropertyDialog()));
  contextMenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);
}

Parameter::~Parameter() {
  delete dialog;
}

void Parameter::openPropertyDialog() {
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

void Parameter::updateElement() {
  fromWidget();
  updateTreeWidgetItem(getValue().c_str());
  mw->updateOctaveParameters();
  mw->mbsimxml(1);
}

void Parameter::initializeDialog() {
  dialog->addTab("General");
  textWidget = new TextWidget;
  ExtWidget *name=new ExtWidget("Name",textWidget);
  dialog->addToTab("General",name);
}

void Parameter::toWidget() {
  textWidget->setName(getName());
}

void Parameter::fromWidget() {
  setName(textWidget->getName());
}

void Parameter::setName(const QString &str) {
  setText(0,str);
  name = str;
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

void Parameter::remove() {
  QTreeWidget *tree = treeWidget();
  tree->invisibleRootItem()->removeChild(this);
  emit parameterChanged(getValue().c_str());
}

void Parameter::initializeUsingXML(TiXmlElement *element) {
}

TiXmlElement* Parameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+getType().toStdString());
  ele0->SetAttribute("name", getName().toStdString());
  TiXmlText* text= new TiXmlText(getValue());
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);
  return ele0;
}

Parameter* Parameter::readXMLFile(const QString &filename, QTreeWidgetItem* parent) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename.toAscii().data());
  assert(ret==true);
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
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile((name.right(15)==".mbsimparam.xml"?name:name+".mbsimparam.xml").toAscii().data());
}

void Parameter::updateTreeWidgetItem(const QString &str) {
  setText(1, str);
}

DoubleParameter::DoubleParameter(const QString &str, QTreeWidgetItem *parentItem, int ind) : Parameter(str,parentItem,ind) {

//  ParameterValueWidget *value_ = new ParameterValueWidget(new PhysicalStringWidget(new ScalarWidget("1"),QStringList(),0));
//  value.setProperty(value_);
  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"),"",""));
  value.setProperty(new ExtPhysicalVarProperty(input));

  updateTreeWidgetItem(getValue().c_str());
}

void DoubleParameter::initializeDialog() {
  Parameter::initializeDialog();
  
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),QStringList(),0));
  valueWidget = new ExtWidget("Value",new ExtPhysicalVarWidget(input));
// ParameterValueWidget *valueWidget_ = new ParameterValueWidget(new PhysicalStringWidget(new ScalarWidget("1"),QStringList(),0));
 // valueWidget = new ExtWidget("Value",valueWidget_);
  dialog->addToTab("General", valueWidget);
//  connect(valueWidget_,SIGNAL(parameterChanged(const QString&)),this,SIGNAL(parameterChanged(const QString&)));
}

void DoubleParameter::toWidget() {
  Parameter::toWidget();
  value.toWidget(valueWidget);
}

void DoubleParameter::fromWidget() {
  Parameter::fromWidget();
  value.fromWidget(valueWidget);
}

string DoubleParameter::getValue() const { 
  return static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue();
}

void DoubleParameter::initializeUsingXML(TiXmlElement *element) {
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->initializeUsingXML(element);
  TiXmlText *text = dynamic_cast<TiXmlText*>(element->FirstChild());
    if(text) {
      val->getPhysicalStringProperty(0)->setValue(text->Value());
      val->getPhysicalStringProperty(1)->setValue(text->Value());
    } 
    else if(element) {
      val->getPhysicalStringProperty(0)->initializeUsingXML(element);
      val->getPhysicalStringProperty(1)->setValue(val->getPhysicalStringProperty(0)->getValue().c_str());
    } 
    updateTreeWidgetItem(getValue().c_str());
}

