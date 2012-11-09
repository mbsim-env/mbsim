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

#include "integrator.h"
#include "editors.h"
#include "element.h"
#include "objectfactory.h"
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>

using namespace std;

Integrator::Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true) {
  if(ind==-1 || ind>=parentItem->childCount())
    parentItem->addChild(this); // insert as last element
  else
    parentItem->insertChild(ind, this); // insert at position ind

  setText(0, str);

  properties=new PropertyDialog(this);
  properties->addTab("General");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"startTime",timeUnits(),2));
  ExtPhysicalVarWidget *d = new ExtPhysicalVarWidget(input);
  startTime=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Start time", "General", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"),MBSIMINTNS"endTime",timeUnits(),2));
  d = new ExtPhysicalVarWidget(input);
  endTime=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "End time", "General", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1e-2"),MBSIMINTNS"plotStepSize",timeUnits(),2));
  d = new ExtPhysicalVarWidget(input);
  plotStepSize=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Plot step size", "General", d);

  contextMenu=new QMenu("Context Menu");
}

Integrator::~Integrator() {
  delete properties;
}

QString Integrator::getInfo() {
  return "";
}

void Integrator::saveAs() {
  QString file=QFileDialog::getSaveFileName(0, "OpenMBV Files",  QString("./")+getType()+".xml", "hdf5 Files (*.xml)");
  if(file!="") {
    if(file.contains(".xml"))
      file.chop(10);
    writeXMLFile(file.toAscii().data());
  }
}
  void Integrator::initializeUsingXML(TiXmlElement *element) {
    startTime->initializeUsingXML(element);
    endTime->initializeUsingXML(element);
    plotStepSize->initializeUsingXML(element);
  }

  TiXmlElement* Integrator::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0=new TiXmlElement(MBSIMINTNS+getType().toStdString());
    parent->LinkEndChild(ele0);
    ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSimIntegrator");

    startTime->writeXMLFile(ele0);
    endTime->writeXMLFile(ele0);
    plotStepSize->writeXMLFile(ele0);

    return ele0;
  }

Integrator* Integrator::readXMLFile(const QString &filename, QTreeWidgetItem* parent) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  assert(doc.LoadFile(filename.toAscii().data())==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);
  Integrator *integrator=ObjectFactory::getInstance()->createIntegrator(e, parent, 1);
  integrator->initializeUsingXML(doc.FirstChildElement());
  return integrator;
}

void Integrator::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  map<string, string> nsprefix=ObjectFactory::getInstance()->getNamespacePrefixMapping();
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".mbsimint.xml").toAscii().data());
}

DOPRI5Integrator::DOPRI5Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1e-6"),MBSIMINTNS"absoluteToleranceScalar",QStringList(),1));
  ExtPhysicalVarWidget *d = new ExtPhysicalVarWidget(input);
  absTol=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Absolute tolerance", "Tolerances", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",QStringList(),1));
  d = new ExtPhysicalVarWidget(input);
  relTol=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Relative tolerance", "Tolerances", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"initialStepSize",QStringList(),2));
  d = new ExtPhysicalVarWidget(input);
  initialStepSize=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Initial step size", "Step size", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"maximalStepSize",QStringList(),2));
  d = new ExtPhysicalVarWidget(input);
  maximalStepSize=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Maximal step size", "Step size", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"maximalNumberOfSteps",QStringList(),1));
  d = new ExtPhysicalVarWidget(input);
  maxSteps=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Number of maximal steps", "Step size", d);

  properties->addStretch();
}

void DOPRI5Integrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol->initializeUsingXML(element);
  relTol->initializeUsingXML(element);
  initialStepSize->initializeUsingXML(element);
  maximalStepSize->initializeUsingXML(element);
  maxSteps->initializeUsingXML(element);
}

TiXmlElement* DOPRI5Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol->writeXMLFile(ele0);
  relTol->writeXMLFile(ele0);
  initialStepSize->writeXMLFile(ele0);
  maximalStepSize->writeXMLFile(ele0);
  maxSteps->writeXMLFile(ele0);
  return ele0;
}

LSODEIntegrator::LSODEIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");
  properties->addTab("Extra");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1e-6"),MBSIMINTNS"absoluteToleranceScalar",QStringList(),1));
  ExtPhysicalVarWidget *d = new ExtPhysicalVarWidget(input);
  absTol=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Absolute tolerance", "Tolerances", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",QStringList(),1));
  d = new ExtPhysicalVarWidget(input);
  relTol=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Relative tolerance", "Tolerances", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"initialStepSize",QStringList(),2));
  d = new ExtPhysicalVarWidget(input);
  initialStepSize=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Initial step size", "Step size", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"maximalStepSize",QStringList(),2));
  d = new ExtPhysicalVarWidget(input);
  maximalStepSize=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Maximal step size", "Step size", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"minimalStepSize",QStringList(),2));
  d = new ExtPhysicalVarWidget(input);
  minimalStepSize=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Maximal step size", "Step size", d);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMINTNS"numberOfMaximalSteps",QStringList(),1));
  d = new ExtPhysicalVarWidget(input);
  maxSteps=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Number of maximal steps", "Step size", d);

  stiff = new BoolEditor(properties,  Utils::QIconCached("lines.svg"), "Stiff modus", "Extra", false);
 properties->addStretch();
}

void LSODEIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol->initializeUsingXML(element);
  relTol->initializeUsingXML(element);
  initialStepSize->initializeUsingXML(element);
  maximalStepSize->initializeUsingXML(element);
  minimalStepSize->initializeUsingXML(element);
  maxSteps->initializeUsingXML(element);
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMINTNS"stiffModus");
  stiff->setValue(Element::getBool(e));
}

TiXmlElement* LSODEIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol->writeXMLFile(ele0);
  relTol->writeXMLFile(ele0);
  initialStepSize->writeXMLFile(ele0);
  maximalStepSize->writeXMLFile(ele0);
  minimalStepSize->writeXMLFile(ele0);
  maxSteps->writeXMLFile(ele0);
  addElementText(ele0,"stiffModus",stiff->getValue());
  return ele0;
}

