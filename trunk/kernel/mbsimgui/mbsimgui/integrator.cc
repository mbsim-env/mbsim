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

  startTime = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Start time", "General", 0, 1, " s");
  endTime = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "End time", "General", 1, 1, " s");
  plotStepSize = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Plot step size", "General", 1e-2, 1e-4, " s");

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
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"startTime");
    startTime->setValue(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"endTime");
    endTime->setValue(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"plotStepSize");
    plotStepSize->setValue(Element::getDouble(e));
//    e=element->FirstChildElement(MBSIMINTNS"initialState");
//    if(e) setInitialState(Element::getVec(e));
  }

  TiXmlElement* Integrator::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0=new TiXmlElement(getType().toStdString());
    parent->LinkEndChild(ele0);
    ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSimIntegrator");

    addElementText(ele0,"startTime",startTime->getValue());
    addElementText(ele0,"endTime",endTime->getValue());
    addElementText(ele0,"plotStepSize",plotStepSize->getValue());
//    if(getInitialState().size())
//      addElementText(ele0,"initialState",mat2str(getInitialState()));

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
  //map<string, string> nsprefix;
  //unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".mbsimint.xml").toAscii().data());
}

DOPRI5Integrator::DOPRI5Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");
  absTol = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Absolute tolerance", "Tolerances", 1e-6, 1e-6, "");
  relTol = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Relative tolerance", "Tolerances", 1e-6, 1e-6, "");
  initialStepSize = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Initial step size", "Step size", 0, 1e-6, " s");
  maximalStepSize = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Maximal step size", "Step size", 0, 1e-6, " s");
  maxSteps = new IntEditor(properties,  Utils::QIconCached("lines.svg"), "Number of maximal steps", "Step size", 2000);
  properties->addStretch();
}

void DOPRI5Integrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  TiXmlElement *e;
//  e=element->FirstChildElement(MBSIMINTNS"absoluteTolerance");
//  if(e) setAbsoluteTolerance(Element::getVec(e));
  e=element->FirstChildElement(MBSIMINTNS"absoluteToleranceScalar");
  if(e) absTol->setValue(Element::getDouble(e));
//  e=element->FirstChildElement(MBSIMINTNS"relativeTolerance");
//  if(e) setRelativeTolerance(Element::getVec(e));
  e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
  if(e) relTol->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"initialStepSize");
  initialStepSize->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"maximalStepSize");
  maximalStepSize->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"maximalNumberOfSteps");
  if (e)
    maxSteps->setValue(Element::getInt(e));
}

TiXmlElement* DOPRI5Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
//  if(getAbsoluteTolerance().size() > 1) 
//    addElementText(ele0,"absoluteTolerance",mat2str(getAbsoluteTolerance()));
//  else
    addElementText(ele0,"absoluteToleranceScalar",absTol->getValue());
//  if(getRelativeTolerance().size() > 1) 
//    addElementText(ele0,"relativeTolerance",mat2str(getRelativeTolerance()));
//  else
//    addElementText(ele0,"relativeToleranceScalar",getRelativeTolerance()(0));
    addElementText(ele0,"relativeToleranceScalar",relTol->getValue());
    addElementText(ele0,"initialStepSize",initialStepSize->getValue());
    addElementText(ele0,"maximalStepSize",maximalStepSize->getValue());
    addElementText(ele0,"maximalNumberOfSteps",maxSteps->getValue());
  return ele0;
}

LSODEIntegrator::LSODEIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");
  properties->addTab("Extra");
  absTol = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Absolute tolerance", "Tolerances", 1e-6, 1e-6, "");
  relTol = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Relative tolerance", "Tolerances", 1e-6, 1e-6, "");
  initialStepSize = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Initial step size", "Step size", 0, 1e-6, " s");
  maximalStepSize = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Maximal step size", "Step size", 0, 1e-6, " s");
  minimalStepSize = new DoubleEditor(properties,  Utils::QIconCached("lines.svg"), "Minimal step size", "Step size", 0, 1e-6, " s");
  maxSteps = new IntEditor(properties,  Utils::QIconCached("lines.svg"), "Number of maximal steps", "Step size", 2000);
  stiff = new BoolEditor(properties,  Utils::QIconCached("lines.svg"), "Stiff modus", "Extra", false);
 properties->addStretch();
}

void LSODEIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  TiXmlElement *e;
//  e=element->FirstChildElement(MBSIMINTNS"absoluteTolerance");
//  if(e) setAbsoluteTolerance(Element::getVec(e));
  e=element->FirstChildElement(MBSIMINTNS"absoluteToleranceScalar");
  if(e) absTol->setValue(Element::getDouble(e));
//  e=element->FirstChildElement(MBSIMINTNS"relativeTolerance");
//  if(e) setRelativeTolerance(Element::getVec(e));
  e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
  if(e) relTol->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"initialStepSize");
  initialStepSize->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"maximalStepSize");
  maximalStepSize->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"minimalStepSize");
  minimalStepSize->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMINTNS"numberOfMaximalSteps");
  maxSteps->setValue(Element::getInt(e));
  e=element->FirstChildElement(MBSIMINTNS"stiffModus");
  stiff->setValue(Element::getBool(e));
}

TiXmlElement* LSODEIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
//  if(getAbsoluteTolerance().size() > 1) 
//    addElementText(ele0,"absoluteTolerance",mat2str(getAbsoluteTolerance()));
//  else
    addElementText(ele0,"absoluteToleranceScalar",absTol->getValue());
//  if(getRelativeTolerance().size() > 1) 
//    addElementText(ele0,"relativeTolerance",mat2str(getRelativeTolerance()));
//  else
//    addElementText(ele0,"relativeToleranceScalar",getRelativeTolerance()(0));
    addElementText(ele0,"relativeToleranceScalar",relTol->getValue());
    addElementText(ele0,"initialStepSize",initialStepSize->getValue());
    addElementText(ele0,"maximalStepSize",maximalStepSize->getValue());
    addElementText(ele0,"minimalStepSize",minimalStepSize->getValue());
    addElementText(ele0,"numberOfMaximalSteps",maxSteps->getValue());
    addElementText(ele0,"stiffModus",stiff->getValue());
  return ele0;
}

