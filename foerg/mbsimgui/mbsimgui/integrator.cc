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
#include "integrator.h"
#include "property_widget.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "solver.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QHBoxLayout>

using namespace std;

extern MainWindow *mw;

Integrator::Integrator(const QString &str, QTreeWidgetItem *parentItem) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true), initialState(0,false), dialog(0) {

  parentItem->addChild(this);

  setText(0, str);

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"startTime"));
  startTime.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"),"s",MBSIMINTNS"endTime"));
  endTime.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-2"),"s",MBSIMINTNS"plotStepSize"));
  plotStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0), "", MBSIMINTNS"initialState"));
  initialState.setProperty(new ExtPhysicalVarProperty(input));

  contextMenu=new QMenu("Context Menu");

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Properties", this);
  connect(action,SIGNAL(triggered()),this,SLOT(openPropertyDialog()));
  contextMenu->addAction(action);
}

Integrator::~Integrator() {
  delete dialog;
}

void Integrator::openPropertyDialog() {
  if(!dialog) {
    dialog = new PropertyDialog;
    connect(dialog,SIGNAL(apply()),this,SLOT(updateElement()));
    initializeDialog();
  }
  toWidget();
  dialog->show();
}

void Integrator::initializeDialog() {

  dialog->addTab("General");
  dialog->addTab("Initial conditions");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  startTimeWidget = new ExtWidget("Start time",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("General", startTimeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),timeUnits(),2));
  endTimeWidget = new ExtWidget("End time",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("General", endTimeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-2"),timeUnits(),2));
  plotStepSizeWidget = new ExtWidget("Plot step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("General", plotStepSizeWidget);

  input.clear();
  z0 = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(z0, QStringList(), 0));
  initialStateWidget = new ExtWidget("Initial state",new ExtPhysicalVarWidget(input),true);
  dialog->addToTab("Initial conditions", initialStateWidget);
}

void Integrator::toWidget() {
  startTime.toWidget(startTimeWidget);
  endTime.toWidget(endTimeWidget);
  plotStepSize.toWidget(plotStepSizeWidget);
  initialState.toWidget(initialStateWidget);
}

void Integrator::fromWidget() {
  startTime.fromWidget(startTimeWidget);
  endTime.fromWidget(endTimeWidget);
  plotStepSize.fromWidget(plotStepSizeWidget);
  initialState.fromWidget(initialStateWidget);
}

void Integrator::updateElement() {
  fromWidget();
  mw->mbsimxml(1);
}

void Integrator::setEndTime(double t) {
  ((ExtPhysicalVarProperty*)endTime.getProperty())->setValue(QString::number(t).toStdString());
}

void Integrator::resizeVariables() {
  z0->resize(solver->getqSize()+solver->getuSize()+solver->getxSize());
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
  startTime.initializeUsingXML(element);
  endTime.initializeUsingXML(element);
  plotStepSize.initializeUsingXML(element);
  initialState.initializeUsingXML(element);
}

TiXmlElement* Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMINTNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSimIntegrator");

  startTime.writeXMLFile(ele0);
  endTime.writeXMLFile(ele0);
  plotStepSize.writeXMLFile(ele0);
  initialState.writeXMLFile(ele0);

  return ele0;
}

Integrator* Integrator::readXMLFile(const QString &filename, QTreeWidgetItem* parent) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename.toAscii().data());
  assert(ret==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);
  Integrator *integrator=ObjectFactory::getInstance()->createIntegrator(e, parent);
  integrator->initializeUsingXML(doc.FirstChildElement());
  return integrator;
}

void Integrator::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile((name.right(13)==".mbsimint.xml"?name:name+".mbsimint.xml").toAscii().data());

}

DOPRI5Integrator::DOPRI5Integrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem), maxSteps(0,false) {
  vector<PhysicalStringProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new PropertyChoiceProperty(property)); 

  input.clear();
  property.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMINTNS"relativeTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  relTol.setProperty(new PropertyChoiceProperty(property)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMINTNS"maximalNumberOfSteps"));
  maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 
}

void DOPRI5Integrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Tolerances");
  dialog->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTolWidget = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Tolerances", absTolWidget);

  input.clear();
  widget.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  rTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(rTol,noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  relTolWidget = new ExtWidget("Relative tolerance",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Tolerances", relTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  initialStepSizeWidget = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", initialStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  maximalStepSizeWidget = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", maximalStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),QStringList(),1));
  maxStepsWidget = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
  dialog->addToTab("Step size", maxStepsWidget);
}

void DOPRI5Integrator::toWidget() {
  Integrator::toWidget();
  absTol.toWidget(absTolWidget);
  relTol.toWidget(relTolWidget);
  initialStepSize.toWidget(initialStepSizeWidget);
  maximalStepSize.toWidget(maximalStepSizeWidget);
  maxSteps.toWidget(maxStepsWidget);
}

void DOPRI5Integrator::fromWidget() {
  Integrator::fromWidget();
  absTol.fromWidget(absTolWidget);
  relTol.fromWidget(relTolWidget);
  initialStepSize.fromWidget(initialStepSizeWidget);
  maximalStepSize.fromWidget(maximalStepSizeWidget);
  maxSteps.fromWidget(maxStepsWidget);
}

void DOPRI5Integrator::resizeVariables() {
  Integrator::resizeVariables();
  if(aTol->size() != z0->size())
    aTol->setVec(getScalars<string>(z0->size(),"1e-6"));
  if(rTol->size() != z0->size())
    rTol->setVec(getScalars<string>(z0->size(),"1e-6"));
}

void DOPRI5Integrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol.initializeUsingXML(element);
  relTol.initializeUsingXML(element);
  initialStepSize.initializeUsingXML(element);
  maximalStepSize.initializeUsingXML(element);
  maxSteps.initializeUsingXML(element);
}

TiXmlElement* DOPRI5Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol.writeXMLFile(ele0);
  relTol.writeXMLFile(ele0);
  initialStepSize.writeXMLFile(ele0);
  maximalStepSize.writeXMLFile(ele0);
  maxSteps.writeXMLFile(ele0);
  return ele0;
}

RADAU5Integrator::RADAU5Integrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem) {
  vector<PhysicalStringProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new PropertyChoiceProperty(property)); 

  input.clear();
  property.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMINTNS"relativeTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  relTol.setProperty(new PropertyChoiceProperty(property)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMINTNS"maximalNumberOfSteps"));
  maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 
}

void RADAU5Integrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Tolerances");
  dialog->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTolWidget = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Tolerances", absTolWidget);

  input.clear();
  widget.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  rTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(rTol,noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  relTolWidget = new ExtWidget("Relative tolerance",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Tolerances", relTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  initialStepSizeWidget = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", initialStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  maximalStepSizeWidget = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", maximalStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),QStringList(),1));
  maxStepsWidget = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
  dialog->addToTab("Step size", maxStepsWidget);
}

void RADAU5Integrator::toWidget() {
  Integrator::toWidget();
  absTol.toWidget(absTolWidget);
  relTol.toWidget(relTolWidget);
  initialStepSize.toWidget(initialStepSizeWidget);
  maximalStepSize.toWidget(maximalStepSizeWidget);
  maxSteps.toWidget(maxStepsWidget);
}

void RADAU5Integrator::fromWidget() {
  Integrator::fromWidget();
  absTol.fromWidget(absTolWidget);
  relTol.fromWidget(relTolWidget);
  initialStepSize.fromWidget(initialStepSizeWidget);
  maximalStepSize.fromWidget(maximalStepSizeWidget);
  maxSteps.fromWidget(maxStepsWidget);
}

void RADAU5Integrator::resizeVariables() {
  Integrator::resizeVariables();
  if(aTol->size() != z0->size())
    aTol->setVec(getScalars<string>(z0->size(),"1e-6"));
  if(rTol->size() != z0->size())
    rTol->setVec(getScalars<string>(z0->size(),"1e-6"));
}

void RADAU5Integrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol.initializeUsingXML(element);
  relTol.initializeUsingXML(element);
  initialStepSize.initializeUsingXML(element);
  maximalStepSize.initializeUsingXML(element);
  maxSteps.initializeUsingXML(element);
}

TiXmlElement* RADAU5Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol.writeXMLFile(ele0);
  relTol.writeXMLFile(ele0);
  initialStepSize.writeXMLFile(ele0);
  maximalStepSize.writeXMLFile(ele0);
  maxSteps.writeXMLFile(ele0);
  return ele0;
}

LSODEIntegrator::LSODEIntegrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem), stiff(0,false) {
  vector<PhysicalStringProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new PropertyChoiceProperty(property)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  relTol.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"minimalStepSize"));
  minimalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMINTNS"numberOfMaximalSteps"));
  maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMINTNS"stiffModus"));
  stiff.setProperty(new ExtPhysicalVarProperty(input)); 
}

void LSODEIntegrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Tolerances");
  dialog->addTab("Step size");
  dialog->addTab("Extra");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTolWidget = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Tolerances", absTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
  relTolWidget = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Tolerances", relTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  initialStepSizeWidget = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", initialStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  maximalStepSizeWidget = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", maximalStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  minimalStepSizeWidget = new ExtWidget("Minimal step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", minimalStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),QStringList(),1));
  maxStepsWidget = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", maxStepsWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),QStringList(),1));
  stiffWidget = new ExtWidget("Stiff modus",new ExtPhysicalVarWidget(input),true); 
  dialog->addToTab("Extra", stiffWidget);
}

void LSODEIntegrator::toWidget() {
  Integrator::toWidget();
  absTol.toWidget(absTolWidget);
  relTol.toWidget(relTolWidget);
  initialStepSize.toWidget(initialStepSizeWidget);
  maximalStepSize.toWidget(maximalStepSizeWidget);
  maxSteps.toWidget(maxStepsWidget);
  stiff.toWidget(stiffWidget);
}

void LSODEIntegrator::fromWidget() {
  Integrator::fromWidget();
  absTol.fromWidget(absTolWidget);
  relTol.fromWidget(relTolWidget);
  initialStepSize.fromWidget(initialStepSizeWidget);
  maximalStepSize.fromWidget(maximalStepSizeWidget);
  maxSteps.fromWidget(maxStepsWidget);
  stiff.fromWidget(stiffWidget);
}

void LSODEIntegrator::resizeVariables() {
  Integrator::resizeVariables();
  if(aTol->size() != z0->size())
    aTol->setVec(getScalars<string>(z0->size(),"1e-6"));
}

void LSODEIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol.initializeUsingXML(element);
  relTol.initializeUsingXML(element);
  initialStepSize.initializeUsingXML(element);
  maximalStepSize.initializeUsingXML(element);
  minimalStepSize.initializeUsingXML(element);
  maxSteps.initializeUsingXML(element);
  stiff.initializeUsingXML(element);
}

TiXmlElement* LSODEIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol.writeXMLFile(ele0);
  relTol.writeXMLFile(ele0);
  initialStepSize.writeXMLFile(ele0);
  maximalStepSize.writeXMLFile(ele0);
  minimalStepSize.writeXMLFile(ele0);
  maxSteps.writeXMLFile(ele0);
  stiff.writeXMLFile(ele0);
  return ele0;
}

LSODARIntegrator::LSODARIntegrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem) {
  vector<PhysicalStringProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new PropertyChoiceProperty(property)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  relTol.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"minimalStepSize"));
  minimalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMINTNS"plotOnRoot"));
  plotOnRoot.setProperty(new ExtPhysicalVarProperty(input)); 
}

void LSODARIntegrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Tolerances");
  dialog->addTab("Step size");
  dialog->addTab("Extra");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTolWidget = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Tolerances", absTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
  relTolWidget = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Tolerances", relTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  initialStepSizeWidget = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", initialStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  maximalStepSizeWidget = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", maximalStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  minimalStepSizeWidget = new ExtWidget("Minimal step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", minimalStepSizeWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),QStringList(),1));
  plotOnRootWidget = new ExtWidget("Plot at root",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Extra", plotOnRootWidget);
}

void LSODARIntegrator::toWidget() {
  Integrator::toWidget();
  absTol.toWidget(absTolWidget);
  relTol.toWidget(relTolWidget);
  initialStepSize.toWidget(initialStepSizeWidget);
  maximalStepSize.toWidget(maximalStepSizeWidget);
  plotOnRoot.toWidget(plotOnRootWidget);
}

void LSODARIntegrator::fromWidget() {
  Integrator::fromWidget();
  absTol.fromWidget(absTolWidget);
  relTol.fromWidget(relTolWidget);
  initialStepSize.fromWidget(initialStepSizeWidget);
  maximalStepSize.fromWidget(maximalStepSizeWidget);
  plotOnRoot.fromWidget(plotOnRootWidget);
}

void LSODARIntegrator::resizeVariables() {
  Integrator::resizeVariables();
  if(aTol->size() != z0->size())
    aTol->setVec(getScalars<string>(z0->size(),"1e-6"));
}

void LSODARIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol.initializeUsingXML(element);
  relTol.initializeUsingXML(element);
  initialStepSize.initializeUsingXML(element);
  minimalStepSize.initializeUsingXML(element);
  maximalStepSize.initializeUsingXML(element);
  plotOnRoot.initializeUsingXML(element);
}

TiXmlElement* LSODARIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol.writeXMLFile(ele0);
  relTol.writeXMLFile(ele0);
  initialStepSize.writeXMLFile(ele0);
  minimalStepSize.writeXMLFile(ele0);
  maximalStepSize.writeXMLFile(ele0);
  plotOnRoot.writeXMLFile(ele0);
  return ele0;
}

TimeSteppingIntegrator::TimeSteppingIntegrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-3"),"s",MBSIMINTNS"stepSize"));
  stepSize.setProperty(new ExtPhysicalVarProperty(input)); 
}

void TimeSteppingIntegrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-3"),timeUnits(),2));
  stepSizeWidget = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", stepSizeWidget);
}

void TimeSteppingIntegrator::toWidget() {
  Integrator::toWidget();
  stepSize.toWidget(stepSizeWidget);
}

void TimeSteppingIntegrator::fromWidget() {
  Integrator::fromWidget();
  stepSize.fromWidget(stepSizeWidget);
}

void TimeSteppingIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  stepSize.initializeUsingXML(element);
}

TiXmlElement* TimeSteppingIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  stepSize.writeXMLFile(ele0);
  return ele0;
}


EulerExplicitIntegrator::EulerExplicitIntegrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem) {
  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-3"),"s",MBSIMINTNS"stepSize"));
  stepSize.setProperty(new ExtPhysicalVarProperty(input)); 
}

void EulerExplicitIntegrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-3"),timeUnits(),2));
  stepSizeWidget = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Step size", stepSizeWidget);
}

void EulerExplicitIntegrator::toWidget() {
  Integrator::toWidget();
  stepSize.toWidget(stepSizeWidget);
}

void EulerExplicitIntegrator::fromWidget() {
  Integrator::fromWidget();
  stepSize.fromWidget(stepSizeWidget);
}

void EulerExplicitIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  stepSize.initializeUsingXML(element);
}

TiXmlElement* EulerExplicitIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  stepSize.writeXMLFile(ele0);
  return ele0;
}

class RKSuiteTypeWidget : public Widget {

  friend class RKSuiteTypeProperty;

  public:
    RKSuiteTypeWidget() {
      QHBoxLayout *layout = new QHBoxLayout;
      layout->setMargin(0);
      setLayout(layout);
      comboBox = new QComboBox;
      comboBox->addItem("Method 23");
      comboBox->addItem("Method 45");
      comboBox->addItem("Method 67");
      layout->addWidget(comboBox);
    }
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {
      vector<string> str;
      str.push_back(MBSIMINTNS"method23");
      str.push_back(MBSIMINTNS"method45");
      str.push_back(MBSIMINTNS"method67");
      TiXmlElement *ele = new TiXmlElement(str[comboBox->currentIndex()]);
      element->LinkEndChild(ele);
      return 0;
    }
    virtual void initializeWidget() {}
    virtual void updateWidget() {}
    virtual void resizeVariables() {}
  protected:
    QComboBox *comboBox;
};

class RKSuiteTypeProperty : public Property {
  public:
    RKSuiteTypeProperty() : index(0) {
      method.push_back(MBSIMINTNS"method23");
      method.push_back(MBSIMINTNS"method45");
      method.push_back(MBSIMINTNS"method67");
    }
    void fromWidget(QWidget *widget) {
      index = static_cast<RKSuiteTypeWidget*>(widget)->comboBox->currentIndex();
    }
    void toWidget(QWidget *widget) {
      static_cast<RKSuiteTypeWidget*>(widget)->comboBox->setCurrentIndex(index);
    }
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {
      TiXmlElement *ele = new TiXmlElement(method[index]);
      element->LinkEndChild(ele);
      return 0;
    }
  protected:
    int index;
    vector<string> method;
};


RKSuiteIntegrator::RKSuiteIntegrator(const QString &str, QTreeWidgetItem *parentItem) : Integrator(str,parentItem), initialStepSize(0,false) {

  type.setProperty(new RKSuiteTypeProperty);

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  relTol.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"thresholdScalar"));
  threshold.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 
}

void RKSuiteIntegrator::initializeDialog() {
  Integrator::initializeDialog();

  dialog->addTab("Tolerances");
  dialog->addTab("Step size");

  typeWidget = new ExtWidget("Type",new RKSuiteTypeWidget);
  dialog->addToTab("General", typeWidget);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
  relTolWidget = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Tolerances", relTolWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
  thresholdWidget = new ExtWidget("Threshold",new ExtPhysicalVarWidget(input)); 
  dialog->addToTab("Tolerances", thresholdWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),timeUnits(),2));
  initialStepSizeWidget = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input),true); 
  dialog->addToTab("Step size", initialStepSizeWidget);
}

void RKSuiteIntegrator::toWidget() {
  Integrator::toWidget();
  type.toWidget(typeWidget);
  relTol.toWidget(relTolWidget);
  threshold.toWidget(thresholdWidget);
  initialStepSize.toWidget(initialStepSizeWidget);
}

void RKSuiteIntegrator::fromWidget() {
  Integrator::fromWidget();
  type.fromWidget(typeWidget);
  relTol.fromWidget(relTolWidget);
  threshold.fromWidget(thresholdWidget);
  initialStepSize.fromWidget(initialStepSizeWidget);
}

void RKSuiteIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  type.initializeUsingXML(element);
  relTol.initializeUsingXML(element);
  threshold.initializeUsingXML(element);
  initialStepSize.initializeUsingXML(element);
}

TiXmlElement* RKSuiteIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  type.writeXMLFile(ele0);
  relTol.writeXMLFile(ele0);
  threshold.writeXMLFile(ele0);
  initialStepSize.writeXMLFile(ele0);
  return ele0;
}


