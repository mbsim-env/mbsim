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
#include "element.h"
#include "objectfactory.h"
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QHBoxLayout>

using namespace std;

Integrator::Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true) {
  if(ind==-1 || ind>=parentItem->childCount())
    parentItem->addChild(this); // insert as last element
  else
    parentItem->insertChild(ind, this); // insert at position ind

  setText(0, str);

  properties=new PropertyWidget(this);
  properties->addTab("General");
  properties->addTab("Initial conditions");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"startTime",timeUnits(),2));
  startTime = new ExtXMLWidget("Start time",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("General", startTime);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),MBSIMINTNS"endTime",timeUnits(),2));
  endTime = new ExtXMLWidget("End time",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("General", endTime);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-2"),MBSIMINTNS"plotStepSize",timeUnits(),2));
  plotStepSize = new ExtXMLWidget("Plot step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("General", plotStepSize);

  input.clear();
  z0 = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(z0, MBSIMINTNS"initialState", QStringList(), 0));
  initialState = new ExtXMLWidget("Initial state",new ExtPhysicalVarWidget(input),true);
  properties->addToTab("Initial conditions", initialState);

  contextMenu=new QMenu("Context Menu");
}

Integrator::~Integrator() {
  delete properties;
}

void Integrator::setEndTime(double t) {
  ((ExtPhysicalVarWidget*)endTime->getWidget())->setValue(QString::number(t).toStdString());
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
  startTime->initializeUsingXML(element);
  endTime->initializeUsingXML(element);
  plotStepSize->initializeUsingXML(element);
  initialState->initializeUsingXML(element);
}

TiXmlElement* Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMINTNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSimIntegrator");

  startTime->writeXMLFile(ele0);
  endTime->writeXMLFile(ele0);
  plotStepSize->writeXMLFile(ele0);
  initialState->writeXMLFile(ele0);

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
  doc.SaveFile((name.right(13)==".mbsimint.xml"?name:name+".mbsimint.xml").toAscii().data());

}

DOPRI5Integrator::DOPRI5Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"absoluteToleranceScalar",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,MBSIMINTNS"absoluteTolerance",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtXMLWidget("Absolute tolerance",new XMLWidgetChoiceWidget(name,widget)); 
  properties->addToTab("Tolerances", absTol);
  //absTol = new ExtXMLWidget("Absolute tolerance",new XMLChoiceWidget(name,widget)); 

  input.clear();
  widget.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  rTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(rTol,MBSIMINTNS"relativeTolerance",noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  relTol = new ExtXMLWidget("Relative tolerance",new XMLWidgetChoiceWidget(name,widget)); 
  //relTol = new ExtXMLWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"initialStepSize",timeUnits(),2));
  initialStepSize = new ExtXMLWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"maximalStepSize",timeUnits(),2));
  maximalStepSize = new ExtXMLWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"maximalNumberOfSteps",QStringList(),1));
  maxSteps = new ExtXMLWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
  properties->addToTab("Step size", maxSteps);

  properties->addStretch();
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

RADAU5Integrator::RADAU5Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"absoluteToleranceScalar",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,MBSIMINTNS"absoluteTolerance",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtXMLWidget("Absolute tolerance",new XMLWidgetChoiceWidget(name,widget)); 
  properties->addToTab("Tolerances", absTol);

  input.clear();
  widget.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  rTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(rTol,MBSIMINTNS"relativeTolerance",noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  relTol = new ExtXMLWidget("Relative tolerance",new XMLWidgetChoiceWidget(name,widget)); 
  //relTol = new ExtXMLWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"initialStepSize",timeUnits(),2));
  initialStepSize = new ExtXMLWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"maximalStepSize",timeUnits(),2));
  maximalStepSize = new ExtXMLWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"maximalNumberOfSteps",QStringList(),1));
  maxSteps = new ExtXMLWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
  properties->addToTab("Step size", maxSteps);

  properties->addStretch();
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
  absTol->initializeUsingXML(element);
  relTol->initializeUsingXML(element);
  initialStepSize->initializeUsingXML(element);
  maximalStepSize->initializeUsingXML(element);
  maxSteps->initializeUsingXML(element);
}

TiXmlElement* RADAU5Integrator::writeXMLFile(TiXmlNode *parent) {
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
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"absoluteToleranceScalar",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,MBSIMINTNS"absoluteTolerance",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtXMLWidget("Absolute tolerance",new XMLWidgetChoiceWidget(name,widget)); 
  properties->addToTab("Tolerances", absTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",noUnitUnits(),1));
  relTol = new ExtXMLWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"initialStepSize",timeUnits(),2));
  initialStepSize = new ExtXMLWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"maximalStepSize",timeUnits(),2));
  maximalStepSize = new ExtXMLWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"minimalStepSize",timeUnits(),2));
  minimalStepSize = new ExtXMLWidget("Minimal step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", minimalStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"numberOfMaximalSteps",QStringList(),1));
  maxSteps = new ExtXMLWidget("Number of maximal steps",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", maxSteps);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),MBSIMINTNS"stiffModus",QStringList(),1));
  stiff = new ExtXMLWidget("Stiff modus",new ExtPhysicalVarWidget(input),true); 
  properties->addToTab("Extra", stiff);

  properties->addStretch();
}

void LSODEIntegrator::resizeVariables() {
  Integrator::resizeVariables();
  if(aTol->size() != z0->size())
    aTol->setVec(getScalars<string>(z0->size(),"1e-6"));
}

void LSODEIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol->initializeUsingXML(element);
  relTol->initializeUsingXML(element);
  initialStepSize->initializeUsingXML(element);
  maximalStepSize->initializeUsingXML(element);
  minimalStepSize->initializeUsingXML(element);
  maxSteps->initializeUsingXML(element);
  stiff->initializeUsingXML(element);
}

TiXmlElement* LSODEIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol->writeXMLFile(ele0);
  relTol->writeXMLFile(ele0);
  initialStepSize->writeXMLFile(ele0);
  maximalStepSize->writeXMLFile(ele0);
  minimalStepSize->writeXMLFile(ele0);
  maxSteps->writeXMLFile(ele0);
  stiff->writeXMLFile(ele0);
  return ele0;
}

LSODARIntegrator::LSODARIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");
  properties->addTab("Extra");

  vector<PhysicalStringWidget*> input;
  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"absoluteToleranceScalar",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(aTol,MBSIMINTNS"absoluteTolerance",QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtXMLWidget("Absolute tolerance",new XMLWidgetChoiceWidget(name,widget)); 
  properties->addToTab("Tolerances", absTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",noUnitUnits(),1));
  relTol = new ExtXMLWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"initialStepSize",timeUnits(),2));
  initialStepSize = new ExtXMLWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"maximalStepSize",timeUnits(),2));
  maximalStepSize = new ExtXMLWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"minimalStepSize",timeUnits(),2));
  minimalStepSize = new ExtXMLWidget("Minimal step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", minimalStepSize);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),MBSIMINTNS"plotOnRoot",QStringList(),1));
  plotOnRoot = new ExtXMLWidget("Plot at root",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Extra", plotOnRoot);

  properties->addStretch();
}

void LSODARIntegrator::resizeVariables() {
  Integrator::resizeVariables();
  if(aTol->size() != z0->size())
    aTol->setVec(getScalars<string>(z0->size(),"1e-6"));
}

void LSODARIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  absTol->initializeUsingXML(element);
  relTol->initializeUsingXML(element);
  initialStepSize->initializeUsingXML(element);
  minimalStepSize->initializeUsingXML(element);
  maximalStepSize->initializeUsingXML(element);
  plotOnRoot->initializeUsingXML(element);
}

TiXmlElement* LSODARIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  absTol->writeXMLFile(ele0);
  relTol->writeXMLFile(ele0);
  initialStepSize->writeXMLFile(ele0);
  minimalStepSize->writeXMLFile(ele0);
  maximalStepSize->writeXMLFile(ele0);
  plotOnRoot->writeXMLFile(ele0);
  return ele0;
}

class RKSuiteTypeWidget : public XMLWidget {
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
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {
      vector<string> str;
      str.push_back(MBSIMINTNS"method23");
      str.push_back(MBSIMINTNS"method45");
      str.push_back(MBSIMINTNS"method67");
      TiXmlElement *ele = new TiXmlElement(str[comboBox->currentIndex()]);
      element->LinkEndChild(ele);
      return 0;
    }
  protected:
    QComboBox *comboBox;
};

TimeSteppingIntegrator::TimeSteppingIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-3"),MBSIMINTNS"stepSize",timeUnits(),2));
  stepSize = new ExtXMLWidget("Time step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", stepSize);

  properties->addStretch();
}

void TimeSteppingIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  stepSize->initializeUsingXML(element);
}

TiXmlElement* TimeSteppingIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  stepSize->writeXMLFile(ele0);
  return ele0;
}


EulerExplicitIntegrator::EulerExplicitIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Step size");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-3"),MBSIMINTNS"stepSize",timeUnits(),2));
  stepSize = new ExtXMLWidget("Time step size",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Step size", stepSize);

  properties->addStretch();
}

void EulerExplicitIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  stepSize->initializeUsingXML(element);
}

TiXmlElement* EulerExplicitIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  stepSize->writeXMLFile(ele0);
  return ele0;
}

RKSuiteIntegrator::RKSuiteIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind) : Integrator(str,parentItem,ind) {
  properties->addTab("Tolerances");
  properties->addTab("Step size");

  type = new ExtXMLWidget("Type",new RKSuiteTypeWidget);
  properties->addToTab("General", type);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"relativeToleranceScalar",noUnitUnits(),1));
  relTol = new ExtXMLWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1e-6"),MBSIMINTNS"thresholdScalar",noUnitUnits(),1));
  threshold = new ExtXMLWidget("Threshold",new ExtPhysicalVarWidget(input)); 
  properties->addToTab("Tolerances", threshold);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMINTNS"initialStepSize",timeUnits(),2));
  initialStepSize = new ExtXMLWidget("Initial step size",new ExtPhysicalVarWidget(input),true); 
  properties->addToTab("Step size", initialStepSize);

  properties->addStretch();
}

void RKSuiteIntegrator::initializeUsingXML(TiXmlElement *element) {
  Integrator::initializeUsingXML(element);
  type->initializeUsingXML(element);
  relTol->initializeUsingXML(element);
  threshold->initializeUsingXML(element);
  initialStepSize->initializeUsingXML(element);
}

TiXmlElement* RKSuiteIntegrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
  type->writeXMLFile(ele0);
  relTol->writeXMLFile(ele0);
  threshold->writeXMLFile(ele0);
  initialStepSize->writeXMLFile(ele0);
  return ele0;
}


