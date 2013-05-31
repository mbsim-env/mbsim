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
#include "integrator_properties.h"
#include "solver.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QHBoxLayout>

using namespace std;
using namespace MBXMLUtils;

extern MainWindow *mw;

Integrator::Integrator() : initialState(0,false) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"startTime"));
  startTime.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"),"s",MBSIMINTNS"endTime"));
  endTime.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-2"),"s",MBSIMINTNS"plotStepSize"));
  plotStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0), "", MBSIMINTNS"initialState"));
  initialState.setProperty(new ExtPhysicalVarProperty(input));
}

Integrator::~Integrator() {
}

void Integrator::initializeUsingXML(TiXmlElement *element) {
  startTime.initializeUsingXML(element);
  endTime.initializeUsingXML(element);
  plotStepSize.initializeUsingXML(element);
  initialState.initializeUsingXML(element);
}

TiXmlElement* Integrator::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMINTNS+getType());
  parent->LinkEndChild(ele0);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSimIntegrator");

  startTime.writeXMLFile(ele0);
  endTime.writeXMLFile(ele0);
  plotStepSize.writeXMLFile(ele0);
  initialState.writeXMLFile(ele0);

  return ele0;
}

Integrator* Integrator::readXMLFile(const string &filename) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    TiXml_setLineNrFromProcessingInstruction(e);
    map<string,string> dummy;
    incorporateNamespace(e, dummy);
    Integrator *integrator=ObjectFactory::getInstance()->createIntegrator(e);
    if(integrator)
      integrator->initializeUsingXML(doc.FirstChildElement());
    return integrator;
  }
  return 0;
}

void Integrator::writeXMLFile(const string &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile(name.substr(name.length()-13,13)==".mbsimint.xml"?name:name+".mbsimint.xml");
}

DOPRI5Integrator::DOPRI5Integrator() : maxSteps(0,false) {
  vector<PhysicalVariableProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new ChoiceProperty("",property)); 

  input.clear();
  property.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0),"",MBSIMINTNS"relativeTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  relTol.setProperty(new ChoiceProperty("",property)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINTNS"maximalNumberOfSteps"));
  maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 
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

RADAU5Integrator::RADAU5Integrator() {
  vector<PhysicalVariableProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new ChoiceProperty("",property)); 

  input.clear();
  property.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0),"",MBSIMINTNS"relativeTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  relTol.setProperty(new ChoiceProperty("",property)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINTNS"maximalNumberOfSteps"));
  maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 
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

LSODEIntegrator::LSODEIntegrator() : stiff(0,false) {
  vector<PhysicalVariableProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new ChoiceProperty("",property)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  relTol.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"minimalStepSize"));
  minimalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINTNS"numberOfMaximalSteps"));
  maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINTNS"stiffModus"));
  stiff.setProperty(new ExtPhysicalVarProperty(input)); 
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

LSODARIntegrator::LSODARIntegrator() {
  vector<PhysicalVariableProperty*> input;
  vector<Property*> property;
  vector<string> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"",MBSIMINTNS"absoluteToleranceScalar"));
  property.push_back(new ExtPhysicalVarProperty(input));
  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(0),"",MBSIMINTNS"absoluteTolerance"));
  property.push_back(new ExtPhysicalVarProperty(input));
  absTol.setProperty(new ChoiceProperty("",property)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  relTol.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"maximalStepSize"));
  maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"minimalStepSize"));
  minimalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINTNS"plotOnRoot"));
  plotOnRoot.setProperty(new ExtPhysicalVarProperty(input)); 
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

TimeSteppingIntegrator::TimeSteppingIntegrator() {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-3"),"s",MBSIMINTNS"stepSize"));
  stepSize.setProperty(new ExtPhysicalVarProperty(input)); 
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

EulerExplicitIntegrator::EulerExplicitIntegrator() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-3"),"s",MBSIMINTNS"stepSize"));
  stepSize.setProperty(new ExtPhysicalVarProperty(input)); 
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

RKSuiteIntegrator::RKSuiteIntegrator() : initialStepSize(0,false) {

  type.setProperty(new RKSuiteTypeProperty);

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"relativeToleranceScalar"));
  relTol.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINTNS"thresholdScalar"));
  threshold.setProperty(new ExtPhysicalVarProperty(input)); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINTNS"initialStepSize"));
  initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 
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


