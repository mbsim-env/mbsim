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
#include "basic_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  TolerancePropertyFactory::TolerancePropertyFactory(const string &type_) : type(type_) {
    name.push_back(MBSIMINT%(type+"ToleranceScalar"));
    name.push_back(MBSIMINT%(type+"Tolerance"));
  }

  Property* TolerancePropertyFactory::createProperty(int i) {
    if(i==0) {
      vector<PhysicalVariableProperty> input;
      input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"",MBSIMINT%(type+"ToleranceScalar")));
      return new ExtPhysicalVarProperty(input);
    }
    if(i==1) {
      vector<PhysicalVariableProperty> input;
      input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIMINT%(type+"Tolerance")));
      return new ExtPhysicalVarProperty(input);
    }
    return NULL;
  }

  Integrator::Integrator() : initialState(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"startTime"));
    startTime.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"s",MBSIMINT%"endTime"));
    endTime.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-2"),"s",MBSIMINT%"plotStepSize"));
    plotStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    initialState.setProperty(new ChoiceProperty2(new VecPropertyFactory(0,MBSIMINT%"initialState",vector<string>(3,"")),"",4));
  }

  Integrator::~Integrator() {
  }

  void Integrator::initializeUsingXML(DOMElement *element) {
    startTime.initializeUsingXML(element);
    endTime.initializeUsingXML(element);
    plotStepSize.initializeUsingXML(element);
    initialState.initializeUsingXML(element);
  }

  DOMElement* Integrator::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(MBSIMINT%getType());
    parent->insertBefore(ele0, NULL);

    startTime.writeXMLFile(ele0);
    endTime.writeXMLFile(ele0);
    plotStepSize.writeXMLFile(ele0);
    initialState.writeXMLFile(ele0);

    return ele0;
  }

  DOPRI5Integrator::DOPRI5Integrator() : maxSteps(0,false) {
    absTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("absolute"),"",3)); 

    relTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("relative"),"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
    initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"maximalStepSize"));
    maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"maximalNumberOfSteps"));
    maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 
  }

  void DOPRI5Integrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    absTol.initializeUsingXML(element);
    relTol.initializeUsingXML(element);
    initialStepSize.initializeUsingXML(element);
    maximalStepSize.initializeUsingXML(element);
    maxSteps.initializeUsingXML(element);
  }

  DOMElement* DOPRI5Integrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    absTol.writeXMLFile(ele0);
    relTol.writeXMLFile(ele0);
    initialStepSize.writeXMLFile(ele0);
    maximalStepSize.writeXMLFile(ele0);
    maxSteps.writeXMLFile(ele0);
    return ele0;
  }

  RADAU5Integrator::RADAU5Integrator() {
    absTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("absolute"),"",3)); 

    relTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("relative"),"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
    initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"maximalStepSize"));
    maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"maximalNumberOfSteps"));
    maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 
  }

  void RADAU5Integrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    absTol.initializeUsingXML(element);
    relTol.initializeUsingXML(element);
    initialStepSize.initializeUsingXML(element);
    maximalStepSize.initializeUsingXML(element);
    maxSteps.initializeUsingXML(element);
  }

  DOMElement* RADAU5Integrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    absTol.writeXMLFile(ele0);
    relTol.writeXMLFile(ele0);
    initialStepSize.writeXMLFile(ele0);
    maximalStepSize.writeXMLFile(ele0);
    maxSteps.writeXMLFile(ele0);
    return ele0;
  }

  LSODEIntegrator::LSODEIntegrator() : stiff(0,false) {
    absTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("absolute"),"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"relativeToleranceScalar"));
    relTol.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
    initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"maximalStepSize"));
    maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"minimalStepSize"));
    minimalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"numberOfMaximalSteps"));
    maxSteps.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"stiffModus"));
    stiff.setProperty(new ExtPhysicalVarProperty(input)); 
  }

  void LSODEIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    absTol.initializeUsingXML(element);
    relTol.initializeUsingXML(element);
    initialStepSize.initializeUsingXML(element);
    maximalStepSize.initializeUsingXML(element);
    minimalStepSize.initializeUsingXML(element);
    maxSteps.initializeUsingXML(element);
    stiff.initializeUsingXML(element);
  }

  DOMElement* LSODEIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    absTol.writeXMLFile(ele0);
    relTol.writeXMLFile(ele0);
    initialStepSize.writeXMLFile(ele0);
    maximalStepSize.writeXMLFile(ele0);
    minimalStepSize.writeXMLFile(ele0);
    maxSteps.writeXMLFile(ele0);
    stiff.writeXMLFile(ele0);
    return ele0;
  }

  LSODARIntegrator::LSODARIntegrator() : gMax(0,false), gdMax(0,false) {
    absTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("absolute"),"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"relativeToleranceScalar"));
    relTol.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
    initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"maximalStepSize"));
    maximalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"minimalStepSize"));
    minimalStepSize.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"plotOnRoot"));
    plotOnRoot.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-5"),"",MBSIMINT%"toleranceForPositionConstraints"));
    gMax.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-5"),"",MBSIMINT%"toleranceForVelocityConstraints"));
    gdMax.setProperty(new ExtPhysicalVarProperty(input));
  }

  void LSODARIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    absTol.initializeUsingXML(element);
    relTol.initializeUsingXML(element);
    initialStepSize.initializeUsingXML(element);
    minimalStepSize.initializeUsingXML(element);
    maximalStepSize.initializeUsingXML(element);
    plotOnRoot.initializeUsingXML(element);
    gMax.initializeUsingXML(element);
    gdMax.initializeUsingXML(element);
  }

  DOMElement* LSODARIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    absTol.writeXMLFile(ele0);
    relTol.writeXMLFile(ele0);
    initialStepSize.writeXMLFile(ele0);
    minimalStepSize.writeXMLFile(ele0);
    maximalStepSize.writeXMLFile(ele0);
    plotOnRoot.writeXMLFile(ele0);
    gMax.writeXMLFile(ele0);
    gdMax.writeXMLFile(ele0);
    return ele0;
  }

  TimeSteppingIntegrator::TimeSteppingIntegrator() {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-3"),"s",MBSIMINT%"stepSize"));
    stepSize.setProperty(new ExtPhysicalVarProperty(input)); 
  }

  void TimeSteppingIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    stepSize.initializeUsingXML(element);
  }

  DOMElement* TimeSteppingIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    stepSize.writeXMLFile(ele0);
    return ele0;
  }

  EulerExplicitIntegrator::EulerExplicitIntegrator() {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-3"),"s",MBSIMINT%"stepSize"));
    stepSize.setProperty(new ExtPhysicalVarProperty(input)); 
  }

  void EulerExplicitIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    stepSize.initializeUsingXML(element);
  }

  DOMElement* EulerExplicitIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    stepSize.writeXMLFile(ele0);
    return ele0;
  }

  RKSuiteIntegrator::RKSuiteIntegrator() : method(0,false), initialStepSize(0,false) {

    method.setProperty(new TextProperty("RK45", MBSIMINT%"method", true));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"relativeToleranceScalar"));
    relTol.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"thresholdScalar"));
    threshold.setProperty(new ExtPhysicalVarProperty(input)); 

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
    initialStepSize.setProperty(new ExtPhysicalVarProperty(input)); 
  }

  void RKSuiteIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    method.initializeUsingXML(element);
    relTol.initializeUsingXML(element);
    threshold.initializeUsingXML(element);
    initialStepSize.initializeUsingXML(element);
  }

  DOMElement* RKSuiteIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
    method.writeXMLFile(ele0);
    relTol.writeXMLFile(ele0);
    threshold.writeXMLFile(ele0);
    initialStepSize.writeXMLFile(ele0);
    return ele0;
  }

}
