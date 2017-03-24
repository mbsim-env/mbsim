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
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DOMElement* Integrator::createXMLElement(DOMNode *parent) {
    DOMElement *element = Solver::createXMLElement(parent);
    DOMDocument *doc=element->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"startTime" );
    E(ele1)->setAttribute("unit", "s");
    DOMText *text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"endTime" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"1");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"plotStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"1e-2");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return element;
  }

  DOMElement* DOPRI5Integrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"absoluteToleranceScalar" );
    DOMText *text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"relativeToleranceScalar" );
    text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"initialStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"maximalStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

  DOMElement* RADAU5Integrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"absoluteToleranceScalar" );
    DOMText *text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"relativeToleranceScalar" );
    text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"initialStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"maximalStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

//  LSODEIntegrator::LSODEIntegrator() {
//    absTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("absolute"),"",3));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"relativeToleranceScalar"));
//    relTol.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
//    initialStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"maximalStepSize"));
//    maximalStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"minimalStepSize"));
//    minimalStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"numberOfMaximalSteps"));
//    maxSteps.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"stiffModus"));
//    stiff.setProperty(new ExtPhysicalVarProperty(input));
//  }

  DOMElement* LSODEIntegrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"absoluteToleranceScalar" );
    DOMText *text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"relativeToleranceScalar" );
    text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"initialStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"maximalStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"minimalStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"numberOfMaximalSteps" );
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

//  LSODARIntegrator::LSODARIntegrator() {
//    absTol.setProperty(new ChoiceProperty2(new TolerancePropertyFactory("absolute"),"",3));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"relativeToleranceScalar"));
//    relTol.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
//    initialStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"maximalStepSize"));
//    maximalStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"minimalStepSize"));
//    minimalStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMINT%"plotOnRoot"));
//    plotOnRoot.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-5"),"",MBSIMINT%"toleranceForPositionConstraints"));
//    gMax.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-5"),"",MBSIMINT%"toleranceForVelocityConstraints"));
//    gdMax.setProperty(new ExtPhysicalVarProperty(input));
//  }

  DOMElement* LSODARIntegrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"absoluteToleranceScalar" );
    DOMText *text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"relativeToleranceScalar" );
    text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"initialStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"minimalStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"maximalStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"numberOfMaximalSteps" );
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"plotOnRoot" );
    text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

//  TimeSteppingIntegrator::TimeSteppingIntegrator() {
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-3"),"s",MBSIMINT%"stepSize"));
//    stepSize.setProperty(new ExtPhysicalVarProperty(input));
//  }

  DOMElement* TimeSteppingIntegrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"stepSize" );
    E(ele1)->setAttribute("unit", "s");
    DOMText *text = doc->createTextNode(X()%"1e-3");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

//  EulerExplicitIntegrator::EulerExplicitIntegrator() {
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-3"),"s",MBSIMINT%"stepSize"));
//    stepSize.setProperty(new ExtPhysicalVarProperty(input));
//  }

  DOMElement* EulerExplicitIntegrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"stepSize" );
    E(ele1)->setAttribute("unit", "s");
    DOMText *text = doc->createTextNode(X()%"1e-3");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

//  RKSuiteIntegrator::RKSuiteIntegrator() {
//
//    method.setProperty(new TextProperty("RK45", MBSIMINT%"method", true));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"relativeToleranceScalar"));
//    relTol.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-6"),"-",MBSIMINT%"thresholdScalar"));
//    threshold.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMINT%"initialStepSize"));
//    initialStepSize.setProperty(new ExtPhysicalVarProperty(input));
//  }

  DOMElement* RKSuiteIntegrator::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Integrator::createXMLElement(parent);
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"relativeToleranceScalar" );
    DOMText *text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"thresholdScalar" );
    text = doc->createTextNode(X()%"1e-6");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return ele0;
  }

}
