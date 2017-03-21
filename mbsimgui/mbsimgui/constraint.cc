/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2016 Martin Förg

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
#include "constraint.h"
#include "frame.h"
#include "rigid_body.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "ombv_properties.h"
#include "mainwindow.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Property* RigidBodyOfReferencePropertyFactory::createProperty(int i) {
    return new RigidBodyOfReferenceProperty("",element,xmlName);
  }

  Property* GeneralizedGearConstraintPropertyFactory::createProperty(int i) {
    return new GearInputReferenceProperty("",element,xmlName);
  }

  Constraint* Constraint::readXMLFile(const string &filename) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Constraint *constraint=Embed<Constraint>::createAndInit(e);
    if(constraint) constraint->initialize();
    return constraint;
  }

  GeneralizedConstraint::GeneralizedConstraint(const string &str) : MechanicalConstraint(str), support(0,false) {
    support.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"supportFrame"));
  }

  void GeneralizedConstraint::initialize() {
    support.initialize();
  }

  DOMElement* GeneralizedConstraint::initializeUsingXML(DOMElement *element) {
    MechanicalConstraint::initializeUsingXML(element);
    support.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = MechanicalConstraint::writeXMLFile(parent);
    support.writeXMLFile(ele0);
    return ele0;
  }

  GeneralizedGearConstraint::GeneralizedGearConstraint(const string &str) : GeneralizedConstraint(str) {

    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));

    independentBodies.setProperty(new ListProperty(new GeneralizedGearConstraintPropertyFactory(this),MBSIM%"independentRigidBody"));
  }

  void GeneralizedGearConstraint::initialize() {
    GeneralizedConstraint::initialize();
    dependentBody.initialize();
    independentBodies.initialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(true);
  }

  void GeneralizedGearConstraint::deinitialize() {
    GeneralizedConstraint::deinitialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(false);
  }

  DOMElement* GeneralizedGearConstraint::initializeUsingXML(DOMElement *element) {
    GeneralizedConstraint::initializeUsingXML(element);
    dependentBody.initializeUsingXML(element);
    independentBodies.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedGearConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = GeneralizedConstraint::writeXMLFile(parent);
    dependentBody.writeXMLFile(ele0);
    independentBodies.writeXMLFile(ele0);
    return ele0;
  }

  GeneralizedDualConstraint::GeneralizedDualConstraint(const string &str) : GeneralizedConstraint(str), independentBody(0,false) {

    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));

    independentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"independentRigidBody"));
  }

  void GeneralizedDualConstraint::initialize() {
    GeneralizedConstraint::initialize();
    dependentBody.initialize();
    independentBody.initialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(true);
  }

  void GeneralizedDualConstraint::deinitialize() {
    GeneralizedConstraint::deinitialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(false);
  }

  DOMElement* GeneralizedDualConstraint::initializeUsingXML(DOMElement *element) {
    GeneralizedConstraint::initializeUsingXML(element);
    dependentBody.initializeUsingXML(element);
    independentBody.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedDualConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = GeneralizedConstraint::writeXMLFile(parent);
    dependentBody.writeXMLFile(ele0);
    independentBody.writeXMLFile(ele0);
    return ele0;
  }

  GeneralizedPositionConstraint::GeneralizedPositionConstraint(const string &str) : GeneralizedDualConstraint(str), constraintFunction(0,false) {

    constraintFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"constraintFunction"));
  }

  DOMElement* GeneralizedPositionConstraint::initializeUsingXML(DOMElement *element) {
    GeneralizedDualConstraint::initializeUsingXML(element);
    constraintFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedPositionConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = GeneralizedDualConstraint::writeXMLFile(parent);

    constraintFunction.writeXMLFile(ele0);

    return ele0;
  }

  GeneralizedVelocityConstraint::GeneralizedVelocityConstraint(const string &str) : GeneralizedDualConstraint(str), constraintFunction(0,false), x0(0,false) {

    constraintFunction.setProperty(new ChoiceProperty2(new ConstraintPropertyFactory(this),"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialState"));
    x0.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* GeneralizedVelocityConstraint::initializeUsingXML(DOMElement *element) {
    GeneralizedDualConstraint::initializeUsingXML(element);
    x0.initializeUsingXML(element);
    constraintFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedVelocityConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = GeneralizedDualConstraint::writeXMLFile(parent);

    x0.writeXMLFile(ele0);
    constraintFunction.writeXMLFile(ele0);

    return ele0;
  }

  GeneralizedAccelerationConstraint::GeneralizedAccelerationConstraint(const string &str) : GeneralizedDualConstraint(str), constraintFunction(0,false), x0(0,false) {

    constraintFunction.setProperty(new ChoiceProperty2(new ConstraintPropertyFactory(this),"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialState"));
    x0.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* GeneralizedAccelerationConstraint::initializeUsingXML(DOMElement *element) {
    GeneralizedDualConstraint::initializeUsingXML(element);
    x0.initializeUsingXML(element);
    constraintFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedAccelerationConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = GeneralizedDualConstraint::writeXMLFile(parent);

    x0.writeXMLFile(ele0);
    constraintFunction.writeXMLFile(ele0);

    return ele0;
  }

  JointConstraint::JointConstraint(const string &str) : MechanicalConstraint(str), refFrameID(0,false), force(0,false), moment(0,false), q0(0,false) {

    dependentBodiesFirstSide.setProperty(new ListProperty(new RigidBodyOfReferencePropertyFactory(this,""),MBSIM%"dependentRigidBodyOnFirstSide"));

    dependentBodiesSecondSide.setProperty(new ListProperty(new RigidBodyOfReferencePropertyFactory(this,""),MBSIM%"dependentRigidBodyOnSecondSide"));

    independentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"independentRigidBody"));

    connections.setProperty(new ConnectFramesProperty(2,this));

    refFrameID.setProperty(new IntegerProperty(0,MBSIM%"frameOfReferenceID"));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"forceDirection"));
    force.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"momentDirection"));
    moment.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialGuess"));
    q0.setProperty(new ExtPhysicalVarProperty(input));
  }

  void JointConstraint::initialize() {
    MechanicalConstraint::initialize();
    dependentBodiesFirstSide.initialize();
    dependentBodiesSecondSide.initialize();
    independentBody.initialize();
    connections.initialize();
    ListProperty *list = static_cast<ListProperty*>(dependentBodiesFirstSide.getProperty());
    for(int i=0; i<list->getSize(); i++) {
      RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(list->getProperty(i))->getBodyPtr();
      if(body)
        body->setConstrained(true);
    }
    list = static_cast<ListProperty*>(dependentBodiesSecondSide.getProperty());
    for(int i=0; i<list->getSize(); i++) {
      RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(list->getProperty(i))->getBodyPtr();
      if(body)
        body->setConstrained(true);
    }
  }

  void JointConstraint::deinitialize() {
    MechanicalConstraint::deinitialize();
    ListProperty *list = static_cast<ListProperty*>(dependentBodiesFirstSide.getProperty());
    for(int i=0; i<list->getSize(); i++) {
      RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(list->getProperty(i))->getBodyPtr();
      if(body)
        body->setConstrained(false);
    }
    list = static_cast<ListProperty*>(dependentBodiesSecondSide.getProperty());
    for(int i=0; i<list->getSize(); i++) {
      RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(list->getProperty(i))->getBodyPtr();
      if(body)
        body->setConstrained(false);
    }
  }

  DOMElement* JointConstraint::initializeUsingXML(DOMElement *element) {
    MechanicalConstraint::initializeUsingXML(element);

    dependentBodiesFirstSide.initializeUsingXML(element);
    dependentBodiesSecondSide.initializeUsingXML(element);

    independentBody.initializeUsingXML(element);

    connections.initializeUsingXML(element);

    refFrameID.initializeUsingXML(element);

    force.initializeUsingXML(element);
    moment.initializeUsingXML(element);

    q0.initializeUsingXML(element);

    return element;
  }

  DOMElement* JointConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = MechanicalConstraint::writeXMLFile(parent);

    dependentBodiesFirstSide.writeXMLFile(ele0);
    dependentBodiesSecondSide.writeXMLFile(ele0);

    independentBody.writeXMLFile(ele0);

    connections.writeXMLFile(ele0);

    refFrameID.writeXMLFile(ele0);

    force.writeXMLFile(ele0);
    moment.writeXMLFile(ele0);

    q0.writeXMLFile(ele0);

    return ele0;
  }

}
