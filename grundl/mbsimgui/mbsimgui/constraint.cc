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
#include "constraint.h"
#include "frame.h"
#include "rigidbody.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "ombv_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  class RigidBodyOfReferencePropertyFactory : public PropertyFactory {
    public:
      RigidBodyOfReferencePropertyFactory(Element *element_, const FQN &xmlName_) : element(element_), xmlName(xmlName_) { }
      Property* createProperty(int i=0);
    protected:
      Element *element;
      FQN xmlName;
  };

  Property* RigidBodyOfReferencePropertyFactory::createProperty(int i) {
    return new RigidBodyOfReferenceProperty("",element,xmlName);
  }

  class GearConstraintPropertyFactory : public PropertyFactory {
    public:
      GearConstraintPropertyFactory(Element *element_) : element(element_) { }
      Property* createProperty(int i=0);
    protected:
      Element *element;
  };

  Property* GearConstraintPropertyFactory::createProperty(int i) {
    ContainerProperty *property = new ContainerProperty;
    property->addProperty(new RigidBodyOfReferenceProperty("",element, MBSIM%"rigidBody"));
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "", MBSIM%"ratio"));
    property->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));
    return property;
  }

  Constraint::Constraint(const string &str, Element *parent) : Object(str, parent) {
  }

  GearConstraint::GearConstraint(const string &str, Element *parent) : Constraint(str, parent), gearForceArrow(0,false), gearMomentArrow(0,false) {

    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));

    independentBodies.setProperty(new ListProperty(new GearConstraintPropertyFactory(this),MBSIM%"Transmission"));
    independentBodies.setXMLName(MBSIM%"transmissions");

    gearForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    gearForceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    gearMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    gearMomentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
  }

  void GearConstraint::initialize() {
    Constraint::initialize();
    dependentBody.initialize();
    independentBodies.initialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(true);
  }

  void GearConstraint::deinitialize() {
    Constraint::deinitialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(false);
  }

  void GearConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    Constraint::initializeUsingXML(element);
    dependentBody.initializeUsingXML(element);
    independentBodies.initializeUsingXML(element);
    gearForceArrow.initializeUsingXML(element);
    gearMomentArrow.initializeUsingXML(element);
  }

  DOMElement* GearConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Constraint::writeXMLFile(parent);
    dependentBody.writeXMLFile(ele0);
    independentBodies.writeXMLFile(ele0);
    gearForceArrow.writeXMLFile(ele0);
    gearMomentArrow.writeXMLFile(ele0);
    return ele0;
  }

  KinematicConstraint::KinematicConstraint(const string &str, Element *parent) : Constraint(str, parent), constraintForceArrow(0,false), constraintMomentArrow(0,false) {

    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));

    constraintForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    constraintForceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    constraintMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    constraintMomentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
  }

  void KinematicConstraint::initialize() {
    Constraint::initialize();
    dependentBody.initialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(true);
  }

  void KinematicConstraint::deinitialize() {
    Constraint::deinitialize();
    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
    if(body)
      body->setConstrained(false);
  }

  void KinematicConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    Constraint::initializeUsingXML(element);
    dependentBody.initializeUsingXML(element);
    constraintForceArrow.initializeUsingXML(element);
    constraintMomentArrow.initializeUsingXML(element);
  }

  DOMElement* KinematicConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Constraint::writeXMLFile(parent);

    dependentBody.writeXMLFile(ele0);
    constraintForceArrow.writeXMLFile(ele0);
    constraintMomentArrow.writeXMLFile(ele0);

    return ele0;
  }

  GeneralizedPositionConstraint::GeneralizedPositionConstraint(const string &str, Element *parent) : KinematicConstraint(str, parent), constraintFunction(0,false) {

    constraintFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIM%"constraintFunction"));
  }

  void GeneralizedPositionConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    KinematicConstraint::initializeUsingXML(element);
    constraintFunction.initializeUsingXML(element);
  }

  DOMElement* GeneralizedPositionConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = KinematicConstraint::writeXMLFile(parent);

    constraintFunction.writeXMLFile(ele0);

    return ele0;
  }

  GeneralizedVelocityConstraint::GeneralizedVelocityConstraint(const string &str, Element *parent) : KinematicConstraint(str, parent), constraintFunction(0,false), x0(0,false) {

    constraintFunction.setProperty(new ChoiceProperty2(new ConstraintPropertyFactory,"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialState"));
    x0.setProperty(new ExtPhysicalVarProperty(input));
  }

  void GeneralizedVelocityConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    KinematicConstraint::initializeUsingXML(element);
    x0.initializeUsingXML(element);
    constraintFunction.initializeUsingXML(element);
  }

  DOMElement* GeneralizedVelocityConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = KinematicConstraint::writeXMLFile(parent);

    x0.writeXMLFile(ele0);
    constraintFunction.writeXMLFile(ele0);

    return ele0;
  }

  GeneralizedAccelerationConstraint::GeneralizedAccelerationConstraint(const string &str, Element *parent) : KinematicConstraint(str, parent), constraintFunction(0,false), x0(0,false) {

    constraintFunction.setProperty(new ChoiceProperty2(new ConstraintPropertyFactory,"",3)); 

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialState"));
    x0.setProperty(new ExtPhysicalVarProperty(input));
  }

  void GeneralizedAccelerationConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    KinematicConstraint::initializeUsingXML(element);
    x0.initializeUsingXML(element);
    constraintFunction.initializeUsingXML(element);
  }

  DOMElement* GeneralizedAccelerationConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = KinematicConstraint::writeXMLFile(parent);

    x0.writeXMLFile(ele0);
    constraintFunction.writeXMLFile(ele0);

    return ele0;
  }

  JointConstraint::JointConstraint(const string &str, Element *parent) : Constraint(str, parent), refFrameID(0,false), force(0,false), moment(0,false), jointForceArrow(0,false), jointMomentArrow(0,false), q0(0,false) {

    independentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"independentRigidBody"));

    dependentBodiesFirstSide.setProperty(new ListProperty(new RigidBodyOfReferencePropertyFactory(this,""),MBSIM%"dependentRigidBody"));
    dependentBodiesFirstSide.setXMLName(MBSIM%"dependentRigidBodiesFirstSide");

    dependentBodiesSecondSide.setProperty(new ListProperty(new RigidBodyOfReferencePropertyFactory(this,""),MBSIM%"dependentRigidBody"));
    dependentBodiesSecondSide.setXMLName(MBSIM%"dependentRigidBodiesSecondSide");

    refFrameID.setProperty(new IntegerProperty(0,MBSIM%"frameOfReferenceID"));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"forceDirection"));
    force.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"momentDirection"));
    moment.setProperty(new ExtPhysicalVarProperty(input));

    connections.setProperty(new ConnectFramesProperty(2,this));

    jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    jointForceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    jointMomentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);

    input.clear();
    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialGuess"));
    q0.setProperty(new ExtPhysicalVarProperty(input));
  }

  void JointConstraint::initialize() {
    Constraint::initialize();
    independentBody.initialize();
    dependentBodiesFirstSide.initialize();
    dependentBodiesSecondSide.initialize();
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
    Constraint::deinitialize();
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

  void JointConstraint::initializeUsingXML(DOMElement *element) {
    Constraint::initializeUsingXML(element);

    q0.initializeUsingXML(element);

    dependentBodiesFirstSide.initializeUsingXML(element);
    dependentBodiesSecondSide.initializeUsingXML(element);

    independentBody.initializeUsingXML(element);

    refFrameID.initializeUsingXML(element);
    force.initializeUsingXML(element);
    moment.initializeUsingXML(element);

    connections.initializeUsingXML(element);

    jointForceArrow.initializeUsingXML(element);
    jointMomentArrow.initializeUsingXML(element);
  }

  DOMElement* JointConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Constraint::writeXMLFile(parent);

    q0.writeXMLFile(ele0);

    dependentBodiesFirstSide.writeXMLFile(ele0);
    dependentBodiesSecondSide.writeXMLFile(ele0);

    independentBody.writeXMLFile(ele0);

    refFrameID.writeXMLFile(ele0);
    force.writeXMLFile(ele0);
    moment.writeXMLFile(ele0);

    connections.writeXMLFile(ele0);

    jointForceArrow.writeXMLFile(ele0);
    jointMomentArrow.writeXMLFile(ele0);

    return ele0;
  }

}
