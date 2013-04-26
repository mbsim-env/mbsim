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

using namespace std;
using namespace MBXMLUtils;

Constraint::Constraint(const string &str, Element *parent) : Object(str, parent) {
}

Constraint::~Constraint() {
}

GearConstraint::GearConstraint(const string &str, Element *parent) : Constraint(str, parent) {

  dependentBody.setProperty(new RigidBodyOfReferenceProperty(0,this,MBSIMNS"dependentRigidBody"));

  independentBodies.setProperty(new GearDependenciesProperty(this,MBSIMNS"independentRigidBodies"));

}

GearConstraint::~GearConstraint() {
}

void GearConstraint::initialize() {
  Constraint::initialize();
  dependentBody.initialize();
  independentBodies.initialize();
}


void GearConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e, *ee;
  Constraint::initializeUsingXML(element);
  dependentBody.initializeUsingXML(element);
  independentBodies.initializeUsingXML(element);
}

TiXmlElement* GearConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Constraint::writeXMLFile(parent);
  dependentBody.writeXMLFile(ele0);
  independentBodies.writeXMLFile(ele0);
  return ele0;
}

KinematicConstraint::KinematicConstraint(const string &str, Element *parent) : Constraint(str, parent), kinematicFunction(0,false), firstDerivativeOfKinematicFunction(0,false), secondDerivativeOfKinematicFunction(0,false) {

  dependentBody.setProperty(new RigidBodyOfReferenceProperty(0,this,MBSIMNS"dependentRigidBody"));

  kinematicFunction.setProperty(new Function1ChoiceProperty(MBSIMNS"kinematicFunction"));

  firstDerivativeOfKinematicFunction.setProperty(new Function1ChoiceProperty(MBSIMNS"firstDerivativeOfKinematicFunction"));

  secondDerivativeOfKinematicFunction.setProperty(new Function1ChoiceProperty(MBSIMNS"secondDerivativeOfKinematicFunction"));

}

KinematicConstraint::~KinematicConstraint() {
}

void KinematicConstraint::initialize() {
  Constraint::initialize();
  dependentBody.initialize();
}

void KinematicConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e, *ee;
  Constraint::initializeUsingXML(element);
  dependentBody.initializeUsingXML(element);
  kinematicFunction.initializeUsingXML(element);
  firstDerivativeOfKinematicFunction.initializeUsingXML(element);
  secondDerivativeOfKinematicFunction.initializeUsingXML(element);
}

TiXmlElement* KinematicConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Constraint::writeXMLFile(parent);

  dependentBody.writeXMLFile(ele0);
  kinematicFunction.writeXMLFile(ele0);
  firstDerivativeOfKinematicFunction.writeXMLFile(ele0);
  secondDerivativeOfKinematicFunction.writeXMLFile(ele0);

  return ele0;
}

JointConstraint::JointConstraint(const string &str, Element *parent) : Constraint(str, parent), force(0,false), moment(0,false) {

  independentBody.setProperty(new RigidBodyOfReferenceProperty(0,this,MBSIMNS"independentRigidBody"));

  DependenciesProperty *dependentBodiesFirstSide_ = new DependenciesProperty(this, MBSIMNS"dependentRigidBodiesFirstSide");
  dependentBodiesFirstSide.setProperty(dependentBodiesFirstSide_);

  DependenciesProperty *dependentBodiesSecondSide_ = new DependenciesProperty(this, MBSIMNS"dependentRigidBodiesSecondSide");
  dependentBodiesSecondSide.setProperty(dependentBodiesSecondSide_);

  connections.setProperty(new ConnectFramesProperty(2,this));

  force.setProperty(new GeneralizedForceDirectionProperty(MBSIMNS"forceDirection"));

  moment.setProperty(new GeneralizedForceDirectionProperty(MBSIMNS"momentDirection"));
}

JointConstraint::~JointConstraint() {
}

void JointConstraint::initialize() {
  Constraint::initialize();
  independentBody.initialize();
  dependentBodiesFirstSide.initialize();
  dependentBodiesSecondSide.initialize();
  connections.initialize();
}

void JointConstraint::initializeUsingXML(TiXmlElement *element) {
  Constraint::initializeUsingXML(element);
  dependentBodiesFirstSide.initializeUsingXML(element);
  dependentBodiesSecondSide.initializeUsingXML(element);
  independentBody.initializeUsingXML(element);

  force.initializeUsingXML(element);
  moment.initializeUsingXML(element);
  connections.initializeUsingXML(element);
}

TiXmlElement* JointConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Constraint::writeXMLFile(parent);

  dependentBodiesFirstSide.writeXMLFile(ele0);
  dependentBodiesSecondSide.writeXMLFile(ele0);

  independentBody.writeXMLFile(ele0);

  force.writeXMLFile(ele0);
  moment.writeXMLFile(ele0);

  connections.writeXMLFile(ele0);

  return ele0;
}
