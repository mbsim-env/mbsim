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
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

extern DOMLSParser *parser;

namespace MBSimGUI {

//  Property* RigidBodyOfReferencePropertyFactory::createProperty(int i) {
//    return new RigidBodyOfReferenceProperty("",element,xmlName);
//  }
//
//  Property* GeneralizedGearConstraintPropertyFactory::createProperty(int i) {
//    return new GearInputReferenceProperty("",element,xmlName);
//  }

  Constraint* Constraint::readXMLFile(const string &filename) {
    shared_ptr<DOMDocument> doc(parser->parseURI(X()%filename));
    DOMElement *e=doc->getDocumentElement();
    Constraint *constraint=Embed<Constraint>::createAndInit(e);
    return constraint;
  }

  GeneralizedConstraint::GeneralizedConstraint(const string &str) : MechanicalConstraint(str) {
//    support.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"supportFrame"));
  }

  GeneralizedGearConstraint::GeneralizedGearConstraint(const string &str) : GeneralizedConstraint(str) {

//    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));
//
//    independentBodies.setProperty(new ListProperty(new GeneralizedGearConstraintPropertyFactory(this),MBSIM%"independentRigidBody"));
  }

  GeneralizedDualConstraint::GeneralizedDualConstraint(const string &str) : GeneralizedConstraint(str) {

//    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));
//
//    independentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"independentRigidBody"));
  }

//  void GeneralizedDualConstraint::initialize() {
//    GeneralizedConstraint::initialize();
//    dependentBody.initialize();
//    independentBody.initialize();
//    RigidBody *body = static_cast<RigidBodyOfReferenceProperty*>(dependentBody.getProperty())->getBodyPtr();
//    if(body)
//      body->setConstrained(true);
//  }

  GeneralizedPositionConstraint::GeneralizedPositionConstraint(const string &str) : GeneralizedDualConstraint(str) {

//    constraintFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"constraintFunction"));
  }

  GeneralizedVelocityConstraint::GeneralizedVelocityConstraint(const string &str) : GeneralizedDualConstraint(str) {

//    constraintFunction.setProperty(new ChoiceProperty2(new ConstraintPropertyFactory(this),"",3));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialState"));
//    x0.setProperty(new ExtPhysicalVarProperty(input));
  }

  GeneralizedAccelerationConstraint::GeneralizedAccelerationConstraint(const string &str) : GeneralizedDualConstraint(str) {

//    constraintFunction.setProperty(new ChoiceProperty2(new ConstraintPropertyFactory(this),"",3));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialState"));
//    x0.setProperty(new ExtPhysicalVarProperty(input));
  }

  JointConstraint::JointConstraint(const string &str) : MechanicalConstraint(str) {

//    dependentBodiesFirstSide.setProperty(new ListProperty(new RigidBodyOfReferencePropertyFactory(this,""),MBSIM%"dependentRigidBodyOnFirstSide"));
//
//    dependentBodiesSecondSide.setProperty(new ListProperty(new RigidBodyOfReferencePropertyFactory(this,""),MBSIM%"dependentRigidBodyOnSecondSide"));
//
//    independentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"independentRigidBody"));
//
//    connections.setProperty(new ConnectFramesProperty(2,this));
//
//    refFrameID.setProperty(new IntegerProperty(0,MBSIM%"frameOfReferenceID"));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"forceDirection"));
//    force.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"momentDirection"));
//    moment.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new VecProperty(0),"",MBSIM%"initialGuess"));
//    q0.setProperty(new ExtPhysicalVarProperty(input));
  }

}
