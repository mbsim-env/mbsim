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
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "dynamic_system_solver.h"
#include "group.h"
#include "rigid_body.h"
#include "flexible_body_ffr.h"
#include "constraint.h"
#include "kinetic_excitation.h"
#include "spring_damper.h"
#include "joint.h"
#include "friction.h"
#include "contact.h"
#include "gear.h"
#include "connection.h"
#include "observer.h"
#include "sensor.h"
//#include "linear_transfer_system.h"
#include "parameter.h"
#include "integrator.h"
#include "analyser.h"
#include <string>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ObjectFactory* ObjectFactory::instance=NULL;

  Environment *ObjectFactory::getEnvironment(DOMElement *element) {
    if(element==NULL) return NULL;
    Environment *obj;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->getEnvironment(element))) return obj;
    cout << string("No Environment of type {")+E(element)->getTagName().first+"}"+E(element)->getTagName().second+" exists.";
    return 0;
  }

  Environment *MBSimObjectFactory::getEnvironment(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"MBSimEnvironment")
      return Environment::getInstance();
    return 0;
  }

  MBSimObjectFactory *MBSimObjectFactory::instance=NULL;

  void MBSimObjectFactory::initialize() {
    if(instance==0) {
      instance=new MBSimObjectFactory;
      ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }

  Frame* ObjectFactory::createFrame(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createFrame(element);
    return 0;
  }
  Frame* MBSimObjectFactory::createFrame(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"FixedRelativeFrame")
      return new FixedRelativeFrame;
    if(E(element)->getTagName()==MBSIMFLEX%"NodeFrame")
      return new NodeFrame;
    return 0;
  }

  Contour* ObjectFactory::createContour(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createContour(element);
    return 0;
  }
  Contour* MBSimObjectFactory::createContour(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"Point")
      return new Point;
    else if(E(element)->getTagName()==MBSIM%"Line")
      return new Line;
    else if(E(element)->getTagName()==MBSIM%"Plane")
      return new Plane;
    else if(E(element)->getTagName()==MBSIM%"Sphere")
      return new Sphere;
    else if(E(element)->getTagName()==MBSIM%"Circle")
      return new Circle;
    else if(E(element)->getTagName()==MBSIM%"Cuboid")
      return new Cuboid;
    else if(E(element)->getTagName()==MBSIM%"LineSegment")
      return new LineSegment;
    else if(E(element)->getTagName()==MBSIM%"PlanarContour")
      return new PlanarContour;
    else if(E(element)->getTagName()==MBSIM%"SpatialContour")
      return new SpatialContour;
    return 0;
  }

  Group* ObjectFactory::createGroup(DOMElement *element) {
    if(element==NULL) return NULL;
    //Group *obj;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (((*i)->createGroup(element)));
    return 0;
  }
  Group* MBSimObjectFactory::createGroup(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"DynamicSystemSolver")
      return new DynamicSystemSolver;
    else if(E(element)->getTagName()==MBSIM%"Group")
      return new Group;
    return 0;
  }

  Object* ObjectFactory::createObject(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createObject(element);
    return 0;
  }
  Object* MBSimObjectFactory::createObject(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"RigidBody")
      return new RigidBody;
    if(E(element)->getTagName()==MBSIMFLEX%"FlexibleBodyFFR")
      return new FlexibleBodyFFR;
    return 0;
  }

  Link* ObjectFactory::createLink(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createLink(element);
    return 0;
  }
  Link* MBSimObjectFactory::createLink(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"KineticExcitation")
      return new KineticExcitation;
    if(E(element)->getTagName()==MBSIM%"SpringDamper")
      return new SpringDamper;
    if(E(element)->getTagName()==MBSIM%"DirectionalSpringDamper")
      return new DirectionalSpringDamper;
    if(E(element)->getTagName()==MBSIM%"Joint")
      return new Joint;
    if(E(element)->getTagName()==MBSIM%"ElasticJoint")
      return new ElasticJoint;
    if(E(element)->getTagName()==MBSIM%"Contact")
      return new Contact;
    if(E(element)->getTagName()==MBSIM%"GeneralizedSpringDamper")
      return new GeneralizedSpringDamper;
    if(E(element)->getTagName()==MBSIM%"GeneralizedElasticConnection")
      return new GeneralizedElasticConnection;
    if(E(element)->getTagName()==MBSIM%"GeneralizedFriction")
      return new GeneralizedFriction;
    if(E(element)->getTagName()==MBSIM%"GeneralizedGear")
      return new GeneralizedGear;
    if(E(element)->getTagName()==MBSIMCONTROL%"GeneralizedPositionSensor")
      return new GeneralizedPositionSensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"GeneralizedVelocitySensor")
      return new GeneralizedVelocitySensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"PositionSensor")
      return new PositionSensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"OrientationSensor")
      return new OrientationSensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"VelocitySensor")
      return new VelocitySensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"AngularVelocitySensor")
      return new AngularVelocitySensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"FunctionSensor")
      return new FunctionSensor;
    if(E(element)->getTagName()==MBSIMCONTROL%"Multiplexer")
      return new Multiplexer;
    if(E(element)->getTagName()==MBSIMCONTROL%"Demultiplexer")
      return new Demultiplexer;
    if(E(element)->getTagName()==MBSIMCONTROL%"LinearTransferSystem")
      return new LinearTransferSystem;
    if(E(element)->getTagName()==MBSIMCONTROL%"SignalOperation")
      return new SignalOperation;
    if(E(element)->getTagName()==MBSIMCONTROL%"ExternSignalSource")
      return new ExternSignalSource;
    if(E(element)->getTagName()==MBSIMCONTROL%"ExternSignalSink")
      return new ExternSignalSink;
    return 0;
  }  

  Constraint* ObjectFactory::createConstraint(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createConstraint(element);
    return 0;
  }
  Constraint* MBSimObjectFactory::createConstraint(DOMElement *element) {
    if(element==0) return 0;
    else if(E(element)->getTagName()==MBSIM%"GeneralizedGearConstraint")
      return new GeneralizedGearConstraint;
    else if(E(element)->getTagName()==MBSIM%"GeneralizedPositionConstraint")
      return new GeneralizedPositionConstraint;
    else if(E(element)->getTagName()==MBSIM%"GeneralizedVelocityConstraint")
      return new GeneralizedVelocityConstraint;
    else if(E(element)->getTagName()==MBSIM%"GeneralizedAccelerationConstraint")
      return new GeneralizedAccelerationConstraint;
    else if(E(element)->getTagName()==MBSIM%"JointConstraint")
      return new JointConstraint;
    else if(E(element)->getTagName()==MBSIM%"GeneralizedConnectionConstraint")
      return new GeneralizedConnectionConstraint;
    return 0;
  }

  Observer* ObjectFactory::createObserver(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createObserver(element);
    return 0;
  }
  Observer* MBSimObjectFactory::createObserver(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"KinematicCoordinatesObserver")
      return new KinematicCoordinatesObserver;
    if(E(element)->getTagName()==MBSIM%"RelativeKinematicsObserver")
      return new RelativeKinematicsObserver;
    if(E(element)->getTagName()==MBSIM%"MechanicalLinkObserver")
      return new MechanicalLinkObserver;
    if(E(element)->getTagName()==MBSIM%"MechanicalConstraintObserver")
      return new MechanicalConstraintObserver;
    if(E(element)->getTagName()==MBSIM%"ContactObserver")
      return new ContactObserver;
    if(E(element)->getTagName()==MBSIM%"FrameObserver")
      return new FrameObserver;
    if(E(element)->getTagName()==MBSIM%"RigidBodyObserver")
      return new RigidBodyObserver;
    if(E(element)->getTagName()==MBSIM%"RigidBodySystemObserver")
      return new RigidBodySystemObserver;
    return 0;
  }  

  Solver* ObjectFactory::createSolver(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createSolver(element);
    return 0;
  }

  Solver* MBSimObjectFactory::createSolver(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIMINT%"DOPRI5Integrator")
      return new DOPRI5Integrator;
    else if(E(element)->getTagName()==MBSIMINT%"RADAU5Integrator")
      return new RADAU5Integrator;
    else if(E(element)->getTagName()==MBSIMINT%"LSODEIntegrator")
      return new LSODEIntegrator;
    else if(E(element)->getTagName()==MBSIMINT%"LSODARIntegrator")
      return new LSODARIntegrator;
    else if(E(element)->getTagName()==MBSIMINT%"TimeSteppingIntegrator")
      return new TimeSteppingIntegrator;
    else if(E(element)->getTagName()==MBSIMINT%"EulerExplicitIntegrator")
      return new EulerExplicitIntegrator;
    else if(E(element)->getTagName()==MBSIMINT%"RKSuiteIntegrator")
      return new RKSuiteIntegrator;
    else if(E(element)->getTagName()==MBSIMANALYSER%"Eigenanalyser")
      return new Eigenanalyser;
    else if(E(element)->getTagName()==MBSIMANALYSER%"HarmonicResponseAnalyser")
      return new HarmonicResponseAnalyser;
    return 0;
  }

  Parameter* ObjectFactory::createParameter(DOMElement *element) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createParameter(element);
    return 0;
  }

  Parameter* MBSimObjectFactory::createParameter(DOMElement *element) {
    if(element==0) return 0;
    if(E(element)->getTagName()==PV%"stringParameter")
      return new StringParameter;
    else if(E(element)->getTagName()==PV%"scalarParameter")
      return new ScalarParameter;
    else if(E(element)->getTagName()==PV%"vectorParameter")
      return new VectorParameter;
    else if(E(element)->getTagName()==PV%"matrixParameter")
      return new MatrixParameter;
    else if(E(element)->getTagName()==PV%"import")
      return new ImportParameter;
    return 0;
  }

}
