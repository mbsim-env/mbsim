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
#include "linear_transfer_system.h"
#include "kinetic_excitation.h"
#include "joint.h"
#include "spring_damper.h"
#include "contact.h"
#include "sensor.h"
#include "widget.h"
#include "parameter.h"
#include "observer.h"
#include "integrator.h"
#include "analyser.h"
#include "friction.h"
#include "gear.h"
#include "connection.h"
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

  Frame* ObjectFactory::createFrame(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createFrame(element,parent);
    return 0;
  }
  Frame* MBSimObjectFactory::createFrame(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"FixedRelativeFrame")
      return new FixedRelativeFrame(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMFLEX%"FixedNodalFrame")
      return new FixedNodalFrame(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMFLEX%"NodeFrame")
      return new NodeFrame(E(element)->getAttribute("name"),parent);
    return 0;
  }

  Contour* ObjectFactory::createContour(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createContour(element,parent);
    return 0;
  }
  Contour* MBSimObjectFactory::createContour(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"Point")
      return new Point(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"Line")
      return new Line(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"Plane")
      return new Plane(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"Sphere")
      return new Sphere(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"Circle")
      return new Circle(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"Cuboid")
      return new Cuboid(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"LineSegment")
      return new LineSegment(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"PlanarContour")
      return new PlanarContour(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"SpatialContour")
      return new SpatialContour(E(element)->getAttribute("name"),parent);
    return 0;
  }

  Group* ObjectFactory::createGroup(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    //Group *obj;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (((*i)->createGroup(element,parent)));
    return 0;
  }
  Group* MBSimObjectFactory::createGroup(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"DynamicSystemSolver")
      return new DynamicSystemSolver(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"Group")
      return new Group(E(element)->getAttribute("name"),parent);
    return 0;
  }

  Object* ObjectFactory::createObject(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createObject(element,parent);
    return 0;
  }
  Object* MBSimObjectFactory::createObject(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"RigidBody")
      return new RigidBody(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMFLEX%"FlexibleBodyFFR")
      return new FlexibleBodyFFR(E(element)->getAttribute("name"),parent);
    return 0;
  }

  Link* ObjectFactory::createLink(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createLink(element,parent);
    return 0;
  }
  Link* MBSimObjectFactory::createLink(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"KineticExcitation")
      return new KineticExcitation(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"SpringDamper")
      return new SpringDamper(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"DirectionalSpringDamper")
      return new DirectionalSpringDamper(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"GeneralizedSpringDamper")
      return new GeneralizedSpringDamper(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"GeneralizedFriction")
      return new GeneralizedFriction(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"GeneralizedGear")
      return new GeneralizedGear(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"Joint")
      return new Joint(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"ElasticJoint")
      return new ElasticJoint(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"Contact")
      return new Contact(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"GeneralizedElasticConnection")
      return new GeneralizedElasticConnection(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"GeneralizedPositionSensor")
      return new GeneralizedPositionSensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"GeneralizedVelocitySensor")
      return new GeneralizedVelocitySensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"AbsolutePositionSensor")
      return new AbsolutePositionSensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"AbsoluteVelocitySensor")
      return new AbsoluteVelocitySensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"AbsoluteAngularPositionSensor")
      return new AbsoluteAngularPositionSensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"AbsoluteAngularVelocitySensor")
      return new AbsoluteAngularVelocitySensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"FunctionSensor")
      return new FunctionSensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"SignalProcessingSystemSensor")
      return new SignalProcessingSystemSensor(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"PIDController")
      return new PIDController(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"UnarySignalOperation")
      return new UnarySignalOperation(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"BinarySignalOperation")
      return new BinarySignalOperation(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"LinearTransferSystem")
      return new LinearTransferSystem(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"ExternSignalSource")
      return new ExternSignalSource(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIMCONTROL%"ExternSignalSink")
      return new ExternSignalSink(E(element)->getAttribute("name"),parent);
    return 0;
  }  

  Constraint* ObjectFactory::createConstraint(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createConstraint(element,parent);
    return 0;
  }
  Constraint* MBSimObjectFactory::createConstraint(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    else if(E(element)->getTagName()==MBSIM%"GeneralizedGearConstraint")
      return new GeneralizedGearConstraint(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"GeneralizedPositionConstraint")
      return new GeneralizedPositionConstraint(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"GeneralizedVelocityConstraint")
      return new GeneralizedVelocityConstraint(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"GeneralizedAccelerationConstraint")
      return new GeneralizedAccelerationConstraint(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"JointConstraint")
      return new JointConstraint(E(element)->getAttribute("name"),parent);
    else if(E(element)->getTagName()==MBSIM%"GeneralizedConnectionConstraint")
      return new GeneralizedConnectionConstraint(E(element)->getAttribute("name"),parent);
    return 0;
  }

  Observer* ObjectFactory::createObserver(DOMElement *element, Element *parent) {
    if(element==NULL) return NULL;
    for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
      return (*i)->createObserver(element,parent);
    return 0;
  }
  Observer* MBSimObjectFactory::createObserver(DOMElement *element, Element *parent) {
    if(element==0) return 0;
    if(E(element)->getTagName()==MBSIM%"CartesianCoordinatesObserver")
      return new CartesianCoordinatesObserver(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"CylinderCoordinatesObserver")
      return new CylinderCoordinatesObserver(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"NaturalCoordinatesObserver")
      return new NaturalCoordinatesObserver(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"AbsoluteKinematicsObserver")
      return new AbsoluteKinematicsObserver(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"RelativeKinematicsObserver")
      return new RelativeKinematicsObserver(E(element)->getAttribute("name"),parent);
    if(E(element)->getTagName()==MBSIM%"MechanicalLinkObserver")
      return new MechanicalLinkObserver(E(element)->getAttribute("name"),parent);
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
      return new StringParameter(E(element)->getAttribute("name"));
    else if(E(element)->getTagName()==PV%"scalarParameter")
      return new ScalarParameter(E(element)->getAttribute("name"));
    else if(E(element)->getTagName()==PV%"vectorParameter")
      return new VectorParameter(E(element)->getAttribute("name"));
    else if(E(element)->getTagName()==PV%"matrixParameter")
      return new MatrixParameter(E(element)->getAttribute("name"));
    else if(E(element)->getTagName()==PV%"import")
      return new ImportParameter;
    return 0;
  }

}
