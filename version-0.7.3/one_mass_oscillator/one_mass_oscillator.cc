#include "one_mass_oscillator.h"
#include "body_rigid_abs.h"
#include "coilspring.h"
#include "connection_flexible.h"
#include "contact_flexible.h"
#include "sphere.h"
#include "contour.h"
#include "userfunction.h"

using namespace AMVis;

OneMassOscillator::OneMassOscillator(const string &projectName) : MultiBodySystem(projectName) {
  // inertial Port
  this->addPort("I",Vec(3));

  // single mass
  BodyRigidAbs *ball = new BodyRigidAbs("Ball");
  ball->setPlotLevel(3);
  addObject(ball);
  ball->setMass(1);
  ball->setJT(Vec("[1;0;0]"));
  ball->addPort("COG",Vec(3));
  ball->setq0(Vec("[1]"));

  // connetion
  ConnectionFlexible *cf = new ConnectionFlexible("Connection");
  cf->setForceDirection(Vec("[1;0;0]"));
  cf->connect(ball->getPort("COG"),this->getPort("I"));
  cf->setTranslationalStiffness(1);
  cf->setTranslationalDamping(0.25*0.0);
  cf->setPlotLevel(3);
  addLink(cf);

  // contact
  Plane *plane = new Plane("Plane");
  plane->setCn(Vec("[-1;0;0]"));
  addContour(plane,Vec(3));

  Point *point = new Point("Point");
  ball->addContour(point,Vec("[-0.1;0;0]"));
  
  FuncQuadratic *potential = new FuncQuadratic(Vec(1,INIT,1.e6),Vec(1,INIT,0.),Vec(1,INIT,0.));

  ContactFlexible *contact = new ContactFlexible("Contact");
  contact->connect(plane,point);
  contact->setPotential(potential);
  //contact->setStiffness(1.e6);
  addLink(contact);

  // visualisation with AMVis
  // ball
  AMVis::Sphere *sp = new AMVis::Sphere(ball->getFullName(),1,true);
  sp->setRadius(0.1);
  ball->setAMVisBody(sp);

  // spring 
  AMVis::CoilSpring *FederVis = new AMVis::CoilSpring(cf->getFullName(),1,false);
  FederVis->setRadius(0.05);
  FederVis->setRadiusCrossSection(0.002);
  FederVis->setNumberOfCoils(10);
  FederVis->setColor(0.5);
  cf->setAMVisSpring(FederVis);
}

