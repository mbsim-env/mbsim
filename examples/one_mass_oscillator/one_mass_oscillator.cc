#include "one_mass_oscillator.h"
#include "body_rigid_abs.h"
#include "coilspring.h"
#include "connection_flexible.h"
#include "sphere.h"

using namespace AMVis;

OneMassOscillator::OneMassOscillator(const string &projectName) : MultiBodySystem(projectName) {
  // Gravitation
  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  setGrav(grav);

  // innertial Port
  this->addPort("I"  ,Vec(3));

  // single mass
  BodyRigidAbs *ball = new BodyRigidAbs("Ball");
  ball->setPlotLevel(3);
  addObject(ball);
  ball->setMass(1);
  ball->setJT(Vec("[1;0;0]"));
  ball->addPort("COG",Vec(3));
  // Initial translation
  ball->setq0(Vec("[1]"));

  // Connetion
  ConnectionFlexible *cf = new ConnectionFlexible("Connection");
  cf->setForceDirection(Vec("[1;0;0]"));
  cf->connect(ball->getPort("COG"),this->getPort("I"));
  cf->setTranslationalStiffness(1);
  cf->setTranslationalDamping(0.25*0.0);
  cf->setPlotLevel(3);
  addLink(cf);

  // Visualisation with AMVis
  // Ball
  AMVis::Sphere *sp = new AMVis::Sphere(ball->getFullName(),1,true);
  sp->setRadius(0.1);
  ball->setAMVisBody(sp);

  // Spring 
  AMVis::CoilSpring *FederVis = new AMVis::CoilSpring(cf->getFullName(),1,false);
  FederVis->setRadius(0.05);
  FederVis->setRadiusCrossSection(0.002);
  FederVis->setNumberOfCoils(10);
  FederVis->setColor(0.5);
  cf->setAMVisSpring(FederVis);
}

