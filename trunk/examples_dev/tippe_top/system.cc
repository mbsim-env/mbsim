#include "system.h"
#include "rigid_body.h"
#include "objobject.h"
#include "contour.h"
#include "constitutive_laws.h"
#include "contact.h"
#include "load.h"
#include "cube.h"
#include "sphere.h"
#include "compoundprimitivebody.h"

extern bool rigidContact;

System::System(const string &projectName) : MultiBodySystem(projectName) {
 // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);

  addFrame("Os",Vec(3),SqrMat(3,EYE));

  Plane *plane = new Plane("Plane");
  double phi = M_PI/2;
  SqrMat AWK(3);
  AWK(0,0) = cos(phi);
  AWK(0,1) = -sin(phi);
  AWK(1,1) = cos(phi);
  AWK(1,0) = sin(phi);
  AWK(2,2) = 1;
  addContour(plane,Vec(3),AWK);

  RigidBody* body = new RigidBody("Body");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));

  Mat J("[1,0,0;0,1,0;0,0,1]");
  body->setTranslation(new LinearTranslation(J));
  body->setRotation(new CardanAngles);
  double m = 6e-3;
  double r1 = 1.5e-2;
  double r2 = 0.5e-2;
  double a1 = 0.3e-2;
  double a2 = 1.6e-2;
  Vec q0(6);
  q0(0) = 0;
  q0(1) = 1.2015e-2;
  q0(2) = 0;
  q0(3) = 0.1;
  body->setq0(q0);
  body->setu0("[0;0;0;0;180;0]");
  body->setMass(m);
  SymMat Theta(3);
  Theta(0,0) =  8e-7;
  Theta(1,1) =  7e-7;
  Theta(2,2) =  8e-7;
  body->setInertiaTensor(Theta);
  Sphere *sphere = new Sphere("Sphere1");
  sphere->setRadius(r1);
  Vec rSM(3);
  rSM(1) = a1;
  body->addContour(sphere,rSM,SqrMat(3,EYE));
  sphere = new Sphere("Sphere2");
  sphere->setRadius(r2);
  rSM(1) = a2;
  body->addContour(sphere,rSM,SqrMat(3,EYE));

  double mu = 0.2;

  Contact *cnf1 = new Contact("Contact1");
  cnf1->connect(getContour("Plane"), body->getContour("Sphere1"));
  //cnf1->setPlotLevel(2);
  addLink(cnf1);

  Contact *cnf2 = new Contact("Contact2");
  cnf2->connect(getContour("Plane"), body->getContour("Sphere2"));
  //cnf2->setPlotLevel(2);
  addLink(cnf2);

  if(rigidContact) {
    cnf1->setContactForceLaw(new UnilateralConstraint);
    cnf1->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf1->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    cnf1->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
    cnf2->setContactForceLaw(new UnilateralConstraint);
    cnf2->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf2->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    cnf2->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    cnf1->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e3));
    cnf1->setFrictionForceLaw(new LinearRegularizedSpatialCoulombFriction(0.3));
    cnf2->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e3));
    cnf2->setFrictionForceLaw(new LinearRegularizedSpatialCoulombFriction(0.3));
  }

  AMVis::Sphere *obj1 = new AMVis::Sphere();
  body->setAMVisBody(obj1);
  obj1->setRadius(r1);
  obj1-> setInitialTranslation(0,a1,0);
  obj1 -> setInitialRotation(0,0,0);
  AMVis::Sphere *obj2 = new AMVis::Sphere();
  body->setAMVisBody(obj2);
  obj2->setRadius(r2);
  obj2-> setInitialTranslation(0,a2,0);
  obj2 -> setInitialRotation(0,0,0);

  AMVis::CompoundPrimitiveBody *obj = new AMVis::CompoundPrimitiveBody(body->getName(),1,false);
  obj ->setColor(0.5);
  obj ->addBody(obj1);
  obj ->addBody(obj2);
  body->setAMVisBody(obj); 


}

