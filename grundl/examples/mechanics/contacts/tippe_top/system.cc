#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/sphere.h"
#include "openmbvcppinterface/invisiblebody.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

extern bool rigidContact;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

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
  body->setInitialGeneralizedPosition(q0);
  body->setInitialGeneralizedVelocity("[0;0;0;0;180;0]");
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
#ifdef HAVE_OPENMBVCPPINTERFACE
  sphere->enableOpenMBV();
#endif
  body->addContour(sphere,rSM,SqrMat(3,EYE));
  sphere = new Sphere("Sphere2");
  sphere->setRadius(r2);
  rSM(1) = a2;
#ifdef HAVE_OPENMBVCPPINTERFACE
  sphere->enableOpenMBV();
  body->getFrame("C")->enableOpenMBV(2*1.2*r1,0);
#endif
  body->addContour(sphere,rSM,SqrMat(3,EYE));

  double mu = 0.2;

  Contact *cnf1 = new Contact("Contact1");
  cnf1->connect(getContour("Plane"), body->getContour("Sphere1"));
  addLink(cnf1);

  Contact *cnf2 = new Contact("Contact2");
  cnf2->connect(getContour("Plane"), body->getContour("Sphere2"));
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
    cnf1->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e3)));
    cnf1->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.3)));
    cnf2->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e3)));
    cnf2->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.3)));
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::InvisibleBody *obj1=new OpenMBV::InvisibleBody;
  body->setOpenMBVRigidBody(obj1);
#endif

}

