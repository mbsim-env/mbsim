#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/load.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
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
  double m = 0.1;
  double r = 0.1;
  Vec q0(6);
  q0(0) = 0;
  q0(1) = 2.2*r;
  q0(2) = 0;
  body->setInitialGeneralizedPosition(q0);
  //body->setu0("[0;0;0;0;0;-30]");
  body->setInitialGeneralizedVelocity("[3;0;1;0;0;0]");
  //body->setu0("[0;0;0;20;0;-30]");
  //body->setu0("[0;0;0;20;10;-30]");
  //body->setu0("[3;0;0;4;-3;2]");
  //body->setu0("[0;0;0;20;10;-30]");
  body->setMass(m);
  SymMat Theta(3);
  Theta(0,0) =  2./5.*m*r*r;
  Theta(1,1) =  2./5.*m*r*r;
  Theta(2,2) =  2./5.*m*r*r;
  body->setInertiaTensor(Theta);
  Sphere *sphere = new Sphere("Sphere");
  sphere->setRadius(r);
#ifdef HAVE_OPENMBVCPPINTERFACE
  sphere->enableOpenMBV();
#endif
  body->addContour(sphere,Vec(3),SqrMat(3,EYE));

  Contact *cnf = new Contact("Contact");
  cnf->setContactForceLaw(new UnilateralConstraint);
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
  //cnf->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e-6,100));
  //cnf->setFrictionForceLaw(new LinearRegularizedSpatialCoulombFriction(0.3));
  cnf->setFrictionForceLaw(new SpatialCoulombFriction(0.1));
  cnf->setFrictionImpactLaw(new SpatialCoulombImpact(0.1));
  cnf->connect(getContour("Plane"), body->getContour("Sphere"));
  // cnf->setFrictionCoefficient(0.3);
  //cnf->setPlotLevel(2);
  addLink(cnf);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::InvisibleBody *obj=new OpenMBV::InvisibleBody;
  body->setOpenMBVRigidBody(obj);
  body->getFrame("C")->enableOpenMBV(2*r*1.2,0);
#endif
}

