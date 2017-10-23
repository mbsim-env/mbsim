#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/links/contact.h"
#include "mbsim/environment.h"

#include "openmbvcppinterface/sphere.h"
#include "openmbvcppinterface/invisiblebody.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

extern bool rigidContact;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double phi = M_PI/2;
  SqrMat AWK(3);
  AWK(0,0) = cos(phi);
  AWK(0,1) = -sin(phi);
  AWK(1,1) = cos(phi);
  AWK(1,0) = sin(phi);
  AWK(2,2) = 1;
  addFrame(new FixedRelativeFrame("P",Vec(3),AWK));
  addContour(new Plane("Plane",getFrame("P")));

  RigidBody* body = new RigidBody("Body");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));

  Mat J("[1,0,0;0,1,0;0,0,1]");
  body->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  body->setRotation(new RotationAboutAxesXYZ<VecV>);
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
  body->setGeneralizedInitialPosition(q0);
  body->setGeneralizedInitialVelocity("[0;0;0;0;180;0]");
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
  sphere->enableOpenMBV();
  body->addFrame(new FixedRelativeFrame("S1",rSM,SqrMat(3,EYE)));
  sphere->setFrameOfReference(body->getFrame("S1"));
  body->addContour(sphere);
  sphere = new Sphere("Sphere2");
  sphere->setRadius(r2);
  rSM(1) = a2;
  sphere->enableOpenMBV();
  body->getFrame("C")->enableOpenMBV(2*1.2*r1,0);
  body->addFrame(new FixedRelativeFrame("S2",rSM,SqrMat(3,EYE)));
  sphere->setFrameOfReference(body->getFrame("S2"));
  body->addContour(sphere);

  double mu = 0.2;

  Contact *cnf1 = new Contact("Contact1");
  cnf1->connect(getContour("Plane"), body->getContour("Sphere1"));
  addLink(cnf1);

  Contact *cnf2 = new Contact("Contact2");
  cnf2->connect(getContour("Plane"), body->getContour("Sphere2"));
  addLink(cnf2);

  if(rigidContact) {
    cnf1->setNormalForceLaw(new UnilateralConstraint);
    cnf1->setNormalImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf1->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    cnf1->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
    cnf2->setNormalForceLaw(new UnilateralConstraint);
    cnf2->setNormalImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf2->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    cnf2->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    cnf1->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e3)));
    cnf1->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.3)));
    cnf2->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e3)));
    cnf2->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.3)));
  }

  std::shared_ptr<OpenMBV::InvisibleBody> obj1=OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>();
  body->setOpenMBVRigidBody(obj1);

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
}
