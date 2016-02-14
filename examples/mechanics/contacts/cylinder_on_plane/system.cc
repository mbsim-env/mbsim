#include "system.h"
#include "mbsim/fixed_relative_frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/line.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#include "openmbvcppinterface/frustum.h"
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
  // Parameters
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*d*d/2.;
  Theta(2,2) = Theta(1,1);
  double mu  = 0.3;

  // Cylinder rolling in hollow cylinder (body 3)
  RigidBody* body = new RigidBody("InnerCylinder");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new TranslationAlongAxesXY<VecV>);
  body->setRotation(new RotationAboutZAxis<VecV>);

  // Fixed cylinder as obstacle 
  RigidBody* body2 = new RigidBody("ObstacleCylinder");
  addObject(body2);

  body2->setFrameOfReference(getFrame("I"));
  body2->setFrameForKinematics(body2->getFrame("C"));
  body2->setMass(m);
  body2->setInertiaTensor(Theta);

  // Hollow cylinder rolling down a plane
  RigidBody* body3 = new RigidBody("CylinderHollow");
  addObject(body3);

  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new TranslationAlongAxesXY<VecV>);
  body3->setRotation(new RotationAboutZAxis<VecV>);

  // Initial translation and rotation
  Vec q0(3);
  q0(0) = -0.1;
  q0(1) = .7;
  body->setInitialGeneralizedPosition(q0);
  q0(1) = .6;
  body3->setInitialGeneralizedPosition(q0);
  Vec u0(3);
  u0(2) = -M_PI*10;
  body->setInitialGeneralizedVelocity(u0);


  // Contour of InnerCylinder
  Circle *circlecontour=new Circle("Circle",d);
  body->addContour(circlecontour);
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour->enableOpenMBV();
#endif

  // Contour of ObstacleCylinder
  Circle *circlecontour2=new Circle("Circle2",d);
  body2->addContour(circlecontour2);
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour2->enableOpenMBV();
#endif

  // Contour of HollowCylinder (outward)
  Circle *circlecontour3=new Circle("Circle3",2.5*d,false);
  body3->addContour(circlecontour3);
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour3->enableOpenMBV();
#endif

  // Contour of SolidCylinder (inward)
  Circle *circlecontour4=new Circle("Circle4",3*d);
  body3->addContour(circlecontour4);
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour4->enableOpenMBV();
#endif

  // Contour of ground plane
  double phi = M_PI*0.6; 
  SqrMat A(3);
  A(0,0) = cos(phi);
  A(0,1) = -sin(phi);
  A(1,1) = cos(phi);
  A(1,0) = sin(phi);
  A(2,2) = 1;
  addFrame(new FixedRelativeFrame("Line",Vec(3),A));
  addContour(new Line("Line",getFrame("Line")));

  // Contact between CylinderHollow and ObstacleCylinder
  Contact *rc2 = new Contact("Contact2");
  rc2->connect(body3->getContour("Circle4"), body2->getContour("Circle2"));
  addLink(rc2);
  if(rigidContact) {
    rc2->setNormalForceLaw(new UnilateralConstraint);
    rc2->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc2->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc2->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc2->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc2->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between InnerCylinder and CylinderHollow
  Contact *rc3 = new Contact("Contact3");
  rc3->connect(body->getContour("Circle"), body3->getContour("Circle3"));
  addLink(rc3);
  if(rigidContact) {
    rc3->setNormalForceLaw(new UnilateralConstraint);
    rc3->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc3->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc3->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc3->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc3->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between HollowCylinder and plane
  Contact *rc4 = new Contact("Contact4");
  rc4->connect(getContour("Line"),body3->getContour("Circle4"));
  addLink(rc4);
  if(rigidContact) {
    rc4->setNormalForceLaw(new UnilateralConstraint);
    rc4->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc4->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc4->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc4->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc4->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }


#ifdef HAVE_OPENMBVCPPINTERFACE
  body->getFrame("C")->enableOpenMBV(1.5*d);
  body->setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());

  body2->getFrame("C")->enableOpenMBV(1.5*d);
  body2->setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());

  body3->getFrame("C")->enableOpenMBV(1.5*d);
  body3->setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());

#endif
}

