#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/line.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/load.h"
#include "mbsim/environment.h"

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
  body->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

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
  body3->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body3->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));



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
  CircleSolid *circlecontour=new CircleSolid("Circle",d);
  body->addContour(circlecontour,Vec(3),SqrMat(3,EYE));
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour->enableOpenMBV();
#endif

  // Contour of ObstacleCylinder
  CircleSolid *circlecontour2=new CircleSolid("Circle2",d);
  body2->addContour(circlecontour2,Vec(3),SqrMat(3,EYE));
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour2->enableOpenMBV();
#endif

  // Contour of HollowCylinder (outward)
  CircleHollow *circlecontour3=new CircleHollow("Circle3",2.5*d);
  body3->addContour(circlecontour3,Vec(3),SqrMat(3,EYE));
#ifdef HAVE_OPENMBVCPPINTERFACE
  circlecontour3->enableOpenMBV();
#endif

  // Contour of HollowCylinder (inward)
  CircleSolid *circlecontour4=new CircleSolid("Circle4",3*d);
  body3->addContour(circlecontour4,Vec(3),SqrMat(3,EYE));
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
  addContour(new Line("Line"),Vec(3),A);


  // Contact between CylinderHollow and ObstacleCylinder
  Contact *rc2 = new Contact("Contact2");
  rc2->connect(body3->getContour("Circle4"), body2->getContour("Circle2"));
  addLink(rc2);
  if(rigidContact) {
    rc2->setContactForceLaw(new UnilateralConstraint);
    rc2->setContactImpactLaw(new UnilateralNewtonImpact);
    rc2->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc2->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc2->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc2->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between InnerCylinder and CylinderHollow
  Contact *rc3 = new Contact("Contact3");
  rc3->connect(body->getContour("Circle"), body3->getContour("Circle3"));
  addLink(rc3);
  if(rigidContact) {
    rc3->setContactForceLaw(new UnilateralConstraint);
    rc3->setContactImpactLaw(new UnilateralNewtonImpact);
    rc3->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc3->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc3->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc3->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between HollowCylinder and plane
  Contact *rc4 = new Contact("Contact4");
  rc4->connect(getContour("Line"),body3->getContour("Circle4"));
  addLink(rc4);
  if(rigidContact) {
    rc4->setContactForceLaw(new UnilateralConstraint);
    rc4->setContactImpactLaw(new UnilateralNewtonImpact);
    rc4->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc4->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc4->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc4->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }


#ifdef HAVE_OPENMBVCPPINTERFACE
  body->getFrame("C")->enableOpenMBV(1.5*d);
//  OpenMBV::Frustum* dummy = new OpenMBV::Frustum;
//  dummy->setBaseRadius(d);
//  dummy->setTopRadius(d);
//  dummy->setHeight(d); 
//  dummy->setScaleFactor(1.);
//  dummy->setMinimalColorValue(0);
//  dummy->setMaximalColorValue(1);
//  dummy->setStaticColor(1);
//  body->setOpenMBVRigidBody(dummy);
  body->setOpenMBVRigidBody(new OpenMBV::InvisibleBody());

  body2->getFrame("C")->enableOpenMBV(1.5*d);
//  OpenMBV::Frustum* dummy2 = new OpenMBV::Frustum;
//  dummy2->setBaseRadius(d);
//  dummy2->setTopRadius(d);
//  dummy2->setHeight(d); 
//  dummy2->setScaleFactor(1.);
//  dummy2->setMinimalColorValue(0);
//  dummy2->setMaximalColorValue(1);
//  dummy2->setStaticColor(0.5);
//  body2->setOpenMBVRigidBody(dummy2);
  body2->setOpenMBVRigidBody(new OpenMBV::InvisibleBody());

  body3->getFrame("C")->enableOpenMBV(1.5*d);
//  OpenMBV::Frustum* dummy3 = new OpenMBV::Frustum;
//  dummy3->setBaseRadius(3*d);
//  dummy3->setTopRadius(3*d);
//  dummy3->setInnerBaseRadius(2.5*d);
//  dummy3->setInnerTopRadius(2.5*d);
//  dummy3->setHeight(d); 
//  dummy3->setScaleFactor(1.);
//  dummy3->setMinimalColorValue(0);
//  dummy3->setMaximalColorValue(1);
//  dummy3->setStaticColor(0.1);
//  body3->setOpenMBVRigidBody(dummy3);
  body3->setOpenMBVRigidBody(new OpenMBV::InvisibleBody());

#endif
}

