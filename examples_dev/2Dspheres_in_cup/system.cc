#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/load.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/frustum2d.h"
//#include "mbsim/contour.h"

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
  setAccelerationOfGravity(grav);
  // Parameters
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*d*d/2.;
  Theta(2,2) = Theta(1,1);
  double mu  = 0.3;
  bool out = false;

  // First Body
  RigidBody* body1 = new RigidBody("Body1");
  addObject(body1);

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("C"));
  body1->setMass(m);
  body1->setInertiaTensor(Theta);
  body1->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  // Second Body
  RigidBody* body2 = new RigidBody("Body2");
  addObject(body2);

  body2->setFrameOfReference(getFrame("I"));
  body2->setFrameForKinematics(body2->getFrame("C"));
  body2->setMass(m);
  body2->setInertiaTensor(Theta);
  body2->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  // Third Body
  RigidBody* body3 = new RigidBody("Body3");
  addObject(body3);

  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body3->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  // Fourth Body
  RigidBody* body4 = new RigidBody("Body4");
  addObject(body4);

  body4->setFrameOfReference(getFrame("I"));
  body4->setFrameForKinematics(body4->getFrame("C"));
  body4->setMass(m);
  body4->setInertiaTensor(Theta);
  body4->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body4->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  // Cup
  RigidBody* cup = new RigidBody("Cup");
  addObject(cup);

  cup->setFrameOfReference(getFrame("I"));
  cup->setFrameForKinematics(cup->getFrame("C"));
  cup->setMass(50*m);
  cup->setInertiaTensor(Theta);

  // Initial translation and rotation
  Vec q0(3), q1(1);
  q0(0) = 0.2;
  if(out==true)
    q0(0) = 0.;
  q0(1) = 1.7;
  body1->setInitialGeneralizedPosition(q0);
  q0(1) = .6;
  Vec u0(3);
  u0(1) = -2.;
  u0(2) = M_PI*20;
  body1->setInitialGeneralizedVelocity(u0);
  u0(2) = M_PI*20;
  body2->setInitialGeneralizedVelocity(u0);
  u0(2) = -M_PI*20;
  body3->setInitialGeneralizedVelocity(u0);
  q0(0) = 1.0;
  body2->setInitialGeneralizedPosition(q0);
  q0(0) = -1.0;
  body3->setInitialGeneralizedPosition(q0);
  q0(0) = 0.6;
  q0(1) = 0.1;
  body4->setInitialGeneralizedPosition(q0);
  u0(2) = M_PI*40;
  body4->setInitialGeneralizedVelocity(u0);


  // Contour of Body1
  CircleSolid *circlecontour1 = new CircleSolid("Circle1",2*d);
  body1->addContour(circlecontour1,Vec(3),SqrMat(3,EYE));

  // Contour of Body2
  CircleSolid *circlecontour2 = new CircleSolid("Circle2",d);
  body2->addContour(circlecontour2,Vec(3),SqrMat(3,EYE));

  // Contour of Body3
  CircleSolid *circlecontour3 = new CircleSolid("Circle3",d);
  body3->addContour(circlecontour3,Vec(3),SqrMat(3,EYE));

  // Contour of Body4
  CircleSolid *circlecontour4 = new CircleSolid("Circle4",d);
  body4->addContour(circlecontour4,Vec(3),SqrMat(3,EYE));

  // Contour of ground plane
  SqrMat A(3);
  A(2,2) = 1;
  A(1,0) = sin(M_PI/2);
  A(0,0) = cos(M_PI/2);
  A(0,1) = -sin(M_PI/2);
  A(1,1) = cos(M_PI/2);

  addContour(new Line("Line"),Vec(3),A);

  // Contour of Frustum
  Vec radii(2);
  radii(0) = 3*d, radii(1) = 5*d;
  if(out==true)
    radii(0) = 5*d, radii(1) = 3*d;
  Frustum2D* frustumcontour = new Frustum2D("Frustum");
  frustumcontour->setRadii(radii);
  frustumcontour->setHeight(5*d);
  frustumcontour->enableOpenMBV();
  cup->addContour(frustumcontour,Vec(3,INIT,0.),SqrMat(3,EYE),cup->getFrame("C"));


  // Contact between Body1 and plane
  Contact *rc1 = new Contact("Contact1");
  rc1->connect(getContour("Line"),body1->getContour("Circle1"));
  addLink(rc1);
  if(rigidContact) {
    rc1->setContactForceLaw(new UnilateralConstraint);
    rc1->setContactImpactLaw(new UnilateralNewtonImpact);
    rc1->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc1->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc1->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc1->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }


  // Contact between Body2 and plane
  Contact *rc2 = new Contact("Contact2");
  rc2->connect(getContour("Line"),body2->getContour("Circle2"));
  addLink(rc2);
  if(rigidContact) {
    rc2->setContactForceLaw(new UnilateralConstraint);
    rc2->setContactImpactLaw(new UnilateralNewtonImpact);
    rc2->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc2->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc2->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc2->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }

  // Contact between Body3 and plane
  Contact *rc3 = new Contact("Contact3");
  rc3->connect(getContour("Line"),body3->getContour("Circle3"));
  addLink(rc3);
  if(rigidContact) {
    rc3->setContactForceLaw(new UnilateralConstraint);
    rc3->setContactImpactLaw(new UnilateralNewtonImpact);
    rc3->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc3->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc3->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc3->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }

  // Contact between Body4 and plane
  Contact *rc4 = new Contact("Contact4");
  rc4->connect(getContour("Line"),body4->getContour("Circle4"));
  addLink(rc4);
  if(rigidContact) {
    rc4->setContactForceLaw(new UnilateralConstraint);
    rc4->setContactImpactLaw(new UnilateralNewtonImpact);
    rc4->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc4->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc4->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc4->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }

  // Contact between Body1 and Frustum
  Contact *rc1f = new Contact("Contact1f");
  rc1f->connect(cup->getContour("Frustum"),body1->getContour("Circle1"));
  addLink(rc1f);
  if(rigidContact) {
    rc1f->setContactForceLaw(new UnilateralConstraint);
    rc1f->setContactImpactLaw(new UnilateralNewtonImpact);
    rc1f->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc1f->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc1f->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc1f->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }

  // Contact between Body2 and Frustum
  Contact *rc2f = new Contact("Contact2f");
  rc2f->connect(cup->getContour("Frustum"),body2->getContour("Circle2"));
  addLink(rc2f);
  if(rigidContact) {
    rc2f->setContactForceLaw(new UnilateralConstraint);
    rc2f->setContactImpactLaw(new UnilateralNewtonImpact);
    rc2f->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc2f->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc2f->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc2f->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }

  // Contact between Body3 and Frustum
  Contact *rc3f = new Contact("Contact3f");
  rc3f->connect(cup->getContour("Frustum"),body3->getContour("Circle3"));
  addLink(rc3f);
  if(rigidContact) {
    rc3f->setContactForceLaw(new UnilateralConstraint);
    rc3f->setContactImpactLaw(new UnilateralNewtonImpact);
    rc3f->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc3f->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc3f->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc3f->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }
  
  // Contact between Body4 and Frustum
  Contact *rc4f = new Contact("Contact4f");
  rc4f->connect(cup->getContour("Frustum"),body4->getContour("Circle4"));
  addLink(rc4f);
  if(rigidContact) {
    rc4f->setContactForceLaw(new UnilateralConstraint);
    rc4f->setContactImpactLaw(new UnilateralNewtonImpact);
    rc4f->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc4f->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc4f->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc4f->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }

  // Contact between Body2 and Body4
  Contact *rc24 = new Contact("Contact24");
  rc24->connect(body2->getContour("Circle2"),body4->getContour("Circle4"));
  addLink(rc24);
  if(rigidContact) {
    rc24->setContactForceLaw(new UnilateralConstraint);
    rc24->setContactImpactLaw(new UnilateralNewtonImpact);
    rc24->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc24->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc24->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
    rc24->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(mu));
  }


#ifdef HAVE_OPENMBVCPPINTERFACE
  body1->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Frustum* dummy1 = new OpenMBV::Frustum;
  dummy1->setBaseRadius(2*d);
  dummy1->setTopRadius(2*d);
  dummy1->setHeight(d); 
  dummy1->setScaleFactor(1.);
  dummy1->setMinimalColorValue(0);
  dummy1->setMaximalColorValue(1);
  dummy1->setStaticColor(1);
  body1->setOpenMBVRigidBody(dummy1);

  body2->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Frustum* dummy2 = new OpenMBV::Frustum;
  dummy2->setBaseRadius(d);
  dummy2->setTopRadius(d);
  dummy2->setHeight(d); 
  dummy2->setScaleFactor(1.);
  dummy2->setMinimalColorValue(0);
  dummy2->setMaximalColorValue(1);
  dummy2->setStaticColor(0.5);
  body2->setOpenMBVRigidBody(dummy2);

  body3->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Frustum* dummy3 = new OpenMBV::Frustum;
  dummy3->setBaseRadius(d);
  dummy3->setTopRadius(d);
  dummy3->setHeight(d); 
  dummy3->setScaleFactor(1.);
  dummy3->setMinimalColorValue(0);
  dummy3->setMaximalColorValue(1);
  dummy3->setStaticColor(0.1);
  body3->setOpenMBVRigidBody(dummy3);

  body4->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Frustum* dummy4 = new OpenMBV::Frustum;
  dummy4->setBaseRadius(d);
  dummy4->setTopRadius(d);
  dummy4->setHeight(d); 
  dummy4->setScaleFactor(1.);
  dummy4->setMinimalColorValue(0);
  dummy4->setMaximalColorValue(1);
  dummy4->setStaticColor(0.3);
  body4->setOpenMBVRigidBody(dummy4);

#endif
}

