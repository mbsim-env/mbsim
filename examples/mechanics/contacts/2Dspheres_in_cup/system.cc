#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/links/contact.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/planar_frustum.h"
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

  // First Body
  RigidBody* body1 = new RigidBody("Body1");
  addObject(body1);

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("C"));
  body1->setMass(m);
  body1->setInertiaTensor(Theta);
  body1->setTranslation(new TranslationAlongAxesXY<VecV>);
  body1->setRotation(new RotationAboutZAxis<VecV>);

  // Second Body
  RigidBody* body2 = new RigidBody("Body2");
  addObject(body2);

  body2->setFrameOfReference(getFrame("I"));
  body2->setFrameForKinematics(body2->getFrame("C"));
  body2->setMass(m);
  body2->setInertiaTensor(Theta);
  body2->setTranslation(new TranslationAlongAxesXY<VecV>);
  body2->setRotation(new RotationAboutZAxis<VecV>);

  // Third Body
  RigidBody* body3 = new RigidBody("Body3");
  addObject(body3);

  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new TranslationAlongAxesXY<VecV>);
  body3->setRotation(new RotationAboutZAxis<VecV>);

  // Fourth Body
  RigidBody* body4 = new RigidBody("Body4");
  addObject(body4);

  body4->setFrameOfReference(getFrame("I"));
  body4->setFrameForKinematics(body4->getFrame("C"));
  body4->setMass(m);
  body4->setInertiaTensor(Theta);
  body4->setTranslation(new TranslationAlongAxesXY<VecV>);
  body4->setRotation(new RotationAboutZAxis<VecV>);

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
  Circle *circlecontour1 = new Circle("Circle1",2*d);
  body1->addContour(circlecontour1);

  // Contour of Body2
  Circle *circlecontour2 = new Circle("Circle2",d);
  body2->addContour(circlecontour2);

  // Contour of Body3
  Circle *circlecontour3 = new Circle("Circle3",d);
  body3->addContour(circlecontour3);

  // Contour of Body4
  Circle *circlecontour4 = new Circle("Circle4",d);
  body4->addContour(circlecontour4);

  // Contour of ground plane
  SqrMat A(3);
  A(2,2) = 1;
  A(1,0) = sin(M_PI/2);
  A(0,0) = cos(M_PI/2);
  A(0,1) = -sin(M_PI/2);
  A(1,1) = cos(M_PI/2);

  addFrame(new FixedRelativeFrame("Line",Vec(3),A));
  addContour(new Line("Line",getFrame("Line")));

  // Contour of Frustum
  Vec radii(2);
  radii(0) = 3*d, radii(1) = 5*d;
  PlanarFrustum* frustumcontour = new PlanarFrustum("Frustum");
  frustumcontour->setRadii(radii);
  frustumcontour->setHeight(5*d);
#ifdef HAVE_OPENMBVCPPINTERFACE
  frustumcontour->enableOpenMBV();
#endif
  cup->addContour(frustumcontour);


  // Contact between Body1 and plane
  Contact *rc1 = new Contact("Contact1");
  rc1->connect(getContour("Line"),body1->getContour("Circle1"));
  addLink(rc1);
  if(rigidContact) {
    rc1->setNormalForceLaw(new UnilateralConstraint);
    rc1->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc1->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc1->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc1->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc1->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }


  // Contact between Body2 and plane
  Contact *rc2 = new Contact("Contact2");
  rc2->connect(getContour("Line"),body2->getContour("Circle2"));
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

  // Contact between Body3 and plane
  Contact *rc3 = new Contact("Contact3");
  rc3->connect(getContour("Line"),body3->getContour("Circle3"));
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

  // Contact between Body4 and plane
  Contact *rc4 = new Contact("Contact4");
  rc4->connect(getContour("Line"),body4->getContour("Circle4"));
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

  // Contact between Body1 and Frustum
  Contact *rc1f = new Contact("Contact1f");
  rc1f->connect(cup->getContour("Frustum"),body1->getContour("Circle1"));
  addLink(rc1f);
  if(rigidContact) {
    rc1f->setNormalForceLaw(new UnilateralConstraint);
    rc1f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc1f->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc1f->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc1f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc1f->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between Body2 and Frustum
  Contact *rc2f = new Contact("Contact2f");
  rc2f->connect(cup->getContour("Frustum"),body2->getContour("Circle2"));
  addLink(rc2f);
  if(rigidContact) {
    rc2f->setNormalForceLaw(new UnilateralConstraint);
    rc2f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc2f->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc2f->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc2f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc2f->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between Body3 and Frustum
  Contact *rc3f = new Contact("Contact3f");
  rc3f->connect(cup->getContour("Frustum"),body3->getContour("Circle3"));
  addLink(rc3f);
  if(rigidContact) {
    rc3f->setNormalForceLaw(new UnilateralConstraint);
    rc3f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc3f->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc3f->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc3f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc3f->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }
  
  // Contact between Body4 and Frustum
  Contact *rc4f = new Contact("Contact4f");
  rc4f->connect(cup->getContour("Frustum"),body4->getContour("Circle4"));
  addLink(rc4f);
  if(rigidContact) {
    rc4f->setNormalForceLaw(new UnilateralConstraint);
    rc4f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc4f->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc4f->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc4f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc4f->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between Body2 and Body4
  Contact *rc24 = new Contact("Contact24");
  rc24->connect(body2->getContour("Circle2"),body4->getContour("Circle4"));
  addLink(rc24);
  if(rigidContact) {
    rc24->setNormalForceLaw(new UnilateralConstraint);
    rc24->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc24->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    rc24->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc24->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc24->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }


#ifdef HAVE_OPENMBVCPPINTERFACE
  body1->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Frustum> dummy1 = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  dummy1->setBaseRadius(2*d);
  dummy1->setTopRadius(2*d);
  dummy1->setHeight(d); 
  dummy1->setScaleFactor(1.);
  dummy1->setMinimalColorValue(0);
  dummy1->setMaximalColorValue(1);
  dummy1->setDiffuseColor(0.3333,0.6666,1);
  body1->setOpenMBVRigidBody(dummy1);

  body2->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Frustum> dummy2 = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  dummy2->setBaseRadius(d);
  dummy2->setTopRadius(d);
  dummy2->setHeight(d); 
  dummy2->setScaleFactor(1.);
  dummy2->setMinimalColorValue(0);
  dummy2->setMaximalColorValue(1);
  dummy2->setDiffuseColor(0.3333,1,1);
  body2->setOpenMBVRigidBody(dummy2);

  body3->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Frustum> dummy3 = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  dummy3->setBaseRadius(d);
  dummy3->setTopRadius(d);
  dummy3->setHeight(d); 
  dummy3->setScaleFactor(1.);
  dummy3->setMinimalColorValue(0);
  dummy3->setMaximalColorValue(1);
  dummy3->setDiffuseColor(0.6666,0.6666,1);
  body3->setOpenMBVRigidBody(dummy3);

  body4->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Frustum> dummy4 = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  dummy4->setBaseRadius(d);
  dummy4->setTopRadius(d);
  dummy4->setHeight(d); 
  dummy4->setScaleFactor(1.);
  dummy4->setMinimalColorValue(0);
  dummy4->setMaximalColorValue(1);
  dummy3->setDiffuseColor(0.6666,1,0.6666);
  body4->setOpenMBVRigidBody(dummy4);

#endif
}

