#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/sphere.h"
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
  Theta(0,0) = Theta(1,1);
  double mu  = 0.3;


  // First Body
  RigidBody* body1 = new RigidBody("Body1");
  addObject(body1);

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("C"));
  body1->setMass(m);
  body1->setInertiaTensor(Theta);
  body1->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  body1->setRotation(new RotationAboutAxesXYZ<VecV>);

  // Second Body
  RigidBody* body2 = new RigidBody("Body2");
  addObject(body2);

  body2->setFrameOfReference(getFrame("I"));
  body2->setFrameForKinematics(body2->getFrame("C"));
  body2->setMass(m);
  body2->setInertiaTensor(Theta);
  body2->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  body2->setRotation(new RotationAboutAxesXYZ<VecV>);

  // Third Body
  RigidBody* body3 = new RigidBody("Body3");
  addObject(body3);

  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  body3->setRotation(new RotationAboutAxesXYZ<VecV>);

  // Cup
  RigidBody* cup = new RigidBody("Cup");
  addObject(cup);

  cup->setFrameOfReference(getFrame("I"));
  cup->setFrameForKinematics(cup->getFrame("C"));
  cup->setMass(50*m);
  cup->setInertiaTensor(Theta);

//  // Cup 2
//  RigidBody* cup2 = new RigidBody("Cup2");
//  addObject(cup2);
//
//  cup2->setFrameOfReference(getFrame("I"));
//  cup2->setFrameForKinematics(cup2->getFrame("C"));
//  cup2->addFrame("D","[0;0.5;0]", SqrMat(3,EYE), cup2->getFrame("C"));
//  cup2->setMass(50*m);
//  cup2->setInertiaTensor(Theta);


  // Initial translation and rotation
  Vec q0_1(6), q0_2(6), q0_3(6), q1(1);
  
  q0_1(0) = 0.25;
  q0_1(1) = 1.7;
  q0_1(2) = -0.1;
  
  q0_2(0)=-1.2;
  q0_2(1)=0.4;
  q0_2(2)=0.1;

  q0_3(0)=0.6;
  q0_3(1)=0.4;
  q0_3(2)=0.2;

  body1->setInitialGeneralizedPosition(q0_1);
  body2->setInitialGeneralizedPosition(q0_2);
  body3->setInitialGeneralizedPosition(q0_3);
  
  Vec u0_1(6), u0_2(6), u0_3(6);

  u0_1(5) = M_PI*30;

  u0_2(3) = M_PI*6;
  u0_2(5) = -M_PI*23;

  u0_3(1) = -2.;
  u0_3(4) = -M_PI*2;
  u0_3(5) = M_PI*20;

  body1->setInitialGeneralizedVelocity(u0_1);
  body2->setInitialGeneralizedVelocity(u0_2);
  body3->setInitialGeneralizedVelocity(u0_3);
 
  // Contour of Body1
  Sphere *spherecontour1 = new Sphere("Sphere1",2*d);
  body1->addContour(spherecontour1);
  
  // Contour of Body2
  Sphere *spherecontour2 = new Sphere("Sphere2",d);
  body2->addContour(spherecontour2);

  // Contour of Body3
  Sphere *spherecontour3 = new Sphere("Sphere3",d);
  body3->addContour(spherecontour3);

  // Contour of ground plane
  SqrMat A(3);
  A(2,2) = 1;
  A(0,1) = 1;
  A(1,0) = 1;
  addFrame(new FixedRelativeFrame("Plane",Vec(3),A));
  addContour(new Plane("Plane",getFrame("Plane")));

  // Contour of Frustum
  Vec radii(2);
  radii(0) = 3*d, radii(1) = 5*d;
  Frustum* frustumcontour = new Frustum("Frustum");
  frustumcontour->setRadii(radii);
  frustumcontour->setHeight(5*d);
#ifdef HAVE_OPENMBVCPPINTERFACE
  frustumcontour->enableOpenMBV();
#endif
  cup->addContour(frustumcontour);

//  // Contour of Frustum 2
//  Vec radii2(2);
//  radii2(0) = 3*d, radii2(1) = 5*d;
//  Frustum* frustumcontour2 = new Frustum("Frustum2");
//  frustumcontour2->setRadii(radii2);
//  frustumcontour2->setHeight(5*d);
//  cup2->addContour(frustumcontour2,Vec(3,INIT,0.),SqrMat(3,EYE),cup2->getFrame("D"));
  
  // Contact between Body1 and plane
  Contact *rc1 = new Contact("Contact1");
  rc1->connect(getContour("Plane"),body1->getContour("Sphere1"));
  addLink(rc1);
  if(rigidContact) {
    rc1->setNormalForceLaw(new UnilateralConstraint);
    rc1->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc1->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    rc1->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    rc1->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc1->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }


 // Contact between Body2 and plane
 Contact *rc2 = new Contact("Contact2");
 rc2->connect(getContour("Plane"),body2->getContour("Sphere2"));
 addLink(rc2);
 if(rigidContact) {
   rc2->setNormalForceLaw(new UnilateralConstraint);
   rc2->setNormalImpactLaw(new UnilateralNewtonImpact);
   rc2->setTangentialForceLaw(new SpatialCoulombFriction(mu));
   rc2->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
 } 
 else {
   rc2->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
   rc2->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
 }

 // Contact between Body3 and plane
 Contact *rc3 = new Contact("Contact3");
 rc3->connect(getContour("Plane"),body3->getContour("Sphere3"));
 addLink(rc3);
 if(rigidContact) {
   rc3->setNormalForceLaw(new UnilateralConstraint);
   rc3->setNormalImpactLaw(new UnilateralNewtonImpact);
   rc3->setTangentialForceLaw(new SpatialCoulombFriction(mu));
   rc3->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
 } 
 else {
   rc3->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
   rc3->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
 }

  // Contact between Body1 and Frustum
  Contact *rc1f = new Contact("Contact1f");
  rc1f->connect(cup->getContour("Frustum"),body1->getContour("Sphere1"));
  addLink(rc1f);
  if(rigidContact) {
    rc1f->setNormalForceLaw(new UnilateralConstraint);
    rc1f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc1f->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    rc1f->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    rc1f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc1f->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between Body2 and Frustum
  Contact *rc2f = new Contact("Contact2f");
  rc2f->connect(cup->getContour("Frustum"),body2->getContour("Sphere2"));
  addLink(rc2f);
  if(rigidContact) {
    rc2f->setNormalForceLaw(new UnilateralConstraint);
    rc2f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc2f->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    rc2f->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    rc2f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc2f->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between Body3 and Frustum
  Contact *rc3f = new Contact("Contact3f");
  rc3f->connect(cup->getContour("Frustum"),body3->getContour("Sphere3"));
  addLink(rc3f);
  if(rigidContact) {
    rc3f->setNormalForceLaw(new UnilateralConstraint);
    rc3f->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc3f->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    rc3f->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    rc3f->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc3f->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact between Body2 and Body3
  Contact *rc23 = new Contact("Contact23");
  rc23->connect(body2->getContour("Sphere2"),body3->getContour("Sphere3"));
  addLink(rc23);
  if(rigidContact) {
    rc23->setNormalForceLaw(new UnilateralConstraint);
    rc23->setNormalImpactLaw(new UnilateralNewtonImpact);
    rc23->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    rc23->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  } 
  else {
    rc23->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc23->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }


//  // Contact between Body1 and Frustum2
//  Contact *rc1f2 = new Contact("Contact1f2");
//  rc1f2->connect(cup2->getContour("Frustum2"),body1->getContour("Sphere1"));
//  addLink(rc1f2);
//  if(rigidContact) {
//    rc1f2->setNormalForceLaw(new UnilateralConstraint);
//    rc1f2->setNormalImpactLaw(new UnilateralNewtonImpact);
//    rc1f2->setTangentialForceLaw(new SpatialCoulombFriction(mu));
//    rc1f2->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
//  } 
//  else {
//    rc1f2->setNormalForceLaw(new LinearRegularizedUnilateralConstraint(1e5,1e4));
//    rc1f2->setTangentialForceLaw(new LinearRegularizedCoulombFriction(mu));}


#ifdef HAVE_OPENMBVCPPINTERFACE
  body1->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Sphere> dummy1 = OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
  dummy1->setRadius(2*d);
  dummy1->setScaleFactor(1.);
  dummy1->setMinimalColorValue(0);
  dummy1->setMaximalColorValue(1);
  dummy1->setDiffuseColor(0.3333,0.6666,1);
  body1->setOpenMBVRigidBody(dummy1);

  body2->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Sphere> dummy2 = OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
  dummy2->setRadius(d);
  dummy2->setScaleFactor(1.);
  dummy2->setMinimalColorValue(0);
  dummy2->setMaximalColorValue(1);
  dummy2->setDiffuseColor(0.3333,1,1);
  body2->setOpenMBVRigidBody(dummy2);

  body3->getFrame("C")->enableOpenMBV(1.5*d);
  boost::shared_ptr<OpenMBV::Sphere> dummy3 = OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
  dummy3->setRadius(d);
  dummy3->setScaleFactor(1.);
  dummy3->setMinimalColorValue(0);
  dummy3->setMaximalColorValue(1);
  dummy3->setDiffuseColor(0.6666,0.6666,1);
  body3->setOpenMBVRigidBody(dummy3);

//  cup2->getFrame("C")->enableOpenMBV(1.5*d);
//  boost::shared_ptr<OpenMBV::Frustum> dummy5 = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
//  dummy5->setBaseRadius(radii2(0));
//  dummy5->setTopRadius(radii2(1));
//  dummy5->setHeight(5*d); 
//  dummy5->setScaleFactor(1.);
//  dummy5->setMinimalColorValue(0);
//  dummy5->setMaximalColorValue(1);
//  dummy5->setDiffuseColor(0.2, 1, 1);
//  dummy5->setInitialRotation(-M_PI/2.,0 ,-M_PI/2);
//  dummy5->setInitialTranslation(0,10*d,0);
//  cup2->setOpenMBVRigidBody(dummy5);

#endif
}

