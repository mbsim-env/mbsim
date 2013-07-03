#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/spring_damper.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/sphere.h"
#include "openmbvcppinterface/cube.h"
#include "openmbvcppinterface/coilspring.h"
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
  double c=1e3;


  // Matrix from inertial frame I to new frame Z
  double alpha = M_PI/10.;
  SqrMat AZI(3);
  AZI(2,2) = 1;
  AZI(0,0) = cos(alpha);
  AZI(0,1) = sin(alpha);
  AZI(1,1) = cos(alpha);
  AZI(1,0) = -sin(alpha);

  // Vector to origin of frame D (in frame Z)
  Vec rD(3);
  rD(0) = 15*d;
  rD(1) = d;

  addFrame("Z",Vec(3),AZI,getFrame("I"));
  addFrame("D",rD,SqrMat(3,EYE),getFrame("Z"));



  // Rolling bodies
    // Cylinder
    RigidBody* body = new RigidBody("CylinderSolid");
    addObject(body);

    body->setFrameOfReference(getFrame("I"));
    body->setFrameForKinematics(body->getFrame("C"));
    body->setMass(m);
    body->setInertiaTensor(Theta);
    body->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
    body->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

    body->setPlotFeature(energy, enabled);

    // Hollow cylinder
    RigidBody* body2 = new RigidBody("CylinderHollow");
    addObject(body2);

    body2->setFrameOfReference(getFrame("I"));
    body2->setFrameForKinematics(body2->getFrame("C"));
    body2->setMass(m);
    Theta(1,1) = m*d*d;
    Theta(2,2) = Theta(1,1);
    body2->setInertiaTensor(Theta);
    body2->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
    body2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

    body2->setPlotFeature(energy, enabled);

    // Sphere
    RigidBody* body3 = new RigidBody("Sphere");
    addObject(body3);

    body3->setFrameOfReference(getFrame("I"));
    body3->setFrameForKinematics(body3->getFrame("C"));
    body3->setMass(m);
    Theta(1,1) = 2/5.0*m*d*d;
    Theta(2,2) = Theta(1,1);
    body3->setInertiaTensor(Theta);
    body3->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
    body3->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

    body3->setPlotFeature(energy, enabled);


  // Stopper
  RigidBody* body4 = new RigidBody("Stopper");
  addObject(body4);

  body4->setFrameOfReference(getFrame("D"));
  body4->setFrameForKinematics(body4->getFrame("C"));
  body4->setMass(m/100.);
  body4->setInertiaTensor(Theta);
  body4->setTranslation(new LinearTranslation("[1,0;0,1;0,0]"));
 

  // Spring 
  SpringDamper *spring = new SpringDamper("Spring");
  addLink(spring);
  spring->setForceFunction(new LinearSpringDamperForce(c,c/10.,d));
  spring->connect(body4->getFrame("C"), getFrame("D"));



  // Initial translation
  Vec q0(3);
  q0(1) = d;
  body->setInitialGeneralizedPosition(q0);
  body2->setInitialGeneralizedPosition(q0);
  body3->setInitialGeneralizedPosition(q0);
  Vec q04(2);
  q04(0) = -d;
  body4->setInitialGeneralizedPosition(q04);


  // Contour of Cylinder
  CircleSolid *circlecontour=new CircleSolid("Circle",d);
  body->addContour(circlecontour,Vec(3),SqrMat(3,EYE));

  // Contour of HollowCylinder (outward)
  CircleSolid *circlecontour2=new CircleSolid("Circle2",d);
  body2->addContour(circlecontour2,Vec(3),SqrMat(3,EYE));

  // Contour of Sphere
  Sphere *spherecontour=new Sphere("Sphere",d);
  body3->addContour(spherecontour,Vec(3),SqrMat(3,EYE));
 
  // Contour of Stopper
  Vec rBP(3), rBP2(3);
  rBP(1)=-d;
  rBP2(0)=-d/2.;
  body4->addContour(new Point("PointStopper"),rBP,SqrMat(3,EYE)); // pairing with ground
  body4->addContour(new Line("LineStopper"),rBP2,(SqrMat("[-1,0,0;0,-1,0;0,0,1]"))); // pairing with cylinders
  body4->addContour(new Plane("PlaneStopper"),rBP2,(SqrMat("[-1,0,0;0,-1,0;0,0,1]"))); // pairing with sphere

  // Contour of ground plane (x-axis of Line/Plane || y_Z)
  addContour(new Line("Line"),Vec(3),(SqrMat("[0,-1,0;1,0,0;0,0,1]")),getFrame("Z"));
  addContour(new Plane("Plane"),Vec(3),(SqrMat("[0,-1,0;1,0,0;0,0,1]")),getFrame("Z"));


  // Contact between Cylinder and plane (Contact-Pairing: CircleSolid-Line)
  Contact *rc = new Contact("Contact");
  rc->connect(getContour("Line"),body->getContour("Circle"));
  addLink(rc);
  if(rigidContact) {
    rc->setContactForceLaw(new UnilateralConstraint);
    rc->setContactImpactLaw(new UnilateralNewtonImpact);
    rc->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    rc->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  } 
  else {
    rc->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }


  // Contact between HollowCylinder and plane (Contact-Pairing: CircleSolid-Line)
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
    rc2->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc2->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  
  // Contact between Sphere and plane (Contact-Pairing: Plane-Sphere)
  Contact *rc3 = new Contact("Contact3");
  rc3->connect(getContour("Plane"),body3->getContour("Sphere"));
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


  // Contact Stopper and ground (Contact-Pairing: Point-Line)
  Contact *rc4 = new Contact("Contact4");
  rc4->connect(getContour("Line"),body4->getContour("PointStopper"));
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

  // Contact Stopper and Cylinder (Contact-Pairing: CircleSolid-Line)
  Contact *rc5 = new Contact("Contact5");
  rc5->connect(body->getContour("Circle"),body4->getContour("LineStopper"));
  addLink(rc5);
  if(rigidContact) {
    rc5->setContactForceLaw(new UnilateralConstraint);
    rc5->setContactImpactLaw(new UnilateralNewtonImpact);
    rc5->setFrictionForceLaw(new PlanarCoulombFriction(0));
    rc5->setFrictionImpactLaw(new PlanarCoulombImpact(0));
  } 
  else {
    rc5->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc5->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact Stopper and CylinderHollow (Contact-Pairing: CircleSolid-Line)
  Contact *rc6 = new Contact("Contact6");
  rc6->connect(body2->getContour("Circle2"),body4->getContour("LineStopper"));
  addLink(rc6);
  if(rigidContact) {
    rc6->setContactForceLaw(new UnilateralConstraint);
    rc6->setContactImpactLaw(new UnilateralNewtonImpact);
    rc6->setFrictionForceLaw(new PlanarCoulombFriction(0));
    rc6->setFrictionImpactLaw(new PlanarCoulombImpact(0));
  } 
  else{
    rc6->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc6->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

  // Contact Stopper and Sphere (Contact-Pairing: Sphere-Plane)
  Contact *rc7 = new Contact("Contact7");
  rc7->connect(body3->getContour("Sphere"),body4->getContour("PlaneStopper"));
  addLink(rc7);
  if(rigidContact) {
    rc7->setContactForceLaw(new UnilateralConstraint);
    rc7->setContactImpactLaw(new UnilateralNewtonImpact);
    rc7->setFrictionForceLaw(new PlanarCoulombFriction(0));
    rc7->setFrictionImpactLaw(new PlanarCoulombImpact(0));
  } 
  else{
    rc7->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    rc7->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }


#ifdef HAVE_OPENMBVCPPINTERFACE
  body->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Frustum* dummy = new OpenMBV::Frustum;
  dummy->setBaseRadius(d);
  dummy->setTopRadius(d);
  dummy->setHeight(d); 
  dummy->setScaleFactor(1.);
  dummy->setMinimalColorValue(0);
  dummy->setMaximalColorValue(1);
  dummy->setStaticColor(1);
  body->setOpenMBVRigidBody(dummy);

  body2->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Frustum* dummy2 = new OpenMBV::Frustum;
  dummy2->setBaseRadius(d);
  dummy2->setTopRadius(d);
  dummy2->setInnerBaseRadius(0.9*d);
  dummy2->setInnerTopRadius(0.9*d);
  dummy2->setHeight(d); 
  dummy2->setScaleFactor(1.);
  dummy2->setMinimalColorValue(0);
  dummy2->setMaximalColorValue(1);
  dummy2->setStaticColor(0.5);
  body2->setOpenMBVRigidBody(dummy2);

  body3->getFrame("C")->enableOpenMBV(1.5*d);
  OpenMBV::Sphere* dummy3 = new OpenMBV::Sphere;
  dummy3->setRadius(d);
  dummy3->setScaleFactor(1.);
  dummy3->setMinimalColorValue(0);
  dummy3->setMaximalColorValue(1);
  dummy3->setStaticColor(0.1);
  body3->setOpenMBVRigidBody(dummy3);

  OpenMBV::Cube *cuboid=new OpenMBV::Cube;
  cuboid->setLength(d);
  cuboid->setStaticColor(0.7);
  body4->setOpenMBVRigidBody(cuboid);
  body4->getFrame("C")->enableOpenMBV(d);

  OpenMBV::CoilSpring* openMBVspring=new OpenMBV::CoilSpring;
  openMBVspring->setSpringRadius(d/5.);
  openMBVspring->setCrossSectionRadius(d/50.);
  openMBVspring->setNumberOfCoils(5);
  spring->setOpenMBVSpring(openMBVspring);

  #endif
}

