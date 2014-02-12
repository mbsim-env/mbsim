#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/sphere.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

SlidingMass::SlidingMass(const string &projectName) :
    DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(0) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double l0 = 1.5; // length
  double b0 = 0.1; // width
  double E = 5.e7; // E-Modul  
  double A = b0 * b0; // cross-section area
  double I1 = 1. / 12. * b0 * b0 * b0 * b0; // moment inertia
  double rho = 9.2e2; // density  
  int elements = 10; // number of finite elements

  double mass = 1.; // mass of ball
  double r = b0; // radius of ball

  Vec3 rcmDispl;
  rcmDispl(2) = 5*b0;
  FixedRelativeFrame * rcmRef = new FixedRelativeFrame("RCMReference", rcmDispl);
  rcmRef->setFrameOfReference(getFrameI());
  addFrame(rcmRef);


  FlexibleBody1s21RCM *rod = new FlexibleBody1s21RCM("Rod", true);
  rod->setLength(l0);
  rod->setEModul(E);
  rod->setCrossSectionalArea(A);
  rod->setMomentInertia(I1);
  rod->setDensity(rho);
  rod->setFrameOfReference(rcmRef);
  rod->setNumberElements(elements);
  Vec q0 = Vec(5 * elements + 3, INIT, 0.);
  for (int i = 1; i <= elements; i++)
    q0(5 * i ) = l0 * i / elements;
  rod->setq0(q0);
  this->addObject(rod);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid = new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements * 4 + 1); // resolution of visualisation
  cuboid->setDiffuseColor(0.7,0,0); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>; // clockwise ordering, no doubling for closure
  OpenMBV::PolygonPoint *corner1 = new OpenMBV::PolygonPoint(b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner1);
  OpenMBV::PolygonPoint *corner2 = new OpenMBV::PolygonPoint(b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner2);
  OpenMBV::PolygonPoint *corner3 = new OpenMBV::PolygonPoint(-b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner3);
  OpenMBV::PolygonPoint *corner4 = new OpenMBV::PolygonPoint(-b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
#endif

  Vec WrOS0B(3, INIT, 0.);
  WrOS0B(0) = l0 * 0.9;
  FixedRelativeFrame * ballRef = new FixedRelativeFrame("B", WrOS0B);
  ballRef->setFrameOfReference(rcmRef);
  addFrame(ballRef);

  RigidBody *ball = new RigidBody("Ball");
  addObject(ball);
  ball->setFrameOfReference(ballRef);
  ball->setMass(mass);
  SymMat Theta(3);
  Theta(0, 0) = 2. / 5. * mass * r * r;
  Theta(1, 1) = 2. / 5. * mass * r * r;
  Theta(2, 2) = 2. / 5. * mass * r * r;
  ball->setInertiaTensor(Theta);
  ball->setTranslation(new LinearTranslation(Mat3x3(EYE)));
  Vec u0Ball(3, INIT, 0.);
  u0Ball(1) = 1;
  ball->setInitialGeneralizedVelocity(u0Ball);


  Point * ballContour = new Point("Mass");
  ball->addContour(ballContour);
  ballContour->setFrameOfReference(ball->getFrameC());

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Sphere * sphere = new OpenMBV::Sphere;
  sphere->setRadius(r);
  sphere->setDiffuseColor(0.5,0.5,1);
  sphere->setTransparency(0.7);
  sphere->setEnable(true);
  ball->setOpenMBVRigidBody(sphere);
#endif


  Contact *contact = new Contact("Contact");
  contact->setContactForceLaw(new BilateralConstraint);
  contact->setContactImpactLaw(new BilateralImpact);
  contact->connect(ballContour, rod->getContour("Contour1sFlexible"));
  this->addLink(contact);

  ContourPointData cpdata;
  cpdata.getLagrangeParameterPosition() = Vec(1, INIT, 0.);
  cpdata.getContourParameterType() = CONTINUUM;
  rod->addFrame("RJ", cpdata);
  Joint *joint = new Joint("Clamping");
  joint->connect(this->getFrame("I"), rod->getFrame("RJ"));
  joint->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->setImpactForceLaw(new BilateralImpact);
  joint->setMomentDirection("[0; 0; 1]");
  joint->setMomentLaw(new BilateralConstraint);
  joint->setImpactMomentLaw(new BilateralImpact);
  this->addLink(joint);
}

