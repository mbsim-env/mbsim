#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/multi_contact.h"
#include "mbsim/contour.h"
#include "mbsim/contours/point.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h"
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  
  Vec grav(3,INIT,0.);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double l0 = 1.5; // length
  double b0 = 0.1; // width
  double E = 5.e7; // E-Modul
  double mu = 0.3; // Poisson ratio
  double G = E/(2*(1+mu)); // shear modulus
  double A = b0*b0; // cross-section area
  double I1 = 1./12.*b0*b0*b0*b0; // moment inertia
  double I2 = 1./12.*b0*b0*b0*b0; // moment inertia
  double I0 = 0.05*(I1 + I2);
  double rho = 9.2e2; // density 
  int elements = 2; // number of finite elements

  double mass = 50.; // mass of ball
  double r = 1.e-2; // radius of ball

  FlexibleBody1s33RCM *rod = new FlexibleBody1s33RCM("Rod", true);
  rod->setLength(l0);
  rod->setEGModuls(E,G);
  rod->setCrossSectionalArea(A);
  rod->setMomentsInertia(I1,I2,I0);
  rod->setDensity(rho);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->setCuboid(b0,b0);

  Vec q0 = Vec(10*elements+6,INIT,0.);
  for(int i=1;i<=elements;i++) q0(10*i) = l0*i/elements;
  rod->setq0(q0);
  this->addObject(rod);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid=new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements*4+1); // resolution of visualisation
  cuboid->setStaticColor(0.6); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>; // clockwise ordering, no doubling for closure
  OpenMBV::PolygonPoint *corner1 = new OpenMBV::PolygonPoint(b0*0.5,b0*0.5,1);
  rectangle->push_back(corner1);
  OpenMBV::PolygonPoint *corner2 = new OpenMBV::PolygonPoint(b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner2);
  OpenMBV::PolygonPoint *corner3 = new OpenMBV::PolygonPoint(-b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner3);
  OpenMBV::PolygonPoint *corner4 = new OpenMBV::PolygonPoint(-b0*0.5,b0*0.5,1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
#endif

  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3,INIT,0.);
  WrOS0B(0) = l0*0.75; WrOS0B(1) = b0*0.5+r; WrOS0B(2) = b0*0.3;
  this->addFrame("B",WrOS0B,SqrMat(3,EYE),this->getFrame("I"));
  ball->setFrameOfReference(this->getFrame("B"));
  ball->setFrameForKinematics(ball->getFrame("C"));
  ball->setMass(mass);
  SymMat Theta(3);
  Theta(0,0) = 2./5.*mass*r*r;
  Theta(1,1) = 2./5.*mass*r*r;
  Theta(2,2) = 2./5.*mass*r*r;
  ball->setInertiaTensor(Theta);
  ball->setTranslation(new LinearTranslation(Mat(3,3,EYE)));
  Point *point = new Point("Point");
  Vec BR(3,INIT,0.); BR(1)=-r;
  ball->addContour(point,BR,SqrMat(3,EYE),ball->getFrame("C"));
  this->addObject(ball);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Sphere *sphere=new OpenMBV::Sphere;
  sphere->setRadius(r);
  sphere->setStaticColor(0.5);
  ball->setOpenMBVRigidBody(sphere);
#endif

  Contact *contact = new Contact("Contact");
  contact->setContactForceLaw(new UnilateralConstraint);
  contact->setContactImpactLaw(new UnilateralNewtonImpact(1.0));
  contact->connect(ball->getContour("Point"),rod->getContour("Top"));
  OpenMBV::Arrow *a_n = new OpenMBV::Arrow;
  //a_n->setHeadDiameter(tP*0.05);
  //a_n->setHeadLength(tP*0.07);
  //a_n->setDiameter(tP*0.02);
  //a_n->setScaleLength(tP*0.1);
  //a_n->setEnable(false);
  contact->setOpenMBVNormalForceArrow(a_n);
  OpenMBV::Arrow *a_t = new OpenMBV::Arrow;
  //a_t->setHeadDiameter(tP*0.05);
  //a_t->setHeadLength(tP*0.07);
  //a_t->setDiameter(tP*0.02);
  //a_t->setScaleLength(tP*0.1);
  //a_t->setEnable(false);
  contact->setOpenMBVFrictionForceArrow(a_t);
  contact->enableOpenMBVContactPoints();

  this->addLink(contact);

  ContourPointData cpdata;
  cpdata.getLagrangeParameterPosition() = Vec(1,INIT,0.);
  cpdata.getContourParameterType() = CONTINUUM;
  rod->addFrame("RJ",cpdata);
  Joint *joint = new Joint("Clamping");
  joint->connect(this->getFrame("I"),rod->getFrame("RJ")); 
  joint->setForceDirection(Mat(3,3,EYE));
  joint->setForceLaw(new BilateralConstraint);
  joint->setImpactForceLaw(new BilateralImpact);
  joint->setMomentDirection(Mat(3,3,EYE));
  joint->setMomentLaw(new BilateralConstraint);
  joint->setImpactMomentLaw(new BilateralImpact);
  this->addLink(joint);
}

