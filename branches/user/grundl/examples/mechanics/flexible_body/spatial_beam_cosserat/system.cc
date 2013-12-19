#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsimFlexibleBody/contours/neutral_contour/contour_1s_neutral_cosserat.h"
// Beginning Contact
#include "mbsim/rigid_body.h"
#include "mbsim/contact.h"
#include "mbsim/contour.h"
#include "mbsim/contours/point.h"
#include "mbsim/constitutive_laws.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
// End Contact
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h" // ball
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h> // Contact
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) :
    DynamicSystemSolver(projectName) {

  Vec grav(3, INIT, 0.);
  //grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double l = 5.; // length
  double b0 = 0.05; // width
  double E = 5.e7; // E-Modul
  double mu = 0.3; // Poisson ratio
  double G = E / (2 * (1 + mu)); // shear modulus
  double A = b0 * b0; // cross-section area
  double I1 = 1. / 12. * b0 * b0 * b0 * b0; // moment inertia
  double I2 = 1. / 12. * b0 * b0 * b0 * b0;
  double I0 = I1 + I2;
  double rho = 9.2e2; // density
  int elements = 20; // number of finite elements

  FlexibleBody1s33Cosserat* rod = new FlexibleBody1s33Cosserat("Rod", false);
  rod->setLength(l);
  rod->setEGModuls(E, G);
  rod->setCrossSectionalArea(A);
  rod->setMomentsInertia(I1, I2, I0);
  rod->setDensity(rho);
  //rod->setMassProportionalDamping(20.);
  //rod->setMaterialDamping(0.1,0.1,0.1);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->setCuboid(b0, b0);
  rod->setCurlRadius(0., l / (2 * M_PI));

  // circle shape
  Vec q0 = Vec(6 * elements, INIT, 0.);
  double R = (l / (2. * M_PI)) * 1.0; // stretched circle
  double dphi = (2 * M_PI) / elements;
  for (int i = 0; i < elements; i++) {
    double phi = M_PI / 2. - i * dphi;
    q0(6 * i) = R * cos(phi);
    q0(6 * i + 1) = R * sin(phi);
    q0(6 * i + 5) = phi - dphi / 2. - M_PI / 2.;
  }

  // relaxed
  //rod->setCurlRadius(1e2,0.);
  //Vec q0 = Vec(6*elements+3,INIT,0.);
  //for(int i=0; i<=elements; i++) {
  //  q0(6*i) = i*l0/elements;
  //}

  rod->setq0(q0);
  rod->setu0(Vec(q0.size(), INIT, 0.));
  this->addObject(rod);

  // add neutral contour to the rod
  std::vector<int> transNodes(elements);
  std::vector<int> rotNodes(elements);
  for (int i = 0; i < elements; i++) {
    transNodes[i] = i;
    rotNodes[i] = i;
  }
  double uMin = 0;  // uMin has to be 0, otherwise Nurbscurve:globalInterpClosed():inv() fails;
  double uMax = 1;
  int degU = 3;
  bool openStructure = false;
  double nodeOffset = 0.5 * (uMax - uMin) / elements;
  Contour1sNeutralCosserat* ncc = new Contour1sNeutralCosserat("neutralFibre", rod, transNodes, rotNodes, nodeOffset, uMin, uMax, degU, openStructure);
  ncc->setFrameOfReference(rod->getFrameOfReference());
  ncc->setAlphaStart(uMin);
  ncc->setAlphaEnd(uMax);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid = new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements * 4 + 1);
  cuboid->setStaticColor(0.5);
  cuboid->setScaleFactor(1.);
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>;
  OpenMBV::PolygonPoint *corner1 = new OpenMBV::PolygonPoint(b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner1);
  OpenMBV::PolygonPoint *corner2 = new OpenMBV::PolygonPoint(b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner2);
  OpenMBV::PolygonPoint *corner3 = new OpenMBV::PolygonPoint(-b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner3);
  OpenMBV::PolygonPoint *corner4 = new OpenMBV::PolygonPoint(-b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner4);
  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid, ncc);
#endif

  FlexibleBand * top = new FlexibleBand("Top");
  top->setWidth(b0);
  top->setNormalDistance(0.5*b0);
  top->setCn(Vec("[1.;0.]"));
  top->setNeutral(ncc);
  top->setAlphaStart(uMin);
  top->setAlphaEnd(uMax);
  Vec contourNodes(elements + 1);
  for (int i = 0; i <= elements; i++)
    contourNodes(i) = (uMax - uMin) / elements * i;
  top->setNodes(contourNodes);

  rod->addContour(top);

// Beginning Contact ---------------------------------------------------

  double mass = 5.; // mass of ball
  double r = 2.e-2; // radius of ball

  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3, INIT, 0.);
  WrOS0B(0) = 0;
  WrOS0B(1) = R + r + 0.1;
  this->addFrame("B", WrOS0B, SqrMat(3, EYE), this->getFrame("I"));
  ball->setFrameOfReference(this->getFrame("B"));
  ball->setFrameForKinematics(ball->getFrame("C"));
  ball->setMass(mass);
  SymMat Theta(3);
  Theta(0, 0) = 2. / 5. * mass * r * r;
  Theta(1, 1) = 2. / 5. * mass * r * r;
  Theta(2, 2) = 2. / 5. * mass * r * r;
  ball->setInertiaTensor(Theta);
  ball->setTranslation(new LinearTranslation(Mat(3, 3, EYE)));

  Vec3 u0;
  u0(1) = -10;
  ball->setInitialGeneralizedVelocity(u0);

  MBSim::Point *point = new MBSim::Point("Point");
  Vec BR(3, INIT, 0.);
  BR(1) = -r;
  ball->addContour(point, BR, SqrMat(3, EYE), ball->getFrame("C"));
  this->addObject(ball);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Sphere *sphere = new OpenMBV::Sphere;
  sphere->setRadius(r);
  sphere->setStaticColor(0.5);
  ball->setOpenMBVRigidBody(sphere);
#endif

  Contact *contact = new Contact("Contact");
  contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e7,0.)));
//  contact->setContactForceLaw(new UnilateralConstraint);
//  contact->setContactImpactLaw(new UnilateralNewtonImpact(1.0));
  /********************* changed heare ********************/
//contact->connect(ball->getContour("Point"),rod->getContour("Top"));
  contact->connect(ball->getContour("Point"), top);
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
  contact->setOpenMBVFrictionArrow(a_t);
  contact->enableOpenMBVContactPoints();

  this->addLink(contact);

// End Contact ---------------------------------------------------
}
