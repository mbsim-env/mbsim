#include "system.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/contours/neutral_contour/contour_1s_neutral_linear_external_FFR.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
// Beginning Contact
//#include "mbsim/rigid_body.h"
//#include "mbsim/contour.h"
//#include "mbsim/constitutive_laws.h"
// End Contact
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h" // ball
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h> // Contact
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName, const std::string & inputFilesPath) :
    DynamicSystemSolver(projectName) {
  Vec grav(3, INIT, 0.); //grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  FlexibleBodyLinearExternalFFR *beam = new FlexibleBodyLinearExternalFFR("beam", false);

//  setPlotFeature(plotRecursive, disabled);

  beam->readFEMData(inputFilesPath);

//  int elements = beam->getNumberElements();
  int nf = beam->getNumberModes();

  beam->setFrameOfReference(this->getFrame("I"));

  // set a initial angle for the body reference
  Vec q0 = Vec(nf + 6, INIT, 0.0);
  Vec u0Beam = Vec(nf + 6, INIT, 0.0);
//  q0(0) = 0.2;
//  q0(1) = 0.3;
//  q0(2) = 0.4;
//  q0(3) = M_PI / 6;
//  q0(4) = M_PI / 3;
//  q0(5) = M_PI / 3;

//  q0(6) = 0.2;  // active the first eigenmode
//  q0(7) = 0.15;
//  q0(8) = 0.11;
//  q0(9) = 0.09;
//  q0(10) = 0.03;

//  u0Beam(0) = 2;
//  u0Beam(1) = 3;
//  u0Beam(2) = 4;
//  u0Beam(3) = 10 * M_PI / 6;
//  u0Beam(4) = 10 * M_PI / 3;
//  u0Beam(5) = 10 * M_PI / 3;
//
//  u0Beam(6) = 0.02;
//  u0Beam(7) = 0.03;
//  u0Beam(8) = 0.04;
//  u0Beam(9) = 0.02;
//  u0Beam(10) = 0.03;

  beam->setq0(q0);
  beam->setu0(u0Beam);

//  Joint * fix = new Joint("Fix");
//  fix->connect(getFrameI(), beam->getFloatingFrameOfReference());
//  fix->setForceDirection(Mat3x3(EYE));
//  fix->setMomentDirection(Mat3x3(EYE));
//  fix->setForceLaw(new BilateralConstraint);
//  fix->setImpactForceLaw(new BilateralImpact);
//  fix->setMomentLaw(new BilateralConstraint);
//  fix->setImpactMomentLaw(new BilateralImpact);
//  addLink(fix);

  this->addObject(beam);

  // add neutral contour to the rod
  int numOfTransNodes = 9;
  std::vector<int> transNodes(numOfTransNodes);
  for (int i = 0; i < numOfTransNodes; i++) {
    transNodes[i] = 228 + i;  // node 228~236
  }

  double uMin = 0;
  double uMax = 1;
  int degU = 3;
  bool openStructure = true;
  double nodeOffset = 0.;

  Contour1sNeutralLinearExternalFFR* ncc = new Contour1sNeutralLinearExternalFFR("neutralFibre", beam, transNodes, nodeOffset, uMin, uMax, degU, openStructure);
  ncc->setFrameOfReference(beam->getFrameOfReference());
  ncc->setAlphaStart(uMin);
  ncc->setAlphaEnd(uMax);

  double b0 = 20;
  int elements = 10;

  FlexibleBand * top = new FlexibleBand("Top", true);
  top->setWidth(b0);
  top->setNormalDistance(0);
  top->setCn(Vec("[1.;0.]"));
  top->setNeutral(ncc);
  top->setAlphaStart(uMin);
  top->setAlphaEnd(uMax);

  Vec contourNodes(numOfTransNodes); // open structure
  for (int i = 0; i < numOfTransNodes; i++)
    contourNodes(i) = (uMax - uMin) / (numOfTransNodes - 1) * i;
  top->setNodes(contourNodes);  // depend on the range of the langrange parameter of the neutral fibre

  beam->addContour(top);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid = new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements * 4 + 1);
  cuboid->setStaticColor(0.5);
  cuboid->setScaleFactor(1.);
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>;
  OpenMBV::PolygonPoint *corner1 = new OpenMBV::PolygonPoint(0, b0 * 0.5, 1);
  rectangle->push_back(corner1);
  OpenMBV::PolygonPoint *corner2 = new OpenMBV::PolygonPoint(0, -b0 * 0.5, 1);
  rectangle->push_back(corner2);
//  OpenMBV::PolygonPoint *corner3 = new OpenMBV::PolygonPoint(b0 * 0.5, -b0 * 0.5, 1);
//  rectangle->push_back(corner3);
//  OpenMBV::PolygonPoint *corner4 = new OpenMBV::PolygonPoint(b0 * 0.5, b0 * 0.5, 1);
//  rectangle->push_back(corner4);
  cuboid->setContour(rectangle);
  top->setOpenMBVSpineExtrusion(cuboid, ncc);
#endif

// Beginning Contact ---------------------------------------------------
// for exciting the beam : ball mass 500; position(90, 200, 10), velocity(-100)

  double mass = 5000000.; // mass of ball
  double r = 1; // radius of ball

  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3, INIT, 0.);
  WrOS0B(0) = 83;  // between node 231-232 (68.     10.,          10.)
  WrOS0B(1) = 25;
  WrOS0B(2) = 18;
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

  Vec3 u0Ball;
  u0Ball(1) = -50;
  ball->setInitialGeneralizedVelocity(u0Ball);

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
  if (0) {
  contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e7,0.)));
  }
  else {
    contact->setContactForceLaw(new UnilateralConstraint);
    contact->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
  }
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

