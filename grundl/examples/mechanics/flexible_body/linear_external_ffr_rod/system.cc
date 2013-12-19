#include "system.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/contours/neutral_contour/contour_2s_neutral_linear_external_FFR.h"
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

  FlexibleBodyLinearExternalFFR *rod = new FlexibleBodyLinearExternalFFR("rod", false);

//  setPlotFeature(plotRecursive, disabled);

  rod->readFEMData(inputFilesPath);

//  int elements = beam->getNumberElements();
  int nf = rod->getNumberModes();

  rod->setFrameOfReference(this->getFrame("I"));

  // set a initial angle for the body reference
  Vec q0 = Vec(nf + 6, INIT, 0.0);
  Vec u0Rod = Vec(nf + 6, INIT, 0.0);
//  q0(0) = 0.2;
//  q0(1) = 0.3;
//  q0(2) = 0.4;
//  q0(3) = M_PI / 6;
//  q0(4) = M_PI / 3;
//  q0(5) = M_PI / 3;

//   q0(6) = 0.2;  // active the first eigenmode
//  q0(7) = 0.15;
//  q0(8) = 0.11;
//  q0(9) = 0.09;
//  q0(10) = 0.03;
//   q0(15) = 0.1;

//  u0Beam(0) = 2;
//  u0Beam(1) = 3;
//  u0Beam(2) = 4;
//  u0Beam(3) = 10 * M_PI / 6;
//  u0Beam(4) = 10 * M_PI / 3;
//  u0Beam(5) = 10 * M_PI / 3;
//
//  u0Rod(6) = 0.02;
//  u0Beam(7) = 0.03;
//  u0Beam(8) = 0.04;
//  u0Beam(9) = 0.02;
//  u0Beam(10) = 0.03;

  rod->setq0(q0);
  rod->setu0(u0Rod);

  Joint * fix = new Joint("Fix");
  fix->connect(getFrameI(), rod->getFloatingFrameOfReference());
  fix->setForceDirection(Mat3x3(EYE));
  fix->setMomentDirection(Mat3x3(EYE));
  fix->setForceLaw(new BilateralConstraint);
  fix->setImpactForceLaw(new BilateralImpact);
  fix->setMomentLaw(new BilateralConstraint);
  fix->setImpactMomentLaw(new BilateralImpact);
  addLink(fix);

  this->addObject(rod);

  // add neutral contour to the rod
  int numOfTransNodesU = 24;
  int numOfTransNodesV = 5;
  Vec startingIndex1("[625; 473; 321; 169; 17]");
  Vec startingIndex2("[637; 485; 333; 181; 29]");
  Vec startingIndex3("[636; 484; 332; 180; 28]");
  Mat transNodes(numOfTransNodesU, numOfTransNodesV, NONINIT);
  for (int j = 0; j < numOfTransNodesV; j++)
    for (int i = 0; i < 11; i++)
        transNodes(i, j) = startingIndex1(j) + i;

  for (int j = 0; j < numOfTransNodesV; j++)
    for (int i = 0; i < 12; i++)
      transNodes(11 + i, j) = startingIndex2(j) + i;

  for (int j = 0; j < numOfTransNodesV; j++)
    transNodes(23, j) = startingIndex3(j);

  cout << transNodes << endl << endl;

  int degU = 3;
  int degV = 3;
  bool openStructure = false;
  double nodeOffset = 0.;

  Contour2sNeutralLinearExternalFFR* ncc = new Contour2sNeutralLinearExternalFFR("neutralFibre", rod, transNodes, nodeOffset, degU, degV, openStructure);
  ncc->setFrameOfReference(rod->getFrameOfReference());
  ncc->setAlphaStart(Vec(2, INIT, 0));
  ncc->setAlphaEnd(Vec(2, INIT, 1));


// Beginning Contact ---------------------------------------------------
// for exciting the beam : ball mass 500; position(90, 200, 10), velocity(-100)

  double mass = 500.; // mass of ball
  double r = 1; // radius of ball

  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3, INIT, 0.);
  WrOS0B(0) = 11.;
  WrOS0B(1) = -5;
  WrOS0B(2) = 0;
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
  contact->connect(ball->getContour("Point"), ncc);
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

