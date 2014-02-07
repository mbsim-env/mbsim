#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_ancf.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"


#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h"
#include <openmbvcppinterface/polygonpoint.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // gravity
  Vec grav(3);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // data beam
  double l0 = 1.5; // length 
  double b0 = 0.1; // width
  double E = 500.e7; // E-Modul
  double A = b0*b0; // cross-section area
  double I1 = 1./12.*b0*b0*b0*b0; // moment inertia
  double rho = 9.2e2; // density  
  int elements = 4; // number of finite elements
  
  // data point mass
  double mass = 2.; // mass of ball
  double r = 1.e-2; // radius of ball
  
  // beam with some angle around z-axis and shift
  FlexibleBody1s21ANCF *rod = new FlexibleBody1s21ANCF("Rod", true);
  rod->setLength(l0);
  rod->setEModul(E);
  rod->setCrossSectionalArea(A);
  rod->setMomentInertia(I1);
  rod->setDensity(rho);
  Vec VecMove(3,INIT,0.);
  //VecMove(0) = 0.3*l0;
  double alpha = 0.; //
  double beta = M_PI/4.; // angle of rotation y
  double gamma = 0.; // angle of rotation z
  SqrMat K=Cardan2AIK(alpha,beta,gamma);
  
  this->addFrame(new FixedRelativeFrame("R",VecMove,K,this->getFrame("I")));
  this->getFrame("R")->enableOpenMBV();
  rod->setFrameOfReference(this->getFrame("R"));
  rod->setNumberElements(elements);
  Vec q0 = Vec(4*elements+4,INIT,0.);
  for(int i=0;i<=elements;i++) {
	  q0(4*i) = l0*i/elements;
	  q0(4*i+2) = 1;
  }
  rod->setq0(q0);
  this->addObject(rod);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid=new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements*4+1); // resolution of visualisation
  cuboid->setStaticColor(0.6); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1); // orthotropic scaling of cross section
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

  FlexibleBand *top = new FlexibleBand("Top");
  Vec nodes(elements+1);
  for(int i=0;i<=elements;i++) nodes(i) = i*l0/elements;
  top->setNodes(nodes);
  top->setWidth(b0);
  top->setCn(Vec("[1.;0.]"));
  top->setAlphaStart(0.);
  top->setAlphaEnd(l0);  
  top->setNormalDistance(0.5*b0);
  rod->addContour(top);
  
  // point mass with point contour at a specific surface point
  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3,INIT,0.);
  WrOS0B(0) = 0.8*l0; WrOS0B(1) = b0*0.5+0.35;
  this->addFrame(new FixedRelativeFrame("B",WrOS0B,SqrMat(3,EYE),this->getFrame("R")));
  ball->setFrameOfReference(this->getFrame("B"));
  ball->setFrameForKinematics(ball->getFrame("C"));
  ball->setMass(mass);
  SymMat Theta(3);
  Theta(0,0) = 2./5.*mass*r*r;
  Theta(1,1) = 2./5.*mass*r*r;
  Theta(2,2) = 2./5.*mass*r*r;
  ball->setInertiaTensor(Theta);
  Mat JacTrans(3,2,INIT,0.); JacTrans(0,0) = 1.; JacTrans(1,1) = 1.;
  ball->setTranslation(new LinearTranslation(JacTrans));
  Point *point = new Point("Point");
  Vec BR(3,INIT,0.); BR(1)=-r;
  ball->addContour(point,BR,SqrMat(3,EYE),ball->getFrame("C"));
  ball->setInitialGeneralizedVelocity(Vec(2,INIT,0.));
  this->addObject(ball);
  
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Sphere *sphere=new OpenMBV::Sphere;
  sphere->setRadius(r);
  sphere->setStaticColor(0.5);
  ball->setOpenMBVRigidBody(sphere);
#endif

  // elastic impact
  Contact *contact = new Contact("Contact");
  contact->setContactForceLaw(new UnilateralConstraint);
  contact->setContactImpactLaw(new UnilateralNewtonImpact(1.0));
  contact->connect(ball->getContour("Point"),rod->getContour("Top"));
  contact->enableOpenMBVContactPoints();
  this->addLink(contact);
  
  // 
  ContourPointData cpdata;
  cpdata.getLagrangeParameterPosition() = Vec(1,INIT,0.);
  cpdata.getContourParameterType() = CONTINUUM;
  rod->addFrame("RJ",cpdata);
  Joint *joint = new Joint("Clamping");
  joint->connect(this->getFrame("R"),rod->getFrame("RJ")); 
  joint->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->setImpactForceLaw(new BilateralImpact);
  joint->setMomentDirection("[0; 0; 1]");
  joint->setMomentLaw(new BilateralConstraint);
  joint->setImpactMomentLaw(new BilateralImpact);
  this->addLink(joint);
}

