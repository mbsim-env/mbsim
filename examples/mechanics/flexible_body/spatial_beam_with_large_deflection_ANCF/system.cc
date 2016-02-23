#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_ancf.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h"
#include <openmbvcppinterface/polygonpoint.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;
using namespace boost;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // gravity
  Vec grav(3);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // data beam
  double l0 = 1.5; // length 
  double b0 = 0.1; // width
  double E = 2e8; // E-Modul
  double mu = 0.3; // Poisson ratio
  double G = E/(2*(1+mu)); // shear modulus
  double A = b0*b0; // cross-section area
  double I1 = 1./12.*b0*b0*b0*b0; // moment inertia
  double I2 = 1./12.*b0*b0*b0*b0; // moment inertia
  double I0 = 0.05*(I1 + I2);
  double rho = 9.2e2; // density  
  int elements = 4; // number of finite elements

  // data point mass
  double mass = 5.; // mass of ball
  double r = 1.e-2; // radius of ball

  // beam
  FlexibleBody1s33ANCF *rod = new FlexibleBody1s33ANCF("Rod", true);
  rod->setLength(l0);
  rod->setEModul(E);
  rod->setShearModul(G);
  rod->setCrossSectionalArea(A);
  rod->setMomentInertia(I0,I1,I2);
  rod->setDensity(rho);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->setCurlRadius(-100.,10.);
  Vec q0 = Vec(6*elements+6,INIT,0.);
  for(int i=0;i<=elements;i++) {
    q0(6*i) = l0*i/elements;
    q0(6*i+3) = 1;
  }
  rod->setq0(q0);
  this->addObject(rod);

#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::SpineExtrusion> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(elements*4+1); // resolution of visualisation
  cuboid->setDiffuseColor(0.26667, 1, 1); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  shared_ptr<OpenMBV::PolygonPoint>  corner1 = OpenMBV::PolygonPoint::create(b0*0.5,b0*0.5,1);
  rectangle->push_back(corner1);
  shared_ptr<OpenMBV::PolygonPoint>  corner2 = OpenMBV::PolygonPoint::create(b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner2);
  shared_ptr<OpenMBV::PolygonPoint>  corner3 = OpenMBV::PolygonPoint::create(-b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner3);
  shared_ptr<OpenMBV::PolygonPoint>  corner4 = OpenMBV::PolygonPoint::create(-b0*0.5,b0*0.5,1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
#endif

  FlexibleBand *top = new FlexibleBand("Top");
  Vec nodes(elements+1);
  for(int i=0;i<=elements;i++) nodes(i) = i*l0/elements;
  top->setNodes(nodes);
  top->setWidth(b0);
  Vec2 RrRP;
  RrRP(0) = 0.5*b0;
  top->setRelativePosition(RrRP);
  top->setRelativeOrientation(M_PI);
  rod->addContour(top);

  // point mass with point contour at a specific surface point
  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3,INIT,0.);
  WrOS0B(0) = 0.5*l0; WrOS0B(1) = b0*0.5+0.35; WrOS0B(2) = b0*0.25;
  this->addFrame(new FixedRelativeFrame("B",WrOS0B,SqrMat(3,EYE),this->getFrame("I")));
  ball->setFrameOfReference(this->getFrame("B"));
  ball->setFrameForKinematics(ball->getFrame("C"));
  ball->setMass(mass);
  SymMat Theta(3);
  Theta(0,0) = 2./5.*mass*r*r;
  Theta(1,1) = 2./5.*mass*r*r;
  Theta(2,2) = 2./5.*mass*r*r;
  ball->setInertiaTensor(Theta);
  Mat JacTrans(3,3,INIT,0.); JacTrans(0,0) = 1.; JacTrans(1,1) = 1.; JacTrans(2,2) = 1.;
  ball->setTranslation(new LinearTranslation<VecV>(JacTrans));
  Point *point = new Point("Point");
  Vec BR(3,INIT,0.); BR(1)=-r;
  ball->addFrame(new FixedRelativeFrame("Point",BR,SqrMat(3,EYE),ball->getFrame("C")));
  point->setFrameOfReference(ball->getFrame("Point"));
  ball->addContour(point);
  ball->setInitialGeneralizedVelocity(Vec(3,INIT,0.));
  this->addObject(ball);

#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Sphere> sphere=OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
  sphere->setRadius(r);
  sphere->setDiffuseColor(1/3., 1, 1);
  ball->setOpenMBVRigidBody(sphere);
#endif

  // elastic impact
  Contact *contact = new Contact("Contact");
  contact->setNormalForceLaw(new UnilateralConstraint);
  contact->setNormalImpactLaw(new UnilateralNewtonImpact(0.5));
  contact->connect(ball->getContour("Point"),rod->getContour("Top"));
  this->addLink(contact);

  // joint 
  rod->addFrame(new Frame1s("RJ"));
  Joint *joint = new Joint("Clamping");
  joint->connect(this->getFrame("I"),rod->getFrame("RJ")); 
  joint->setForceDirection(Mat("[1,0,0;0,1,0;0,0,1]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->setMomentDirection("[0,0; 1,0; 0,1]");
  joint->setMomentLaw(new BilateralConstraint);
  this->addLink(joint);
}

