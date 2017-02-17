#include "system.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/observers/contact_observer.h"

#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/polygonpoint.h>

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // acceleration of gravity
  Vec grav(3,INIT,0.); 
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // input flexible ring
  double l0 = 1.; // length ring
  double E = 2.5e9; // E-Modul alu
  double rho = 2.5e3; // density alu
  int elements = 20; // number of finite elements
  double b0 = 0.02; // width
  double A = b0*b0; // cross-section area
  double I = 1./12.*b0*b0*b0*b0; // moment inertia

  // input infty-norm balls (cuboids)
  int nBalls = 80; // number of balls
  double mass = 0.025; // mass of ball

  // flexible ring
  rod = new FlexibleBody1s21ANCF("Rod",false);
  rod->setLength(l0);
  rod->setEModul(E);
  rod->setCrossSectionalArea(A);
  rod->setMomentInertia(I);
  rod->setDensity(rho);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->initRelaxed(M_PI/2.);
  this->addObject(rod);

  std::shared_ptr<OpenMBV::SpineExtrusion> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(elements*4); // resolution of visualisation
  cuboid->setDiffuseColor(1/3.0, 1, 1); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  shared_ptr<OpenMBV::PolygonPoint> corner1 = OpenMBV::PolygonPoint::create(b0*0.5,b0*0.5,1);
  rectangle->push_back(corner1);
  shared_ptr<OpenMBV::PolygonPoint> corner2 = OpenMBV::PolygonPoint::create(b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner2);
  shared_ptr<OpenMBV::PolygonPoint> corner3 = OpenMBV::PolygonPoint::create(-b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner3);
  shared_ptr<OpenMBV::PolygonPoint> corner4 = OpenMBV::PolygonPoint::create(-b0*0.5,b0*0.5,1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
  
  Contour1sFlexible *neutral = new Contour1sFlexible("Neutral");
  rod->addContour(neutral);

  FlexibleBand *contour1sFlexible = new FlexibleBand("Contour1sFlexible");
  Vec nodes(elements+1);
  for(int i=0;i<=elements;i++) nodes(i) = i*l0/elements;
  contour1sFlexible->setNodes(nodes);
  contour1sFlexible->setWidth(0.1);
  contour1sFlexible->setContourOfReference(neutral);
  rod->addContour(contour1sFlexible);

  // balls
  assert(nBalls>1);
  double d = 7.*l0/(8.*nBalls); // thickness
  double b = b0*1.5; // height / width

  for(int i=0;i<nBalls;i++) {
    stringstream name;
    name << "Element_" << i;
    RigidBody *ball = new RigidBody(name.str());
    balls.push_back(ball);
    balls[i]->setFrameOfReference(this->getFrame("I"));
    balls[i]->setFrameForKinematics(balls[i]->getFrame("C"));
    balls[i]->setTranslation(new TranslationAlongAxesXY<VecV>);
    balls[i]->setRotation(new RotationAboutZAxis<VecV>);
    balls[i]->setMass(mass);
    SymMat Theta(3,INIT,0.);
    Theta(0,0) = 1./6.*mass*b*b;
    Theta(1,1) = 1./12.*mass*(d*d + b*b);
    Theta(2,2) = 1./12.*mass*(d*d + b*b);
    balls[i]->setInertiaTensor(Theta);
    this->addObject(balls[i]);

    Point *pt = new Point("COG");
    balls[i]->addContour(pt);

    Point *tP = new Point("topPoint");
    balls[i]->addFrame(new FixedRelativeFrame("topPoint",d*Vec("[0.5;0;0]") + b*Vec("[0;0.5;0]"),SqrMat(3,EYE),balls[i]->getFrame("C")));
    tP->setFrameOfReference(balls[i]->getFrame("topPoint"));
    balls[i]->addContour(tP);

    Point *bP = new Point("bottomPoint");
    balls[i]->addFrame(new FixedRelativeFrame("bottomPoint",d*Vec("[0.5;0;0]") - b*Vec("[0;0.5;0]"),SqrMat(3,EYE),balls[i]->getFrame("C")));
    bP->setFrameOfReference(balls[i]->getFrame("bottomPoint"));
    balls[i]->addContour(bP);

    Plane *plane = new Plane("Plane");
    SqrMat trafoPlane(3,INIT,0.); trafoPlane(0,0) = -1.; trafoPlane(1,1) = 1.; trafoPlane(2,2) = -1.;
    balls[i]->addFrame(new FixedRelativeFrame("Plane",-d*Vec("[0.5;0;0]"),trafoPlane,balls[i]->getFrame("C")));
    plane->setFrameOfReference(balls[i]->getFrame("Plane"));
    balls[i]->addContour(plane);

    std::shared_ptr<OpenMBV::Cuboid> cube=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
    cube->setLength(d,b,b);
    cube->setDiffuseColor(0, 1, 1);
    balls[i]->setOpenMBVRigidBody(cube);
  }

  //Set balls to correct position
  FlexibleBody1s21ANCF * rodInfo = new FlexibleBody1s21ANCF("InfoRod", false);

  rodInfo->setq0(rod->getq());
  rodInfo->setu0(rod->getu());
  rodInfo->setNumberElements(rod->getNumberElements());
  rodInfo->setLength(rod->getLength());
  rodInfo->setFrameOfReference(rod->getFrameOfReference());

  rodInfo->initInfo();
  //rodInfo->updateStateDependentVariables(0.);

  for(unsigned int i=0;i<balls.size();i++) {
    Vec q0(3,INIT,0.);
    double xL = fmod(i*rodInfo->getLength()/balls.size() + rodInfo->getLength()*0.25,rodInfo->getLength());
  //  ContourPointData cp;
  //  cp.getContourParameterType() = ContourPointData::continuum;
  //  cp.getLagrangeParameterPosition()(0) = xL;

  //  rodInfo->updateKinematicsForFrame(cp,Frame::position_cosy);
    Vec3 r = rodInfo->getPosition(xL);
    q0(0) = r(0);
    q0(1) = r(1);

    SqrMat3 A = rodInfo->getOrientation(xL);
    q0(2) = fmod(AIK2Cardan(A)(2)+M_PI,2*M_PI);
    balls[i]->setGeneralizedInitialPosition(q0);
  }

  delete rodInfo;

  // inertial ball constraint
  this->addFrame(new FixedRelativeFrame("BearingFrame",l0/(2*M_PI)*Vec("[0;1;0]"),SqrMat(3,EYE),this->getFrame("I")));
  Joint *joint = new Joint("BearingJoint");
  joint->setForceDirection(Mat("[1,0;0,1;0,0]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->connect(this->getFrame("BearingFrame"),balls[0]->getFrame("C"));
  this->addLink(joint);

  // constraints balls on flexible band
  for(int i=0;i<nBalls;i++) {
    Contact *contact = new Contact("Band_"+balls[i]->getName());
    contact->setNormalForceLaw(new BilateralConstraint);
    contact->setNormalImpactLaw(new BilateralImpact);
    contact->connect(balls[i]->getContour("COG"),rod->getContour("Contour1sFlexible"));
    contact->setSearchAllContactPoints(true);
    this->addLink(contact);

    ContactObserver *observer = new ContactObserver(contact->getName()+"_Observer");
    addObserver(observer);
    observer->setContact(contact);
    observer->enableOpenMBVContactPoints(0.01);
  }

  // inner-ball contacts
  for(int i=0;i<nBalls;i++) {
    stringstream namet,nameb;
    namet << "ContactTop_" << i;
    nameb << "ContactBot_" << i;
    Contact *ctrt = new Contact(namet.str());
    Contact *ctrb = new Contact(nameb.str());
    ctrt->setNormalForceLaw(new UnilateralConstraint);
    ctrt->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    ctrb->setNormalForceLaw(new UnilateralConstraint);
    ctrb->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    if(i==nBalls-1) {
      ctrt->connect(balls[0]->getContour("topPoint"),balls[i]->getContour("Plane"));
      ctrb->connect(balls[0]->getContour("bottomPoint"),balls[i]->getContour("Plane"));
    }
    else {
      ctrt->connect(balls[i+1]->getContour("topPoint"),balls[i]->getContour("Plane"));
      ctrb->connect(balls[i+1]->getContour("bottomPoint"),balls[i]->getContour("Plane"));
    }
    this->addLink(ctrt);
    this->addLink(ctrb);
  }
}
