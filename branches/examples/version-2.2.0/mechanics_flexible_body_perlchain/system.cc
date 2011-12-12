#include "system.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/polygonpoint.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // acceleration of gravity
  Vec grav(3,INIT,0.); grav(1) = -9.81;
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
  rod = new FlexibleBody1s21RCM("Rod",false);
  rod->setLength(l0);
  rod->setEModul(E);
  rod->setCrossSectionalArea(A);
  rod->setMomentInertia(I);
  rod->setDensity(rho);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->initRelaxed(M_PI/2.);
  this->addObject(rod);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid=new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements*4); // resolution of visualisation
  cuboid->setStaticColor(0.5); // color in (minimalColorValue, maximalColorValue)
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
    balls[i]->setTranslation(new LinearTranslation("[1,0;0,1;0,0]"));
    balls[i]->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
    balls[i]->setMass(mass);
    SymMat Theta(3,INIT,0.);
    Theta(0,0) = 1./6.*mass*b*b;
    Theta(1,1) = 1./12.*mass*(d*d + b*b);
    Theta(2,2) = 1./12.*mass*(d*d + b*b);
    balls[i]->setInertiaTensor(Theta);
    this->addObject(balls[i]);

    Point *pt = new Point("COG");
    balls[i]->addContour(pt,Vec(3,INIT,0.),SqrMat(3,EYE),balls[i]->getFrame("C"));

    Point *tP = new Point("topPoint");
    balls[i]->addContour(tP,d*Vec("[0.5;0;0]") + b*Vec("[0;0.5;0]"),SqrMat(3,EYE),balls[i]->getFrame("C"));

    Point *bP = new Point("bottomPoint");
    balls[i]->addContour(bP,d*Vec("[0.5;0;0]") - b*Vec("[0;0.5;0]"),SqrMat(3,EYE),balls[i]->getFrame("C"));

    Plane *plane = new Plane("Plane");
    SqrMat trafoPlane(3,INIT,0.); trafoPlane(0,0) = -1.; trafoPlane(1,1) = 1.; trafoPlane(2,2) = -1.;
    balls[i]->addContour(plane,-d*Vec("[0.5;0;0]"),trafoPlane,balls[i]->getFrame("C"));

#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Cuboid *cube=new OpenMBV::Cuboid;
    cube->setLength(d,b,b);
    cube->setStaticColor(1.);
    balls[i]->setOpenMBVRigidBody(cube);
#endif
  }

  // inertial ball constraint
  this->addFrame("BearingFrame",l0/(2*M_PI)*Vec("[0;1;0]"),SqrMat(3,EYE),this->getFrame("I"));
  Joint *joint = new Joint("BearingJoint");
  joint->setForceDirection(Mat("[1,0;0,1;0,0]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->setImpactForceLaw(new BilateralImpact);
  joint->connect(this->getFrame("BearingFrame"),balls[0]->getFrame("C"));
  this->addLink(joint);

  // constraints balls on flexible band
  for(int i=0;i<nBalls;i++) {
    Contact *contact = new Contact("Band_"+balls[i]->getName());
    contact->setContactForceLaw(new BilateralConstraint);
    contact->setContactImpactLaw(new BilateralImpact);
    contact->connect(balls[i]->getContour("COG"),rod->getContour("Contour1sFlexible"));
    this->addLink(contact);
  }

  // inner-ball contacts
  for(int i=0;i<nBalls;i++) {
    stringstream namet,nameb;
    namet << "ContactTop_" << i;
    nameb << "ContactBot_" << i;
    Contact *ctrt = new Contact(namet.str());
    Contact *ctrb = new Contact(nameb.str());
    ctrt->setContactForceLaw(new UnilateralConstraint);
    ctrt->setContactImpactLaw(new UnilateralNewtonImpact(0.));
    ctrb->setContactForceLaw(new UnilateralConstraint);
    ctrb->setContactImpactLaw(new UnilateralNewtonImpact(0.));
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

void System::initialize() {
  DynamicSystemSolver::initialize();
  rod->updateStateDependentVariables(0.);

  for(unsigned int i=0;i<balls.size();i++) {
    Vec q0(3,INIT,0.);
    double xL = fmod(i*rod->getLength()/balls.size() + rod->getLength()*0.25,rod->getLength());
    ContourPointData cp;
    cp.getContourParameterType() = CONTINUUM;
    cp.getLagrangeParameterPosition() = Vec(1,INIT,xL);

    rod->updateKinematicsForFrame(cp,position_cosy);
    q0(0) = cp.getFrameOfReference().getPosition()(0);
    q0(1) = cp.getFrameOfReference().getPosition()(1);
    q0(2) = -AIK2Cardan(cp.getFrameOfReference().getOrientation())(2) + M_PI*0.5;
    balls[i]->setInitialGeneralizedPosition(q0);
  }

}

