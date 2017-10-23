#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/observers/contact_observer.h"

#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h"
#include <openmbvcppinterface/polygonpoint.h>

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) :
    DynamicSystemSolver(projectName) {
  
  Vec grav(3, INIT, 0.);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double l0 = 1.5; // length
  double b0 = 0.1; // width
  double E = 5.e7; // E-Modul
  double mu = 0.3; // Poisson ratio
  double G = E / (2 * (1 + mu)); // shear modulus
  double A = b0 * b0; // cross-section area
  double I1 = 1. / 12. * b0 * b0 * b0 * b0; // moment inertia
  double I2 = 1. / 12. * b0 * b0 * b0 * b0; // moment inertia
  double I0 = 0.05 * (I1 + I2);
  double rho = 9.2e2; // density 
  int elements = 2; // number of finite elements

  double mass = 50.; // mass of ball
  double r = 1.e-2; // radius of ball

  FlexibleBody1s33RCM *rod = new FlexibleBody1s33RCM("Rod", true);
  rod->setLength(l0);
  rod->setEGModuls(E, G);
  rod->setCrossSectionalArea(A);
  rod->setMomentsInertia(I1, I2, I0);
  rod->setDensity(rho);
  rod->setFrameOfReference(getFrameI());
  rod->setNumberElements(elements);
//  rod->setCuboid(b0, b0);

  Vec q0 = Vec(10 * elements + 6, INIT, 0.);
  for (int i = 1; i <= elements; i++)
    q0(10 * i) = l0 * i / elements;
  rod->setq0(q0);
  this->addObject(rod);

  std::shared_ptr<OpenMBV::SpineExtrusion> cuboid = OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(elements*4+1); // resolution of visualisation
  cuboid->setDiffuseColor(0.6666, 0.3333, 0.6666); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  shared_ptr<OpenMBV::PolygonPoint>  corner1 = OpenMBV::PolygonPoint::create(b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner1);
  shared_ptr<OpenMBV::PolygonPoint>  corner2 = OpenMBV::PolygonPoint::create(b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner2);
  shared_ptr<OpenMBV::PolygonPoint>  corner3 = OpenMBV::PolygonPoint::create(-b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner3);
  shared_ptr<OpenMBV::PolygonPoint>  corner4 = OpenMBV::PolygonPoint::create(-b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);

  Contour1sFlexible *neutral = new Contour1sFlexible("Neutral");
  rod->addContour(neutral);

  FlexibleBand *top = new FlexibleBand("Top");
  Vec nodes(elements+1);
  for(int i=0;i<=elements;i++) nodes(i) = i*l0/elements;
  top->setNodes(nodes);
  top->setWidth(b0);
  Vec2 RrRP;
  RrRP(0) = 0.5*b0;
  top->setRelativePosition(RrRP);
  top->setContourOfReference(neutral);
  rod->addContour(top);

  RigidBody *ball = new RigidBody("Ball");
  Vec WrOS0B(3, INIT, 0.);
  WrOS0B(0) = l0*0.75; WrOS0B(1) = b0*0.5+r; WrOS0B(2) = b0*0.3;
  this->addFrame(new FixedRelativeFrame("B", WrOS0B, SqrMat(3, EYE), this->getFrame("I")));
  ball->setFrameOfReference(this->getFrame("B"));
  ball->setFrameForKinematics(ball->getFrame("C"));
  ball->setMass(mass);
  SymMat Theta(3);
  Theta(0, 0) = 2. / 5. * mass * r * r;
  Theta(1, 1) = 2. / 5. * mass * r * r;
  Theta(2, 2) = 2. / 5. * mass * r * r;
  ball->setInertiaTensor(Theta);
  ball->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  Vec BR(3,INIT,0.); BR(1)=-r;
  ball->addFrame(new FixedRelativeFrame("P", BR, SqrMat(3, EYE), ball->getFrame("C")));
  ball->addContour(new Point("Point", ball->getFrame("P")));
  this->addObject(ball);

  std::shared_ptr<OpenMBV::Sphere> sphere = OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
  sphere->setRadius(r);
  sphere->setDiffuseColor(0.6666, 1, 0.6666); // color in (minimalColorValue, maximalColorValue)
  ball->setOpenMBVRigidBody(sphere);

  Contact *contact = new Contact("Contact");
  contact->setNormalForceLaw(new UnilateralConstraint);
  contact->setNormalImpactLaw(new UnilateralNewtonImpact(1.0));
  contact->connect(ball->getContour("Point"), rod->getContour("Top"));

  this->addLink(contact);

  ContactObserver *observer = new ContactObserver(contact->getName()+"_Observer");
  addObserver(observer);
  observer->setContact(contact);
  observer->enableOpenMBVNormalForce();
  observer->enableOpenMBVTangentialForce();
  observer->enableOpenMBVContactPoints(0.1);

  rod->addFrame(new Frame1s("RJ"));
  Joint *joint = new Joint("Clamping");
  joint->connect(this->getFrame("I"), rod->getFrame("RJ"));
  joint->setForceDirection(Mat(3, 3, EYE));
  joint->setForceLaw(new BilateralConstraint);
  joint->setMomentDirection(Mat(3, 3, EYE));
  joint->setMomentLaw(new BilateralConstraint);
  this->addLink(joint);

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
}
