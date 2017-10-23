#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/circle.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/observers/contact_observer.h"

#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h"
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h>

#include <iostream>

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

// global environment
  Vec grav(3);//"[0.0;-9.81;0.0]");
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

// belt globals
//  const double l0 = 4.0; // length
  const double b0_contact = 0.0; // width for contact
  const double b0         = 5.0e-3; // width for visualisation and stiffness
  const double E = 5.e7; // E-Modul  
  const double A = b0*b0; // cross-section area
  const double I1 = 1./12.*b0*b0*b0*b0; // moment inertia
  const double rho = 1.5e3; // density  
  const int elements = 36; // number of finite elements

  const double mu = 0.2;
  const double nodesPerElement = 0.25;

  const double v0 = 1.0;
  const double F0 = 0.0;

// disk globals
  const Vec        massDisks = 1e-3*Vec("[1800]"); // masses of disks
  const Vec       radiiDisks = 1e-3*Vec("[180]"); // radius of disks
  const int           nDisks = massDisks.size();
        VecInt sideInOut(nDisks);
        //Vec    inertiasDisks = Vec(nDisks,INIT,0.0);      // radius of ball
        Mat    positionDisks = Mat(3,nDisks);    // radius of ball
  assert(nDisks == radiiDisks.size() && nDisks == massDisks.size());

  positionDisks.col(0) = Vec("[0.00;0.0;0.0]"); sideInOut(0) = -1; // 1: inside; -1:outside
//  positionDisks.col(1) = Vec("[0.50;0.0;0.0]"); sideInOut(1) = -1; // 1: inside; -1:outside
//  positionDisks.col(2) = Vec("[0.25;0.3;0.0]"); sideInOut(2) = -1; // 1: inside; -1:outside

// frames for disk axes
  Vec mid(3,INIT,0.0);
  for(int i=0;i<nDisks;i++) mid += positionDisks.col(i)/double(nDisks);
  this->addFrame(new FixedRelativeFrame("OB",mid,SqrMat(3,EYE),this->getFrame("I")));

  double inc = 0.021;
  double off = -0.08;
  double lengthPath = 2*M_PI*radiiDisks(0)*(1.0+inc);
  this->addFrame(new FixedRelativeFrame("off",radiiDisks(0)*0*inc*(1.0-off)*Vec("[-0.5;0;0]"),SqrMat(3,EYE),this->getFrame("I")));

  double beltLength = lengthPath * (1.0 - F0/(E*A));
// implementation
  FlexibleBody1s21RCM *belt = new FlexibleBody1s21RCM("Belt", false);
  belt->setLength(beltLength);
  belt->setEModul(E);
  belt->setCrossSectionalArea(A);
  belt->setMomentInertia(I1);
  belt->setDensity(rho);
  belt->setFrameOfReference(this->getFrame("off"));
  belt->setNumberElements(elements);
  belt->initRelaxed(0.0);
//  belt->setInitialGeneralizedPosition(q0);
  this->addObject(belt);

  std::shared_ptr<OpenMBV::SpineExtrusion> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(5*elements+1); // resolution of visualisation
  cuboid->setDiffuseColor(0.6666,1,0.6666); 
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  double h0 = 50.0e-3;
  double b0_vis = 1.0e-3;
  shared_ptr<OpenMBV::PolygonPoint>  corner1 = OpenMBV::PolygonPoint::create( b0_vis*0.5, h0*0.5,1);
  rectangle->push_back(corner1);
  shared_ptr<OpenMBV::PolygonPoint>  corner2 = OpenMBV::PolygonPoint::create( b0_vis*0.5,-h0*0.5,1);
  rectangle->push_back(corner2);
  shared_ptr<OpenMBV::PolygonPoint>  corner3 = OpenMBV::PolygonPoint::create(-b0_vis*0.5,-h0*0.5,1);
  rectangle->push_back(corner3);
  shared_ptr<OpenMBV::PolygonPoint>  corner4 = OpenMBV::PolygonPoint::create(-b0_vis*0.5, h0*0.5,1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  belt->setOpenMBVSpineExtrusion(cuboid);

  Contour1sFlexible *neutral = new Contour1sFlexible("Neutral");
  belt->addContour(neutral);

  FlexibleBand *top = new FlexibleBand("Top");
  int nNodes = (int)(nodesPerElement*elements + 1);
  Vec nodes(nNodes);
  for(int i=0;i<nNodes;i++) nodes(i) = i*beltLength/(nNodes-1);
  top->setNodes(nodes);
  cout << nodes << endl;
  top->setWidth(0.1);
  Vec2 r;
  r(0) = -0.5*b0_contact;
  top->setRelativePosition(r);
  top->setContourOfReference(neutral);
  belt->addContour(top);

  for(int i=0;i<nDisks;i++) {
    stringstream name;
    name.clear();
    name << "Disk" << i; 
    RigidBody *disk = new RigidBody(name.str());

    name.clear();
    name << "B" << i;
    this->addFrame(new FixedRelativeFrame(name.str(),positionDisks.col(i),SqrMat(3,EYE),this->getFrame("I")));
    disk->setFrameOfReference(this->getFrame(name.str()));
    disk->setFrameForKinematics(disk->getFrame("C"));
    disk->setMass(massDisks(i));
    SymMat Theta(3);
    Theta(0,0) = 2./5.*massDisks(i)*radiiDisks(i)*radiiDisks(i);
    Theta(1,1) = 2./5.*massDisks(i)*radiiDisks(i)*radiiDisks(i);
    Theta(2,2) = 2./5.*massDisks(i)*radiiDisks(i)*radiiDisks(i);
    disk->setInertiaTensor(Theta);
//    Mat JacTrans(3,2,INIT,0.);
//    JacTrans(0,0) = 1.;
//    JacTrans(1,1) = 1.;
//    disk->setTranslation(new LinearTranslation(JacTrans));
    disk->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1.0]")));

    Circle *cDisk = new Circle("cDisk");
    cDisk->setRadius(radiiDisks(i));
    disk->addContour(cDisk);
    disk->setGeneralizedInitialVelocity(-Vec("[1.0]")*v0*double(sideInOut(i))/radiiDisks(i));
    cDisk->enableOpenMBV();
    this->addObject(disk);

    //  ContactKinematicsSolidCircleFlexibleBand *ck = new ContactKinematicsSolidCircleFlexibleBand();
    name.clear();
    name << "Contact" << i;
    Contact *contact = new Contact(name.str());
    //  contact->setContactKinematics(ck);
    //contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(c,d)));
    cout << name.str() << " is ";
    contact->setNormalForceLaw(new UnilateralConstraint);
    contact->setNormalImpactLaw(new UnilateralNewtonImpact(.0));
    contact->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    contact->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
    contact->connect(disk->getContour("cDisk"),belt->getContour("Top"));
    //contact->setPlotFeature(linkLagrangeParameters, enabled);
    this->addLink(contact);

    ContactObserver *observer = new ContactObserver(name.str()+"_Observer");
    addObserver(observer);
    observer->setContact(contact);
    observer->enableOpenMBVNormalForce(_scaleLength=0.01);
    observer->enableOpenMBVTangentialForce(_scaleLength=0.01);
    observer->enableOpenMBVContactPoints(0.001,true);
  }

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
}
