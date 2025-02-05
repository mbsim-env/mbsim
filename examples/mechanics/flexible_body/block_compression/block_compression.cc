#include "block_compression.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"

#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/symbolic_function.h"
#include "mbsim/functions/composite_function.h"
#include "mbsim/observers/contact_observer.h"

#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/polygonpoint.h>

#include <mbsim/utils/eps.h>

#include <mbsim/utils/stopwatch.h>

#include <mbsimFlexibleBody/frames/frame_1s.h>
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include <mbsimFlexibleBody/contours/flexible_band.h>

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Rod : public FlexibleBody1s21RCM {
  public:
    Rod(std::string name) :
        FlexibleBody1s21RCM(name, true) {
    }

    virtual void updateLLM() {
      FlexibleBody1s21RCM::updateLLM();
    }

    void updateGyroscopicAccelerations(Frame1s* frame) override { }
};

BlockCompression::BlockCompression(const string &projectName) :
    DynamicSystemSolver(projectName) {

  if (0) {
    Block * bl = new Block("TestBlock");
    bl->setMass(1);
    bl->setInertiaTensor(SymMat3(EYE));
    bl->width = 1;
    bl->height = 1;
    bl->thickness = 0.5;
    bl->groove = 0.1;
//    bl->setTranslation(new TranslationAlongAxesXY<VecV>);
//    bl->setRotation(new RotationAboutZAxis<VecV>);
    addObject(bl);

    bl = new Block("TestBlock2");
    bl->setMass(1);
    bl->setInertiaTensor(SymMat3(EYE));
    bl->width = 1;
    bl->height = 1;
    bl->thickness = 0.5;
    bl->groove = 0.1;
//    bl->setTranslation(new TranslationAlongAxesXY<VecV>);
    bl->setRotation(new RotationAboutZAxis<VecV>);
    addObject(bl);
  }
  else {

    hingePointDistance = 300e-3;
    l0 = 100e-3;  // length ring
    hingeDistanceLeft = 50e-3;
    endShift = 5e-3;

    // acceleration of gravity
    Vec grav(3, INIT, 0.);
    grav(1) = -9.81;
    //getMBSimEnvironment()->setAccelerationOfGravity(grav);

    // input flexible ring
    double E = 2.1e11;  // E-Modul
    double rho = 7.7e3;  // density
    int elements = 10;  // number of finite elements
    b0 = 1e-3;  // thickness rod
    double w0 = 10e-3;  // width
    double A = w0 * b0;  // cross-section area
    int layers = 9;
    double I = layers * 1. / 12. * b0 / layers * b0 / layers * b0 / layers * w0;  // moment inertia

    // flexible ring
    rod = new Rod("Rod");
    rod->setLength(l0);
    rod->setEModul(E);
    rod->setCrossSectionalArea(A);
    rod->setMomentInertia(I);
    rod->setDensity(rho);
    rod->setFrameOfReference(getFrameI());
    rod->setNumberElements(elements);

    std::shared_ptr<OpenMBV::SpineExtrusion> cuboid = OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
    cuboid->setNumberOfSpinePoints(elements * 4); // resolution of visualisation
    cuboid->setDiffuseColor(1 / 3.0, 1, 1); // color in (minimalColorValue, maximalColorValue)
    cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
    std::shared_ptr<vector<std::shared_ptr<OpenMBV::PolygonPoint> > > rectangle = std::make_shared<vector<std::shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
    std::shared_ptr<OpenMBV::PolygonPoint> corner1 = OpenMBV::PolygonPoint::create(b0 * 0.5, w0 * 0.5, 1);
    rectangle->push_back(corner1);
    std::shared_ptr<OpenMBV::PolygonPoint> corner2 = OpenMBV::PolygonPoint::create(b0 * 0.5, -w0 * 0.5, 1);
    rectangle->push_back(corner2);
    std::shared_ptr<OpenMBV::PolygonPoint> corner3 = OpenMBV::PolygonPoint::create(-b0 * 0.5, -w0 * 0.5, 1);
    rectangle->push_back(corner3);
    std::shared_ptr<OpenMBV::PolygonPoint> corner4 = OpenMBV::PolygonPoint::create(-b0 * 0.5, w0 * 0.5, 1);
    rectangle->push_back(corner4);

    cuboid->setContour(rectangle);
    rod->setOpenMBVSpineExtrusion(cuboid);

    double ringPrestressing = 10.; // Force initially acting on the rings
    double epstL = 100; // Leht Damping in elongation direction
    double mpL = 100; // Damping in all masses direction
    startElongationOfBeam = ringPrestressing * l0 / E / A;

    rod->setCurlRadius(0);
    rod->setLehrDamping(epstL);
    rod->setMassProportionalDamping(mpL);

    //Frames
    ringStartFrame = new Frame1s("Ring Start",0.);
    rod->addFrame(ringStartFrame);
    ringStartFrame->enableOpenMBV(0.001);

    ringEndFrame = new Frame1s("Ring End",l0);
    rod->addFrame(ringEndFrame);
    ringEndFrame->enableOpenMBV(0.001);
    //Contours
    FlexibleBand * top = new FlexibleBand("Top");
    rod->addContour(top);

    FlexibleBand * bottom = new FlexibleBand("Bottom");
    rod->addContour(bottom);

    Contour1sFlexible *neutral = new Contour1sFlexible("Neutral");
    rod->addContour(neutral);

    vector<double> nodes(2);
    nodes[0] = 0.;
    nodes[1] = l0;

    Vec2 RrRP;
    RrRP(0) = 0.5*b0;

    top->setRelativePosition(RrRP);
    bottom->setRelativePosition(-RrRP);

    bottom->setRelativeOrientation(M_PI);

    top->setContourOfReference(neutral);
    bottom->setContourOfReference(neutral);

    top->setNodes(nodes);
    bottom->setNodes(nodes);

    vector<double> contourNodes(elements + 1);
    for (int i = 0; i <= elements; i++)
      contourNodes[i] = l0 / elements * i;
    top->setNodes(contourNodes);
    bottom->setNodes(contourNodes);

    top->setWidth(w0);  // adapted for double width
    bottom->setWidth(w0);

    Vec q0M2D(5 * elements + 3, INIT, 0);

    for (int i = 0; i < 5 * elements + 3; i += 5) {
      q0M2D(i) = i / 5 * (l0 + startElongationOfBeam) / elements + hingeDistanceLeft;
    }

    Vec u0M2D(5 * elements + 3, INIT, 0);
    rod->setq0(q0M2D);
    rod->setu0(u0M2D);

    addObject(rod);
    defineEdges();
    clampRod();
    addBlocks();
    addBoundaryConditions();
    addContacts();
  }
}

void BlockCompression::plot() {
  DynamicSystemSolver::plot();

//  cout << h[0] + W[0]*la << endl;
}

void BlockCompression::defineEdges() {
  /*Add start body to system*/
  leftJointBody = new RigidBody("LeftJointBody");
  leftJointBody->setMass(1e-9);
  leftJointBody->setInertiaTensor(1e-9 * SymMat(3, EYE));
  leftJointBody->setRotation(new RotationAboutZAxis<VecV>); //only rotation
  leftJointBody->setFrameOfReference(getFrameI());
  addObject(leftJointBody);
  leftJointBody->getFrameC()->enableOpenMBV(0.001);

// Add Frame for clamping the start of the ring
  Vec3 translationRingStartFrame;
  translationRingStartFrame(0) = hingeDistanceLeft;
  FixedRelativeFrame * clampingRingStartFrame = new FixedRelativeFrame("clampingRingStartFrame", translationRingStartFrame);
  leftJointBody->addFrame(clampingRingStartFrame);

  /*Add end body to system*/
  rightJointBody = new RigidBody("RightJointBody");
  rightJointBody->setMass(1e-9);
  rightJointBody->setInertiaTensor(1e-9 * SymMat(3, EYE));
  rightJointBody->setRotation(new RotationAboutZAxis<VecV>); //only rotation

  //Create Reference frame for right hinge Point
  FixedRelativeFrame * endFrame = new FixedRelativeFrame("End-Frame");
  Vec translationEndFrame(3, INIT, 0.);
  translationEndFrame(0) = hingePointDistance;
  endFrame->setRelativePosition(translationEndFrame);
  addFrame(endFrame);

  // create right hing point body with rotation
  rightJointBody->setFrameOfReference(endFrame);
  addObject(rightJointBody);
  rightJointBody->getFrameC()->enableOpenMBV(0.001);

// Add Frame for clamping the end of the ring
  FixedRelativeFrame * clampingRingEndFrame = new FixedRelativeFrame("clampingRingEndFrame");
  Vec translationRingEndFrame(3, INIT, 0.);
  translationRingEndFrame(0) = -(hingePointDistance - hingeDistanceLeft - l0 - startElongationOfBeam);
  clampingRingEndFrame->setRelativePosition(translationRingEndFrame);
  rightJointBody->addFrame(clampingRingEndFrame);

// Add Frame for Clamping the last element
  FixedRelativeFrame * clampinglastElementFrame = new FixedRelativeFrame("clampinglastElementFrame");
  Vec translationlastElementFrame(3, INIT, 0.);
  translationlastElementFrame(0) = -hingePointDistance + hingeDistanceLeft + l0 - endShift;
  clampinglastElementFrame->setRelativePosition(translationlastElementFrame);
  clampinglastElementFrame->enableOpenMBV(0.001);
  rightJointBody->addFrame(clampinglastElementFrame);

}

void BlockCompression::addBlocks() {

  NoBlocks = 5;
  dxElement = 5e-3; // i.e. the thickness of the blocks
  height = 30e-3; // height Elements
  width = 10e-3;
  groove = b0;
  mass = 1;

  SymMat3 Theta(EYE);

  for (int i = 0; i < NoBlocks; i++) {
    string name = "Block" + toString(i);
    blocks.push_back(new Block(name));
    blocks[i]->width = width;
    blocks[i]->height = height;
    blocks[i]->thickness = dxElement;
    blocks[i]->groove = groove;
    blocks[i]->setMass(mass);
    blocks[i]->setInertiaTensor(Theta);
    addObject(blocks[i]);
  }

  for (int i = 1; i < NoBlocks; i++) { // do not let the blocks rotate at the start or the end (joints are used only to measre the force...)
    Vec trans(3, INIT, 0);
    string name = "Block" + toString(i);
    trans(0) = hingeDistanceLeft + l0 - endShift - i * dxElement;
    FixedRelativeFrame * frame = new FixedRelativeFrame("Frame_" + name, trans);
    addFrame(frame);
    blocks[i]->setTranslation(new TranslationAlongAxesXY<VecV>);
    blocks[i]->setRotation(new RotationAboutZAxis<VecV>);

    blocks[i]->setFrameOfReference(frame);
  }

  blocks[0]->setFrameOfReference(rightJointBody->getFrame("clampinglastElementFrame"));
// Reference frqame for the last block is the frame of the clamping movement body! --> this one is created later!
}

void BlockCompression::addContacts() {
  double cBlBlBottom = 1e5;
  double cBlBlTop = 5e5;
  double cBlRod = 1e4;

  for (int i = 1; i < NoBlocks; i++) {
  Contact * BlBlBottom = new Contact("BottomContacts_"+to_string(i));
  BlBlBottom->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlBlBottom, 0.)));
  addLink(BlBlBottom);
  ContactObserver *observer = new ContactObserver("BottomContactsObserver_"+to_string(i));
  addObserver(observer);
  observer->setContact(BlBlBottom);
  observer->enableOpenMBVNormalForce(_colorRepresentation=OpenMBVArrow::absoluteValue,_scaleLength=1e-3);
  observer->enableOpenMBVContactPoints(1e-3);

  Contact * BlBlTop = new Contact("TopContacts_"+to_string(i));
  BlBlTop->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlBlTop, 0.)));
  addLink(BlBlTop);
  observer = new ContactObserver("TopContactsObserver_"+to_string(i));
  addObserver(observer);
  observer->setContact(BlBlTop);
  observer->enableOpenMBVNormalForce(_colorRepresentation=OpenMBVArrow::absoluteValue,_scaleLength=1e-3);
  observer->enableOpenMBVContactPoints(1e-3);

    /*(Inter-)Contact between Blocks*/
    BlBlBottom->connect(blocks[i]->bottom, blocks[i - 1]->back);
    BlBlTop->connect(blocks[i]->top, blocks[i - 1]->back);
  }
  for (int i = 0; i < NoBlocks; i++) {
  Contact * BlRodBottom = new Contact("BlockRodContactsBottom_"+to_string(i));
  BlRodBottom->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlRod, 0.)));
  addLink(BlRodBottom);
  ContactObserver *observer = new ContactObserver("BlockRodContactsObserverBottom_"+to_string(i));
  addObserver(observer);
  observer->setContact(BlRodBottom);
  observer->enableOpenMBVNormalForce(_colorRepresentation=OpenMBVArrow::absoluteValue,_scaleLength=1e-3);
  observer->enableOpenMBVContactPoints(1e-3);

  Contact * BlRodTop = new Contact("BlockRodContactsTop_"+to_string(i));
  BlRodTop->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlRod, 0.)));
  addLink(BlRodTop);
  observer = new ContactObserver("BlockRodContactsObserverTop_"+to_string(i));
  addObserver(observer);
  observer->setContact(BlRodTop);
  observer->enableOpenMBVNormalForce(_colorRepresentation=OpenMBVArrow::absoluteValue,_scaleLength=1e-3);
  observer->enableOpenMBVContactPoints(1e-3);

    /*Contact between block and rod*/
    BlRodBottom->connect(blocks[i]->saddle, rod->getContour("Bottom"));
    BlRodTop->connect(blocks[i]->ear, rod->getContour("Top"));
  }
}

void BlockCompression::clampRod() {
  /*Clamping of the ring to the first and last element (all three directions)*/

//start point is fixed to first element
  Joint* clampingRingStart = new Joint("Clamping Ring Start");
  clampingRingStart->setForceDirection(Mat("[1,0;0,1;0,0]"));
  clampingRingStart->setForceLaw(new BilateralConstraint());

  clampingRingStart->connect(leftJointBody->getFrame("clampingRingStartFrame"), ringStartFrame);
  leftJointBody->getFrame("clampingRingStartFrame")->enableOpenMBV(0.001);
  addLink(clampingRingStart);

//start point is fixed to first element
  Joint* clampingRingEnd = new Joint("Clamping Ring End");
  clampingRingEnd->setForceDirection(Mat("[1,0;0,1;0,0]"));
  clampingRingEnd->setForceLaw(new BilateralConstraint());

  clampingRingEnd->connect(rightJointBody->getFrame("clampingRingEndFrame"), ringEndFrame);
  rightJointBody->getFrame("clampingRingEndFrame")->enableOpenMBV(0.001);
  addLink(clampingRingEnd);

//TODO: Maybe it is necessary to clamp the ring moment wise --> that would be bad for the QS!
}

void BlockCompression::addBoundaryConditions() {

  /*Bearing for the last element to the endBody*/
//  Joint* jointClampingLastElement = new Joint("Clamping Last Element");
//  jointClampingLastElement->setForceDirection(Mat("[1,0;0,1;0,0]"));
//
//  jointClampingLastElement->setForceLaw(new BilateralConstraint());
//
//  jointClampingLastElement->connect(rightJointBody->getFrameC(), blocks[0]->getFrameC());
//  addLink(jointClampingLastElement);

  /*Movement of the first element*/
//Set translation of dummy start body into x (and not z) direction
  RigidBody * compressionBody = new RigidBody("Compression Body");

  IndependentVariable t;
  double startPos = hingeDistanceLeft + l0 - endShift - (NoBlocks - 1) * dxElement;
  double velocity = 10;
  SymbolicExpression fexp2 = startPos + velocity * t;

  MBSim::SymbolicFunction<double(double)> *f2 = new MBSim::SymbolicFunction<double(double)>();
  f2->setIndependentVariable(t);
  f2->setDependentFunction(fexp2);
  compressionBody->setTranslation(new CompositeFunction<Vec3(double(double))>(new TranslationAlongXAxis<double>(), f2));

  compressionBody->setMass(1e-6);
  compressionBody->setInertiaTensor(1e-6 * SymMat(3, EYE));

  compressionBody->setFrameOfReference(leftJointBody->getFrameC());
  compressionBody->getFrameC()->enableOpenMBV(0.001);
  addObject(compressionBody);
  blocks[blocks.size() - 1]->setFrameOfReference(compressionBody->getFrameC());

//  Joint * jointClampingFirstElement = new Joint("Clamping First Element");
//  jointClampingFirstElement->connect(compressionBody->getFrameC(), blocks[blocks.size() - 1]->getFrameC());
//  jointClampingFirstElement->setForceDirection(Mat("[1,0;0,1;0,0]"));
//
//  jointClampingFirstElement->setForceLaw(new BilateralConstraint());
//
//  addLink(jointClampingFirstElement);

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
}

