#include "block_compression.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"

#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/functions/kinetic_functions.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/symbolic_function.h"
#include "mbsim/functions/nested_function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/polygonpoint.h>
#endif

#include <mbsim/utils/eps.h>

#include <mbsim/utils/stopwatch.h>

#include <mbsimFlexibleBody/contours/flexible_band.h>

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;
using namespace casadi;

class Rod : public FlexibleBody1s21RCM {
  public:
    Rod(std::string name) :
        FlexibleBody1s21RCM(name, true) {
    }

    virtual void facLLM(int i = 0) {
      FlexibleBody1s21RCM::facLLM(i);
    }
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
//    MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

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

#ifdef HAVE_OPENMBVCPPINTERFACE
    boost::shared_ptr<OpenMBV::SpineExtrusion> cuboid = OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
    cuboid->setNumberOfSpinePoints(elements * 4); // resolution of visualisation
    cuboid->setDiffuseColor(1 / 3.0, 1, 1); // color in (minimalColorValue, maximalColorValue)
    cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
    boost::shared_ptr<vector<boost::shared_ptr<OpenMBV::PolygonPoint> > > rectangle = boost::make_shared<vector<boost::shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
    boost::shared_ptr<OpenMBV::PolygonPoint> corner1 = OpenMBV::PolygonPoint::create(b0 * 0.5, w0 * 0.5, 1);
    rectangle->push_back(corner1);
    boost::shared_ptr<OpenMBV::PolygonPoint> corner2 = OpenMBV::PolygonPoint::create(b0 * 0.5, -w0 * 0.5, 1);
    rectangle->push_back(corner2);
    boost::shared_ptr<OpenMBV::PolygonPoint> corner3 = OpenMBV::PolygonPoint::create(-b0 * 0.5, -w0 * 0.5, 1);
    rectangle->push_back(corner3);
    boost::shared_ptr<OpenMBV::PolygonPoint> corner4 = OpenMBV::PolygonPoint::create(-b0 * 0.5, w0 * 0.5, 1);
    rectangle->push_back(corner4);

    cuboid->setContour(rectangle);
    rod->setOpenMBVSpineExtrusion(cuboid);
#endif

    double ringPrestressing = 10.; // Force initially acting on the rings
    double epstL = 100; // Leht Damping in elongation direction
    double mpL = 100; // Damping in all masses direction
    startElongationOfBeam = ringPrestressing * l0 / E / A;

    rod->setCurlRadius(0);
    rod->setLehrDamping(epstL);
    rod->setMassProportionalDamping(mpL);

    //Frames
    ringStartFrame = new Frame("Ring Start");
    rod->addFrame(ringStartFrame, 0);
    ringStartFrame->enableOpenMBV(0.001);

    ringEndFrame = new Frame("Ring End");
    rod->addFrame(ringEndFrame, l0);
    ringEndFrame->enableOpenMBV(0.001);
    //Contours
    FlexibleBand * top = new FlexibleBand("Top");
    rod->addContour(top);

    FlexibleBand * bottom = new FlexibleBand("Bottom");
    rod->addContour(bottom);

    /* cuboid */
    top->setCn(Vec("[1.;0.]"));
    bottom->setCn(Vec("[-1.;0.]"));

    top->setAlphaStart(0.);
    top->setAlphaEnd(l0);

    bottom->setAlphaStart(0.);
    bottom->setAlphaEnd(l0);

    Vec contourNodes(elements + 1);
    for (int i = 0; i <= elements; i++)
      contourNodes(i) = l0 / elements * i;
    top->setNodes(contourNodes);
    bottom->setNodes(contourNodes);

    top->setWidth(w0);  // adapted for double width
    bottom->setWidth(w0);
    top->setNormalDistance(0.5 * b0);
    bottom->setNormalDistance(0.5 * b0);

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

void BlockCompression::plot(double t, double dt) {
  DynamicSystemSolver::plot(t,dt);

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
  leftJointBody->getFrameC()->enableOpenMBV(0.01);

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
  rightJointBody->getFrameC()->enableOpenMBV(0.01);

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
  clampinglastElementFrame->enableOpenMBV(0.01);
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
    string name = "Block" + numtostr(i);
    blocks.push_back(new Block(name));
    blocks[i]->width = width;
    blocks[i]->height = height;
    blocks[i]->thickness = dxElement;
    blocks[i]->groove = groove;
    blocks[i]->setMass(mass);
    blocks[i]->setInertiaTensor(Theta);
    addObject(blocks[i]);
  }

  for (int i = 1; i < NoBlocks - 1; i++) { // do not let the blocks rotate at the start or the end (joints are used only to measre the force...)
    Vec trans(3, INIT, 0);
    string name = "Block" + numtostr(i);
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

  Contact * BlBlBottom = new Contact("BottomContacts");
  BlBlBottom->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlBlBottom, 0.)));
  addLink(BlBlBottom);
  BlBlBottom->enableOpenMBVNormalForce(1e-3);

  Contact * BlBlTop = new Contact("TopContacts");
  BlBlTop->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlBlTop, 0.)));
  addLink(BlBlTop);
  BlBlTop->enableOpenMBVNormalForce(1e-3);

  Contact * BlRod = new Contact("BlockRodContacts");
  BlRod->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cBlRod, 0.)));
  addLink(BlRod);
  BlRod->enableOpenMBVNormalForce(1e-3);

  for (int i = 1; i < NoBlocks; i++) {
    /*(Inter-)Contact between Blocks*/
    BlBlBottom->connect(blocks[i]->bottom, blocks[i - 1]->back);
    BlBlTop->connect(blocks[i]->top, blocks[i - 1]->back);
  }
  for (int i = 0; i < NoBlocks; i++) {
    /*Contact between block and rod*/
    BlRod->connect(blocks[i]->saddle, rod->getContour("Bottom"));
    BlRod->connect(blocks[i]->ear, rod->getContour("Top"));
  }
}

void BlockCompression::clampRod() {
  /*Clamping of the ring to the first and last element (all three directions)*/

//start point is fixed to first element
  Joint* clampingRingStart = new Joint("Clamping Ring Start");
  clampingRingStart->setForceDirection(Mat("[1,0;0,1;0,0]"));
  clampingRingStart->setForceLaw(new BilateralConstraint());

  clampingRingStart->connect(leftJointBody->getFrame("clampingRingStartFrame"), ringStartFrame);
  leftJointBody->getFrame("clampingRingStartFrame")->enableOpenMBV(0.01);
  addLink(clampingRingStart);

//start point is fixed to first element
  Joint* clampingRingEnd = new Joint("Clamping Ring End");
  clampingRingEnd->setForceDirection(Mat("[1,0;0,1;0,0]"));
  clampingRingEnd->setForceLaw(new BilateralConstraint());

  clampingRingEnd->connect(rightJointBody->getFrame("clampingRingEndFrame"), ringEndFrame);
  rightJointBody->getFrame("clampingRingEndFrame")->enableOpenMBV(0.01);
  addLink(clampingRingEnd);

//TODO: Maybe it is necessary to clamp the ring moment wise --> that would be bad for the QS!
}

void BlockCompression::addBoundaryConditions() {

  /*Bearing for the last element to the endBody*/
  Joint* jointClampingLastElement = new Joint("Clamping Last Element");
  jointClampingLastElement->setForceDirection(Mat("[1,0;0,1;0,0]"));

  jointClampingLastElement->setForceLaw(new BilateralConstraint());

  jointClampingLastElement->connect(rightJointBody->getFrameC(), blocks[0]->getFrameC());
//  addLink(jointClampingLastElement);

  /*Movement of the first element*/
//Set translation of dummy start body into x (and not z) direction
  RigidBody * compressionBody = new RigidBody("Compression Body");

  SX t = SX::sym("t");
  double startPos = hingeDistanceLeft + l0 - endShift - (NoBlocks - 1) * dxElement;
  double velocity = 10;
  SX fexp2 = startPos + velocity * t;
  SXFunction foo2(t, fexp2);

  SymbolicFunction<double(double)> *f2 = new SymbolicFunction<double(double)>(foo2);
  compressionBody->setTranslation(new NestedFunction<Vec3(double(double))>(new TranslationAlongXAxis<double>(), f2));

  compressionBody->setMass(1e-6);
  compressionBody->setInertiaTensor(1e-6 * SymMat(3, EYE));

  compressionBody->setFrameOfReference(leftJointBody->getFrameC());
  addObject(compressionBody);
  blocks[blocks.size() - 1]->setFrameOfReference(compressionBody->getFrameC());

//  Joint * jointClampingFirstElement = new Joint("Clamping First Element");
//  jointClampingFirstElement->connect(compressionBody->getFrameC(), blocks[blocks.size() - 1]->getFrameC());
//  jointClampingFirstElement->setForceDirection(Mat("[1,0;0,1;0,0]"));
//
//  jointClampingFirstElement->setForceLaw(new BilateralConstraint());
//
//  addLink(jointClampingFirstElement);
}

