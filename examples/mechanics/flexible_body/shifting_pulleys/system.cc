#include "system.h"
#include "cvt_reference_curve.h"

#include <mbsimFlexibleBody/flexible_body/flexible_body_1S_reference_curve.h>

using namespace fmatvec;
using namespace MBSim;
using namespace MBSimFlexibleBody;

ALETester::ALETester(const std::string & name_) :
    DynamicSystemSolver(name_) {

  flushEvery = 1;

  int elements = 1; // use number of finite elements (1 means: only reference movement, if additional dofs are locked properly)
  int elementOrder = 3;
  int nodeDofs = 2;
  bool useReferenceKinematics = false;

  // Set the DOFs that should be locked
  std::set<int> lockedDofs;
  int firstDof = 2 + 0 * 4;

  // Hermite polynoms:
  lockedDofs.insert(firstDof);
  lockedDofs.insert(firstDof + 1);
  lockedDofs.insert(firstDof + 2);
  lockedDofs.insert(firstDof + 3);

// lock tangential direction
//      for (int i = 2; i < 2 + elements * 4; i += 2) {
//        lockedDofs.insert(i);
//      }


  double width = 2;
  double height = 0.01;
  double length = 1;
  double dA = 0.2;
  double rho = 7700;
  double E = 2.1e11;

  CVTReferenceSurfaceParts * ref = new CVTReferenceSurfaceParts();
  ref->setConstraints(length, dA, 0., -0.45, 0.45);

  ring = new FlexibleBody1SReferenceCurve("Ring", ref);
  addObject(ring);

  ring->setPlotFeature(rightHandSide, true);

  ring->setLength(length);
  ring->setDensity(rho);
  ring->setCrossSectionalArea(width * height);
  ring->setEModul(E);
  ring->setIn(width * pow(height, 3.) / 12.);
  ring->setdPulley(dA);
  ring->setNumberElements(elements);
  ring->setElementOrder(elementOrder);
  ring->setUseSpatialReferenceKinematics(useReferenceKinematics);
  ring->setNodeDofs(nodeDofs);
  ring->setLockedDofs(lockedDofs);
  ring->setMassProportionalDamping(100.);

  if (nodeDofs == 3) {
    ring->setNu(0.3);
    ring->setIb(height * pow(width, 3.) / 12.);
    ring->setIt(0.32 * width * height);
  }

  Contour1sNeutralFlexibleBody1SReferenceCurve* ncc = ring->createNeutralPhase("NeutralPhase");
  std::shared_ptr < OpenMBV::SpineExtrusion > cuboid = OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(max(100, elements * elementOrder * 10 + 1)); // resolution of visualisation
  cuboid->setDiffuseColor(0, 0, 1); // h=0, s=0, v=0 --> black
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr < vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  double widthFactor = 0.5;
  shared_ptr < OpenMBV::PolygonPoint > corner1 = OpenMBV::PolygonPoint::create(height * 0.5, width * widthFactor, 1);
  rectangle->push_back(corner1);
  shared_ptr < OpenMBV::PolygonPoint > corner2 = OpenMBV::PolygonPoint::create(height * 0.5, -width * widthFactor, 1);
  rectangle->push_back(corner2);
  shared_ptr < OpenMBV::PolygonPoint > corner3 = OpenMBV::PolygonPoint::create(-height * 0.5, -width * widthFactor, 1);
  rectangle->push_back(corner3);
  shared_ptr < OpenMBV::PolygonPoint > corner4 = OpenMBV::PolygonPoint::create(-height * 0.5, width * widthFactor, 1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  ncc->setOpenMBVSpineExtrusion(cuboid);

  int qSize = 2 + elements * (nodeDofs * 2) - lockedDofs.size(); // for C1 - hermite polynoms
//      int qSize = 2 + (elements) * nodeDofs * (elementOrder) - lockedDofs.size(); // for positional elements (only pos DoFs)
  Vec q0 = Vec(qSize, INIT, 0.);
  q0(1) = 0.4; // intial ratio

  // add some deflection in one direction
  if (0) {
    double add = 1e-4;

    int locked = 0;
    for (int i = 2; i < q0.size(); i++) {
      if (std::find(lockedDofs.begin(), lockedDofs.end(), i) == lockedDofs.end()) {
        // if global the global DOF direction is not locked
        if ((i - 2) % 4 == 1) {
          // if the DOF direction is the normal (=1) direction
          q0(i - locked) = add;
//            add *= -1;
        }
      }
      else {
        // for every DoFs that has been found locked this has to be taken into account when setting the normal deflection...
        locked++;
      }
    }
  }

  if (0) {
    q0(2) = 1e-3;
  }

  Vec u0 = Vec(qSize, INIT, 0.);
//      u0(0) = -10; // intial tangential speed

  ring->setq0(q0);
  ring->setu0(u0);

  // print frames
  if (0) {
    for (int i = 0; i < elements / 2 + 1; i++) {
      MBSimFlexibleBody::NodeFrame * frame = new MBSimFlexibleBody::NodeFrame("Frame" + toString(i), i);
      ring->addFrame(frame);
      frame->enableOpenMBV(1e-1);
    }
  }
}

