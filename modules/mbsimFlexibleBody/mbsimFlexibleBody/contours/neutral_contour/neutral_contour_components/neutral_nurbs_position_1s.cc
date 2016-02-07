/*
 * NeutralPosition1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_position_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/node_frame.h"

using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {
//  class FlexibleBodyContinuum<double>;

  NeutralNurbsPosition1s::NeutralNurbsPosition1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_) :
      NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_) {
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsPosition1s::~NeutralNurbsPosition1s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsPosition1s::update(ContourPointData &cp) {
    Vec3 Tmpv = curve.pointAt(cp.getLagrangeParameterPosition()(0));
    cp.getFrameOfReference().setPosition(Tmpv);
  }

  // TODO: this Normal and secondTangent is only work for the neutral curve on the xy plane. Need to adapt to different situations.
  void NeutralNurbsPosition1s::updatePositionNormal(ContourPointData &cp) {
    Vec3 n = crossProduct(binormalDir, tangent(cp));
    cp.getFrameOfReference().getOrientation().set(0, n);
  }

  void NeutralNurbsPosition1s::updatePositionFirstTangent(ContourPointData &cp) {
    cp.getFrameOfReference().getOrientation().set(1, tangent(cp));
  }

  void NeutralNurbsPosition1s::updatePositionSecondTangent(ContourPointData &cp) {
    Vec3 t = tangent(cp);
    Vec3 n = crossProduct(binormalDir, t);
    Vec3 b = crossProduct(t, n);
    cp.getFrameOfReference().getOrientation().set(2, b);
  }

  void NeutralNurbsPosition1s::buildNodelist(double t) {
    NodeFrame frame;
    for (int i = 0; i < nodes.size(); i++) {
      frame.setNodeNumber(nodes(i));
//      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame, Frame::position);
      Nodelist.set(i, trans(frame.getPosition(t)));
    }
//    cout << "neutralPosition"<< Nodelist << endl << endl;
  }

  Vec3 NeutralNurbsPosition1s::tangent(const ContourPointData & cp) {
    Vec3 t = curve.firstDn(cp.getLagrangeParameterPosition()(0));
    return t / nrm2(t);
  }
} /* namespace MBSimFlexibleBody */
