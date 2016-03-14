/*
 * NeutralPosition1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_position_1s.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/frames/node_frame.h"

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

  Vec3 NeutralNurbsPosition1s::getPosition(double t, double s) {
    if(updCurve) computeCurve(t,true);
    return curve.pointAt(s);
  }

  void NeutralNurbsPosition1s::update(double t, ContourFrame *frame) {
    if(updCurve) computeCurve(t,true);
    frame->setPosition(curve.pointAt(frame->getEta()));
  }

  // TODO: this Normal and secondTangent is only work for the neutral curve on the xy plane. Need to adapt to different situations.
  void NeutralNurbsPosition1s::updatePositionNormal(double t, ContourFrame *frame) {
    Vec3 n = crossProduct(binormalDir, getWs(t,frame->getZeta()));
    frame->getOrientation(false).set(0, n);
  }

  void NeutralNurbsPosition1s::updatePositionFirstTangent(double t, ContourFrame *frame) {
    frame->getOrientation(false).set(1, getWs(t,frame->getZeta()));
  }

  void NeutralNurbsPosition1s::updatePositionSecondTangent(double time, ContourFrame *frame) {
    Vec3 t = getWs(time,frame->getZeta());
    Vec3 n = crossProduct(binormalDir, t);
    Vec3 b = crossProduct(t, n);
    frame->getOrientation(false).set(2, b);
  }

  void NeutralNurbsPosition1s::buildNodelist(double t) {
    for (int i = 0; i < nodes.size(); i++) {
      NodeFrame P("P",nodes(i));
      P.setParent(parent);
      Nodelist.set(i, trans(P.getPosition(t)));
    }
//    cout << "neutralPosition"<< Nodelist << endl << endl;
  }

  Vec3 NeutralNurbsPosition1s::getWs(double time, const Vec2 &zeta) {
    Vec3 t = curve.firstDn(zeta(0));
    return t / nrm2(t);
  }
} /* namespace MBSimFlexibleBody */
