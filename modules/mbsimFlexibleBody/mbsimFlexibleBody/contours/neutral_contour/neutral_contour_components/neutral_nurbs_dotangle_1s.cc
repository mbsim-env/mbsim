/*
 * neutral_nurbs_dotangle_1s.cc
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_dotangle_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/node_frame.h"

using namespace MBSim;

namespace MBSimFlexibleBody {
//  class FlexibleBodyContinuum<double>;

  NeutralNurbsDotangle1s::NeutralNurbsDotangle1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_) :
      NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_) {
    // TODO Auto-generated constructor stub

  }
  
  NeutralNurbsDotangle1s::~NeutralNurbsDotangle1s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsDotangle1s::update(ContourPointData &cp) {
    throw;
//    double uStaggered;
//    double oringnalPosition = cp.getLagrangeParameterPosition()(0);
//
//    if (oringnalPosition < nodeOffset)  // on the first half transElement
//      uStaggered = uMax + oringnalPosition - nodeOffset;
//    else
//      uStaggered = oringnalPosition - nodeOffset;
//
//    Vec3 Tmpv = curve.pointAt(uStaggered);
//    cp.getFrameOfReference().setDotAnglesOfOrientation(Tmpv);
  }

  void NeutralNurbsDotangle1s::buildNodelist(double t) {
//    NodeFrame frame;
//    for (int i = 0; i < nodes.size(); i++) {
//      frame.setNodeNumber(nodes(i));
//      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame, Frame::dotAngle);
//      Nodelist.set(i, trans(frame.getDotAnglesOfOrientation()));
//    }
////    cout << "neutralDotAngle" << Nodelist << endl << endl;
  }

  void NeutralNurbsDotangle1s::computeCurve(double t, bool update) {
    buildNodelist(t);

    if (update)
      curve.update(Nodelist);
    else {
      curve.globalInterp(Nodelist, uMin, uMax, degU, true);
    }
//    cout << "Neutral nurbs dotAngle 1s curve: " << endl;
////    stringstream x;
////    x << "xdA = [";
//    stringstream z;
//    z << "zdA = [";
//    for (int i = 0; i < 100; i++) {
//      double u = (uMax - uMin) / 100 * i;
////      x << curve.pointAt(u)(0) << ", ";
//      z << curve.pointAt(u)(2) << ", ";
//    }
////    x << "];" << endl;
//    z << "];" << endl;
//    cout << z.str();

  }

} /* namespace MBSimFlexibleBody */
