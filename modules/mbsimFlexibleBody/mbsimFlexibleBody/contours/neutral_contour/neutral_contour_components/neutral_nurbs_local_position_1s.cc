/*
 * NeutralPosition1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_local_position_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/node_frame.h"

using namespace MBSim;


namespace MBSimFlexibleBody {

  NeutralNurbsLocalPosition1s::NeutralNurbsLocalPosition1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsLocalPosition1s::~NeutralNurbsLocalPosition1s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsLocalPosition1s::update(ContourPointData &cp){
//    Vec3 Tmpv = curve.pointAt(cp.getLagrangeParameterPosition()(0));
//    cp.getFrameOfReference().setLocalPosition(Tmpv);
    throw;
  }

  void NeutralNurbsLocalPosition1s::buildNodelist(double t){
    throw;
//    NodeFrame frame;
//    for (int i = 0; i < nodes.size(); i++) {
//      frame.setNodeNumber(nodes(i));
//      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame,  Frame::localPosition);
//      Nodelist.set(i, trans(frame.getLocalPosition()));
//    }
//    cout << "neutralLocalPosition"<< Nodelist << endl << endl;
  }
} /* namespace MBSimFlexibleBody */
