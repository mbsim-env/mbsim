/*
 * NeutralVelocity1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_velocity_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace MBSim;


namespace MBSimFlexibleBody {
//  class FlexibleBodyContinuum<double>;

  NeutralNurbsVelocity1s::NeutralNurbsVelocity1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsVelocity1s::~NeutralNurbsVelocity1s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsVelocity1s::update(ContourPointData &cp){
    Vec3 Tmpv = curve.pointAt(cp.getLagrangeParameterPosition()(0));
    cp.getFrameOfReference().setVelocity(Tmpv);
  }

  void NeutralNurbsVelocity1s::buildNodelist(){
    NodeFrame frame;
    for (int i = 0; i < nodes.size(); i++) {
      frame.setNodeNumber(nodes(i));
      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame, Frame::velocity);
      Nodelist.set(i, trans(frame.getVelocity()));
    }
//    cout << "neutralVelocity"<< Nodelist << endl << endl;
  }


} /* namespace MBSimFlexibleBody */
