/*
 * NeutralPosition1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_position_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {
//  class FlexibleBodyContinuum<double>;

  NeutralNurbsPosition1s::NeutralNurbsPosition1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsPosition1s::~NeutralNurbsPosition1s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsPosition1s::update(ContourPointData &cp){
    Vec3 Tmpv = curve.pointAt(cp.getLagrangeParameterPosition()(0));
    cp.getFrameOfReference().setPosition(Tmpv);
  }

  // TODO: this Normal and secondTangent is only work for the neutral curve on the xy plane. Need to adapt to different situations.
  void NeutralNurbsPosition1s::updatePositionNormal(ContourPointData &cp){
    Vec3 b("[0; 0; -1]"); // binormal point to negative z direction
    Vec3 Tmpv = curve.normal(cp.getLagrangeParameterPosition()(0), b);
    Tmpv = Tmpv / nrm2(Tmpv);  // normalize the normal vector
    cp.getFrameOfReference().getOrientation().set(0, Tmpv);
  }

  void NeutralNurbsPosition1s::updatePositionFirstTangent(ContourPointData &cp){
    Vec3 Tmpv = curve.firstDn(cp.getLagrangeParameterPosition()(0));
    Tmpv = Tmpv / nrm2(Tmpv);  // normalize the vector
    cp.getFrameOfReference().getOrientation().set(1, Tmpv);
  }

  // TODO : this only works fore the xy plane curve.
  void NeutralNurbsPosition1s::updatePositionSecondTangent(ContourPointData &cp){
    Vec3 Tmpv("[0; 0; -1]");
    cp.getFrameOfReference().getOrientation().set(2, Tmpv);
  }




  void NeutralNurbsPosition1s::buildNodelist(){
    NodeFrame frame;
    for (int i = 0; i < nodes.size(); i++) {
      frame.setNodeNumber(nodes(i));
      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame,  position);
      Nodelist.set(i, trans(frame.getPosition()));
    }
//    cout << "neutralPosition"<< Nodelist << endl << endl;
  }
} /* namespace MBSimFlexibleBody */
