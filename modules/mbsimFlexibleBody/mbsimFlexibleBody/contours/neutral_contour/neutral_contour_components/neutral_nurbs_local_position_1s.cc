/*
 * NeutralPosition1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_local_position_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsim/frames//contour_frame.h"
#include "mbsimFlexibleBody/frames/node_frame.h"

using namespace MBSim;


namespace MBSimFlexibleBody {

  NeutralNurbsLocalPosition1s::NeutralNurbsLocalPosition1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsLocalPosition1s::~NeutralNurbsLocalPosition1s() {
    // TODO Auto-generated destructor stub
  }

  Vec3 NeutralNurbsLocalPosition1s::getLocalPosition(double s){
//    if(updCurve) computeCurve(true);
    return curve.pointAt(s);
  }

  void NeutralNurbsLocalPosition1s::update(ContourFrame *frame) {
    throw;
//    if(updCurve) computeCurve(true);
//    frame->setLocalPosition(curve.pointAt(frame->getEta()));
  }

  void NeutralNurbsLocalPosition1s::buildNodelist(){
    Vec3 r;
    for (int i = 0; i < nodes.size(); i++) {
      r = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getLocalPosition(nodes(i));
      Nodelist.set(i, trans(r));
    }
    cout << "neutralLocalPosition"<< Nodelist << endl << endl;
  }
} /* namespace MBSimFlexibleBody */
