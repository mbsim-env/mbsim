/*
 * NeutralVelocity1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_velocity_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {
//  class FlexibleBodyContinuum<double>;

  NeutralNurbsVelocity1s::NeutralNurbsVelocity1s(Element* parent_, std::vector<ContourPointData>& ContourPoints_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, ContourPoints_, nodeOffset_, uMin_, uMax_, degU_, openStructure_){
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
    for (int i = 0; i < numOfNodes; i++) {
      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsForFrame(contourPoints.at(i), velocity);
      Nodelist.set(i, trans(contourPoints.at(i).getFrameOfReference().getVelocity()));
    }
//    cout << "neutralVelocity"<< Nodelist << endl << endl;
  }


} /* namespace MBSimFlexibleBody */
