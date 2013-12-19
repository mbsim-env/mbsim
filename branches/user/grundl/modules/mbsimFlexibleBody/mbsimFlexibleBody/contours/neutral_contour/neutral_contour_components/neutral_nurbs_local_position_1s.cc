/*
 * NeutralPosition1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_local_position_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {

  NeutralNurbsLocalPosition1s::NeutralNurbsLocalPosition1s(Element* parent_, std::vector<ContourPointData>& ContourPoints_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, ContourPoints_, nodeOffset_, uMin_, uMax_, degU_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsLocalPosition1s::~NeutralNurbsLocalPosition1s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsLocalPosition1s::update(ContourPointData &cp){
    Vec3 Tmpv = curve.pointAt(cp.getLagrangeParameterPosition()(0));
    cp.getFrameOfReference().setLocalPosition(Tmpv);
  }

  void NeutralNurbsLocalPosition1s::buildNodelist(){
    for (int i = 0; i < numOfNodes; i++) {
      static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsForFrame(contourPoints.at(i), localPosition);
      Nodelist.set(i, trans(contourPoints.at(i).getFrameOfReference().getLocalPosition()));
    }
//    cout << "neutralLocalPosition"<< Nodelist << endl << endl;
  }
} /* namespace MBSimFlexibleBody */
