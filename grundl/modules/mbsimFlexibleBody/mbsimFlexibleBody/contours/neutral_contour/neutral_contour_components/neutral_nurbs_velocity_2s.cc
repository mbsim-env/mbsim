/*
 * neutral_nurbs_velocity_2s.cc
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_velocity_2s.h"
#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {

  NeutralNurbsVelocity2s::NeutralNurbsVelocity2s(Element* parent_, fmatvec::GeneralMatrix<ContourPointData>& ContourPoints_, double nodeOffset_, int degU_, int degV_, bool openStructure_):
    NeutralNurbs2s(parent_, ContourPoints_, nodeOffset_, degU_, degV_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsVelocity2s::~NeutralNurbsVelocity2s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsVelocity2s::update(ContourPointData &cp){
    Vec3 Tmpv = surface.pointAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1));
    cp.getFrameOfReference().setVelocity(Tmpv);
  }

  void NeutralNurbsVelocity2s::buildNodelist(){
    for (int i = 0; i < numOfNodesU; i++) {
      for (int j = 0; j < numOfNodesV; j++) {
        static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsForFrame(contourPoints(i, j), velocity);
        Nodelist(i,j) = contourPoints(i, j).getFrameOfReference().getVelocity();
        cout << "contourPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl;
        cout << "neutralVelocity2s i, j " << i << ", " << j << Nodelist(i,j) << endl << endl;
      }
//    cout << "neutralVelocity2s"<< Nodelist << endl << endl;
    }
  }

} /* namespace MBSimFlexibleBody */
