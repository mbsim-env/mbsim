/*
 * neutral_nurbs_velocity_2s.cc
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_velocity_2s.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace MBSim;

namespace MBSimFlexibleBody {

  NeutralNurbsVelocity2s::NeutralNurbsVelocity2s(Element* parent_, const MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_) :
      NeutralNurbs2s(parent_, nodes, nodeOffset_, degU_, degV_, openStructure_) {
    
  }
  
  NeutralNurbsVelocity2s::~NeutralNurbsVelocity2s() {
  }

  void NeutralNurbsVelocity2s::update(ContourPointData &cp) {
    Vec3 Tmpv = surface.pointAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1));
    cp.getFrameOfReference().setVelocity(Tmpv);
  }

  void NeutralNurbsVelocity2s::buildNodelist() {
    NodeFrame frame;
    for (int i = 0; i < numOfNodesU; i++) {
      for (int j = 0; j < numOfNodesV; j++) {
        frame.setNodeNumber(nodes(i, j));
        static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame, velocity);
        Nodelist(i, j) = frame.getVelocity();
//        cout << "contourPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl;
//        cout << "neutralVelocity2s i, j " << i << ", " << j << Nodelist(i,j) << endl << endl;
      }
//    cout << "neutralVelocity2s"<< Nodelist << endl << endl;
    }
  }

} /* namespace MBSimFlexibleBody */
