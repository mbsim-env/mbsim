/*
 * neutral_nurbs_velocity_2s.cc
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_velocity_2s.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/frames/contour_frame.h"

using namespace MBSim;
using namespace fmatvec;

namespace MBSimFlexibleBody {

  NeutralNurbsVelocity2s::NeutralNurbsVelocity2s(Element* parent_, const MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_) :
      NeutralNurbs2s(parent_, nodes, nodeOffset_, degU_, degV_, openStructure_) {
    
  }
  
  NeutralNurbsVelocity2s::~NeutralNurbsVelocity2s() = default;

  void NeutralNurbsVelocity2s::update(ContourFrame *frame) {
    if(updSurface) computeCurve(true);
    frame->setVelocity(surface.pointAt(frame->evalEta(),frame->evalXi()));
  }

  void NeutralNurbsVelocity2s::buildNodelist() {
    for (int i = 0; i < numOfNodesU; i++) {
      for (int j = 0; j < numOfNodesV; j++) {
        NodeFrame P("P",nodes(i,j));
        P.setParent(parent);
        Nodelist(i,j) = P.evalVelocity();
//        msg(Debug) << "contourPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl;
//        msg(Debug) << "neutralVelocity2s i, j " << i << ", " << j << Nodelist(i,j) << endl << endl;
      }
//    msg(Debug) << "neutralVelocity2s"<< Nodelist << endl << endl;
    }
  }

} /* namespace MBSimFlexibleBody */
