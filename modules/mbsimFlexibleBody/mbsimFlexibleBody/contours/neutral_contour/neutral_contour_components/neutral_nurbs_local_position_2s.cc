/*
 * neutral_nurbs_local_position_2s.cc
 *
 *  Created on: 04.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_local_position_2s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"

using namespace MBSim;
using namespace fmatvec;

namespace MBSimFlexibleBody {

  NeutralNurbsLocalPosition2s::NeutralNurbsLocalPosition2s(Element* parent_, const MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_):
    NeutralNurbs2s(parent_, nodes, nodeOffset_, degU_, degV_, openStructure_){
    
  }
  
  NeutralNurbsLocalPosition2s::~NeutralNurbsLocalPosition2s() = default;

  Vec3 NeutralNurbsLocalPosition2s::evalLocalPosition(const Vec2 &zeta){
//    if(updSurface) computeCurve(true);
    return surface.pointAt(zeta(0),zeta(1));
  }

  void NeutralNurbsLocalPosition2s::update(ContourFrame *frame) {
    throw;
//    Vec3 Tmpv = surface.pointAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1));
//    cp.getFrameOfReference().setLocalPosition(Tmpv);
  }

  void NeutralNurbsLocalPosition2s::buildNodelist() {
    Vec3 r;
    for (int i = 0; i < numOfNodesU; i++) {
      for (int j = 0; j < numOfNodesV; j++) {
        r = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->evalLocalPosition(nodes(i,j));
        Nodelist(i,j) = r;
//        cout << "contourLocalPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl; // the index get here is one less than the index in Abaqus.
//        cout << "neutralLocalPosition2s i, j " << i << ", " << j << Nodelist(i,j) << endl << endl;
      }
//    cout << "neutralLocalPosition2s"<< Nodelist << endl << endl;
    }
  }

  void NeutralNurbsLocalPosition2s::surfMeshParamsClosedU(Vec& uk, Vec& vl) {
    buildNodelist();
    MBSim::surfMeshParamsClosedU(Nodelist, uk, vl, degU);
  }

  void NeutralNurbsLocalPosition2s::surfMeshParams(Vec& uk, Vec& vl) {
    buildNodelist();
    MBSim::surfMeshParams(Nodelist, uk, vl);
  }


  // TODO add a special computeCurve(bool update) here, as the LocalPosition is constant.


} /* namespace MBSimFlexibleBody */
