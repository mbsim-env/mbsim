/*
 * neutral_nurbs_local_position_2s.cc
 *
 *  Created on: 04.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_local_position_2s.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace MBSim;


namespace MBSimFlexibleBody {

  NeutralNurbsLocalPosition2s::NeutralNurbsLocalPosition2s(Element* parent_, const MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_):
    NeutralNurbs2s(parent_, nodes, nodeOffset_, degU_, degV_, openStructure_){
    
  }
  
  NeutralNurbsLocalPosition2s::~NeutralNurbsLocalPosition2s() {
  }

  void NeutralNurbsLocalPosition2s::update(ContourPointData &cp){
    throw;
//    Vec3 Tmpv = surface.pointAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1));
//    cp.getFrameOfReference().setLocalPosition(Tmpv);
  }

  void NeutralNurbsLocalPosition2s::buildNodelist(double t){
    throw;
//    NodeFrame frame;
//    for (int i = 0; i < numOfNodesU; i++) {
//      for (int j = 0; j < numOfNodesV; j++) {
//        frame.setNodeNumber(nodes(i, j));
//        static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame, Frame::localPosition);
//        Nodelist(i,j) = frame.getLocalPosition();
////        cout << "contourLocalPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl; // the index get here is one less than the index in Abaqus.
////        cout << "neutralLocalPosition2s i, j " << i << ", " << j << Nodelist(i,j) << endl << endl;
//      }
////    cout << "neutralLocalPosition2s"<< Nodelist << endl << endl;
//    }
  }

  void NeutralNurbsLocalPosition2s::surfMeshParamsClosedU(double t, Vec& uk, Vec& vl) {
    buildNodelist(t);
    MBSim::surfMeshParamsClosedU(Nodelist, uk, vl, degU);
  }

  void NeutralNurbsLocalPosition2s::surfMeshParams(double t, Vec& uk, Vec& vl) {
    buildNodelist(t);
    MBSim::surfMeshParams(Nodelist, uk, vl);
  }


  // TODO add a special computeCurve(bool update) here, as the LocalPosition is constant.


} /* namespace MBSimFlexibleBody */
