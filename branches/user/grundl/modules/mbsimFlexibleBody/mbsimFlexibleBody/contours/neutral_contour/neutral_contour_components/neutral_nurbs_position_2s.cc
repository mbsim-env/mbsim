/*
 * neutral_nurbs_position_2s.cc
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_position_2s.h"
#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {

  NeutralNurbsPosition2s::NeutralNurbsPosition2s(Element* parent_, const MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_):
    NeutralNurbs2s(parent_, nodes, nodeOffset_, degU_, degV_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsPosition2s::~NeutralNurbsPosition2s() {
    // TODO Auto-generated destructor stub
  }

  void NeutralNurbsPosition2s::update(ContourPointData &cp){
    Vec3 Tmpv = surface.pointAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1));
    cp.getFrameOfReference().setPosition(Tmpv);
  }

  // TODO: this Normal and secondTangent is only work for the neutral surface on the xy plane. Need to adapt to different situations.
  void NeutralNurbsPosition2s::updatePositionNormal(ContourPointData &cp){
    Vec3 Tmpv = surface.normal(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1));  // TODO: check whether this normal point outwards of the material. In the nurbs_disk_2s, the normal is been reversed manually.
    Tmpv = Tmpv / nrm2(Tmpv);  // normalize the normal vector
    cp.getFrameOfReference().getOrientation().set(0, Tmpv);
  }

  void NeutralNurbsPosition2s::updatePositionFirstTangent(ContourPointData &cp){
    GeneralMatrix<Vec3> skl;
    /************************************************************************************
     *       | s(u,v)   Sv |   | s(u,v)                                \partial{s(u,v)} / \partial{v} |
     * skl = |             | = |                                                                      |
     *       | Su       0  |   | \partial{s(u,v)} / \partial{u}        0                              |
     * 0 <= k + l <= d
     * Su is along the U direction. It is the first tangent
     * Sv is along the V direction. It is the second tangent
     * For more information, see NURBS book Page112: fig.3.26
     ************************************************************************************/
    surface.deriveAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1), 1, skl);
    Vec3 Tmpv = skl(1, 0);  // Su
    Tmpv = Tmpv / nrm2(Tmpv);  // normalize the vector
    cp.getFrameOfReference().getOrientation().set(1, Tmpv);

    // debug
//    double u = cp.getLagrangeParameterPosition()(0);
//    double v = cp.getLagrangeParameterPosition()(1);
//    cout << "u, v =" << u << ", "<< v << endl;
//    Vec3 TmpvT = surface.normal(u, v);  // TODO: check whether this normal point outwards of the material. In the nurbs_disk_2s, the normal is been reversed manually.
//    cout << "normal at position2s:\n" << TmpvT;
//    GeneralMatrix<Vec3> sklT;
//    surface.deriveAt(u, v, 1, sklT);
//    cout << "\n derive at position2s:\n" << sklT << endl << endl;
//
//    TmpvT = TmpvT / nrm2(TmpvT);  // normalize the normal vector
//    cout << "normal after normalization:\n" << TmpvT << endl;
//    TmpvT = sklT(1,0);
//    TmpvT = TmpvT / nrm2(TmpvT);  // normalize the vector
//    cout << "first Tangent after normalization:\n" << TmpvT<< endl;
//    TmpvT = sklT(0,1);
//    TmpvT = TmpvT / nrm2(TmpvT);  // normalize the vector
//    cout << "second Tangent after normalization:\n" << TmpvT << endl;
  }

  // TODO : this only works fore the xy plane surface.
  void NeutralNurbsPosition2s::updatePositionSecondTangent(ContourPointData &cp){
    GeneralMatrix<Vec3> skl;
    surface.deriveAt(cp.getLagrangeParameterPosition()(0), cp.getLagrangeParameterPosition()(1), 1, skl);
    Vec3 Tmpv = skl(0, 1);  // Sv
    Tmpv = Tmpv / nrm2(Tmpv);  // normalize the vector
    cp.getFrameOfReference().getOrientation().set(2, Tmpv);
  }




  void NeutralNurbsPosition2s::buildNodelist(){
    NodeFrame frame;
    for (int i = 0; i < numOfNodesU; i++) {
      for (int j = 0; j < numOfNodesV; j++) {
        frame.setNodeNumber(nodes(i,j));
        static_cast<FlexibleBodyContinuum<double>*>(parent)->updateKinematicsAtNode(&frame, position);
        Nodelist(i,j) = frame.getPosition();
//        cout << "contourPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl;
//        cout << "nP2(" << i <<","<< j<< ")" << trans(Nodelist(i,j)) << endl << endl;
      }
//    cout << "neutralPosition2s"<< Nodelist << endl << endl;
    }
  }

} /* namespace MBSimFlexibleBody */
