/*
 * neutral_nurbs_position_2s.cc
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_position_2s.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/frames/contour_frame.h"

using namespace MBSim;


namespace MBSimFlexibleBody {

  NeutralNurbsPosition2s::NeutralNurbsPosition2s(Element* parent_, const MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_):
    NeutralNurbs2s(parent_, nodes, nodeOffset_, degU_, degV_, openStructure_){
    // TODO Auto-generated constructor stub
    
  }
  
  NeutralNurbsPosition2s::~NeutralNurbsPosition2s() {
    // TODO Auto-generated destructor stub
  }

  Vec3 NeutralNurbsPosition2s::getPosition(const Vec2 &zeta) {
    if(updSurface) computeCurve(true);
    return surface.pointAt(zeta(0), zeta(1));
  }

  Vec3 NeutralNurbsPosition2s::getWs(const Vec2 &zeta) {
    if(updSurface) computeCurve(true);
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
    surface.deriveAt(zeta(0), zeta(1), 1, skl);
    Vec3 Tmpv = skl(1, 0);  // Su
    return Tmpv / nrm2(Tmpv);  // normalize the vector
  }

  Vec3 NeutralNurbsPosition2s::getWt(const Vec2 &zeta) {
    if(updSurface) computeCurve(true);
    GeneralMatrix<Vec3> skl;
    surface.deriveAt(zeta(0), zeta(1), 1, skl);
    Vec3 Tmpv = skl(0, 1);  // Sv
    return Tmpv / nrm2(Tmpv);  // normalize the vector
  }

  Vec3 NeutralNurbsPosition2s::getWn(const Vec2 &zeta) {
    if(updSurface) computeCurve(true);
    Vec3 Tmpv = surface.normal(zeta(0), zeta(1));  // TODO: check whether this normal point outwards of the material. In the nurbs_disk_2s, the normal is been reversed manually.
    return Tmpv / nrm2(Tmpv);  // normalize the normal vector
  }

  void NeutralNurbsPosition2s::update(ContourFrame *frame){
    if(updSurface) computeCurve(true);
    frame->setPosition(surface.pointAt(frame->getEta(), frame->getXi()));
  }

  void NeutralNurbsPosition2s::updatePositionNormal(ContourFrame *frame) {
    frame->getOrientation(false).set(0, getWn(frame->getZeta()));
  }

  void NeutralNurbsPosition2s::updatePositionFirstTangent(ContourFrame *frame) {
    frame->getOrientation(false).set(1, getWs(frame->getZeta()));
  }

  void NeutralNurbsPosition2s::updatePositionSecondTangent(ContourFrame *frame) {
    frame->getOrientation(false).set(2, getWt(frame->getZeta()));
  }

  void NeutralNurbsPosition2s::buildNodelist(){
    for (int i = 0; i < numOfNodesU; i++) {
      for (int j = 0; j < numOfNodesV; j++) {
        NodeFrame P("P",nodes(i,j));
        P.setParent(parent);
        Nodelist(i,j) = P.evalPosition();
//        cout << "contourPoints(i,j):"  << contourPoints(i,j).getNodeNumber() << endl;
//        cout << "nP2(" << i <<","<< j<< ")" << trans(Nodelist(i,j)) << endl << endl;
      }
//    cout << "neutralPosition2s"<< Nodelist << endl << endl;
    }
  }

} /* namespace MBSimFlexibleBody */
