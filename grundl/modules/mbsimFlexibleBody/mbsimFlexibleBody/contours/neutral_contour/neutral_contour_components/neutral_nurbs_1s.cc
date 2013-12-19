/*
 * neutral_nurbs_1s.cc
 *
 *  Created on: 07.11.2013
 *      Author: zwang
 */

#include <config.h>

#include "neutral_nurbs_1s.h"

namespace MBSimFlexibleBody {


  NeutralNurbs1s::NeutralNurbs1s(Element* parent_, std::vector<ContourPointData>& ContourPoints_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
      curve(), parent(parent_), contourPoints(ContourPoints_), nodeOffset(nodeOffset_), numOfNodes(ContourPoints_.size()), Nodelist(ContourPoints_.size(), NONINIT), uMin(uMin_), uMax(uMax_), degU(degU_), openStructure(openStructure_){

  }

  NeutralNurbs1s:: ~NeutralNurbs1s(){

  }

  void NeutralNurbs1s::computeCurve(bool update){
    buildNodelist();

    if (update)
      curve.update(Nodelist);
    else {
      if (openStructure) {
        curve.globalInterp(Nodelist, uMin, uMax, degU, true);
      }
      else {
        curve.globalInterpClosed(Nodelist, uMin, uMax, degU, true);
      }
    }

    cout << "Neutral nurbs ?? 1s curve: " << endl;
    stringstream x;
    x << "xpl = [";
    stringstream y;
    y << "ypl = [";
    for (int i = 0; i < 100; i++) {
      double u = (uMax-uMin)/100 * i;
      x << curve.pointAt(u)(0) << ", ";
      y << curve.pointAt(u)(1) << ", ";
    }
    x << "];" << endl;
    y << "];" << endl;
    cout << x.str() << y.str();
  }

}
