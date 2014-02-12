/*
 * neutral_nurbs_2s.cc
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */

#include <config.h>

#include "neutral_nurbs_2s.h"

namespace MBSimFlexibleBody {


  NeutralNurbs2s::NeutralNurbs2s(Element* parent_,  const MatVVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_):
      surface(), parent(parent_), nodes(nodes), nodeOffset(nodeOffset_), numOfNodesU(nodes.rows()), numOfNodesV(nodes.cols()), Nodelist(nodes.rows(), nodes.cols()), degU(degU_), degV(degV_), openStructure(openStructure_){

  }

  NeutralNurbs2s:: ~NeutralNurbs2s(){

  }

  void NeutralNurbs2s::computeCurve(const Vec& uk, const Vec& vl, bool update){
    buildNodelist();

//    if (update)
//      surface.update(Nodelist);
//    else {
//      if (openStructure) {
//        surface.globalInterp(Nodelist, degV, degU, true);
//      }
//      else {
//        surface.globalInterpClosedU(Nodelist, degV, degU, true);
//      }
//    }
    if (openStructure) {
      surface.globalInterp(Nodelist, uk, vl, degU, degV);
    }else{
      surface.globalInterpClosedU(Nodelist, uk, vl, degU, degV);
    }
  }
}
