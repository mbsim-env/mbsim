/*
 * neutral_nurbs_2s.h
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */

#ifndef NEUTRAL_NURBS_2S_H_
#define NEUTRAL_NURBS_2S_H_

#include "mbsim/contour_pdata.h"
#include <mbsim/numerics/nurbs/nurbs_surface.h>

namespace MBSimFlexibleBody {
  
  class NeutralNurbs2s {
    public:
      NeutralNurbs2s(MBSim::Element* parent_,  const fmatvec::MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_);
      virtual ~NeutralNurbs2s();
      virtual void computeCurve(const Vec& uk, const Vec& vl, bool update = false);
      virtual void update(MBSim::ContourPointData &cp) = 0;
//      virtual const fmatvec::Vec getuVec() {
//        return surface.getuVec();
//      }
    protected:
      virtual void buildNodelist() = 0;
      MBSim::NurbsSurface surface;
      MBSim::Element *parent;
      fmatvec::MatVI nodes;
      double nodeOffset;
      int numOfNodesU;
      int numOfNodesV;
      GeneralMatrix<Vec3> Nodelist;
      int degU;
      int degV;
      bool openStructure;
  };

} /* namespace MBSimFlexibleBody */
#endif
