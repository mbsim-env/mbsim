/*
 * neutral_nurbs_1s.h
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */

#ifndef NEUTRAL_NURBS_1S_H_
#define NEUTRAL_NURBS_1S_H_

#include "mbsim/contour_pdata.h"
#include <mbsim/numerics/nurbs/nurbs_curve.h>

namespace MBSimFlexibleBody {
  
  class NeutralNurbs1s {
    public:
      NeutralNurbs1s(MBSim::Element* parent_, const fmatvec::VecInt &nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      virtual ~NeutralNurbs1s();
      virtual void computeCurve(bool update = false);
      virtual void update(MBSim::ContourPointData &cp) = 0;
      virtual const fmatvec::Vec getuVec() {
        return curve.getuVec();
      }
    protected:
      virtual void buildNodelist() = 0;
      MBSim::NurbsCurve curve;
      MBSim::Element *parent;
      fmatvec::VecInt nodes;

      /*!
       * \brief offset between "leading" nodeset and this nodeset (e.g. used in cosserat between translational (= leading) nodeset and rotational nodeset)
       */
      double nodeOffset;
      fmatvec::MatVx3 Nodelist;
      double uMin;
      double uMax;
      int degU;
      bool openStructure;
  };

} /* namespace MBSimFlexibleBody */
#endif
