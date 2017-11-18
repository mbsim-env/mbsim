/*
 * neutral_nurbs_dotangle_1s.h
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */

#ifndef NEUTRAL_NURBS_DOTANGLE_1S_H_
#define NEUTRAL_NURBS_DOTANGLE_1S_H_

#include "neutral_nurbs_1s.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsDotangle1s : public MBSimFlexibleBody::NeutralNurbs1s {
    public:
      NeutralNurbsDotangle1s(MBSim::Element* parent_, const fmatvec::VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      ~NeutralNurbsDotangle1s() override;
      void update(MBSim::ContourFrame *frame) override;
      void computeCurve(bool update) override;
    protected:
      void buildNodelist() override;
  };

} /* namespace MBSimFlexibleBody */
#endif
