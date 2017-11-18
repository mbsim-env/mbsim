/*
 * NeutralVelocity1sNurbs.h
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */

#ifndef _NEUTRAL_NURBS_VELOCITY_1S_H_
#define _NEUTRAL_NURBS_VELOCITY_1S_H_

#include "neutral_nurbs_1s.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsVelocity1s : public MBSimFlexibleBody::NeutralNurbs1s {
    public:
      NeutralNurbsVelocity1s(MBSim::Element* parent_, const fmatvec::VecInt & nodes, double nodeOffset, double uMin_, double uMax_, int degU_, bool openStructure_);
      ~NeutralNurbsVelocity1s() override;
      void update(MBSim::ContourFrame *frame) override;
    protected:
      void buildNodelist() override;
  };

} /* namespace MBSimFlexibleBody */
#endif
