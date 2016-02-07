/*
 * neutral_nurbs_position_2s.h
 *
 *  Created on: 04.12.2013
 *      Author: zwang
 */

#ifndef _NEUTRAL_NURBS_POSITION_2S_H_
#define _NEUTRAL_NURBS_POSITION_2S_H_

#include "neutral_nurbs_2s.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsPosition2s : public MBSimFlexibleBody::NeutralNurbs2s {
    public:
      NeutralNurbsPosition2s(MBSim::Element* parent_, const fmatvec::MatVI & nodes, double nodeOffset, int degU_, int degV_, bool openStructure_);
      virtual ~NeutralNurbsPosition2s();
      virtual void update(MBSim::ContourPointData &cp);
      virtual void updatePositionNormal(MBSim::ContourPointData &cp);
      virtual void updatePositionFirstTangent(MBSim::ContourPointData &cp);
      virtual void updatePositionSecondTangent(MBSim::ContourPointData &cp);
    protected:
      virtual void buildNodelist(double t);
  };

} /* namespace MBSimFlexibleBody */
#endif
