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
      NeutralNurbsPosition2s(Element* parent_, const fmatvec::MatVI & nodes, double nodeOffset, int degU_, int degV_, bool openStructure_);
      virtual ~NeutralNurbsPosition2s();
      virtual void update(ContourPointData &cp);
      virtual void updatePositionNormal(ContourPointData &cp);
      virtual void updatePositionFirstTangent(ContourPointData &cp);
      virtual void updatePositionSecondTangent(ContourPointData &cp);
    protected:
      virtual void buildNodelist();
  };

} /* namespace MBSimFlexibleBody */
#endif
