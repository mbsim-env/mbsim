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
      fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getWn(double t, const fmatvec::Vec2 &zeta);
      virtual void update(double t, MBSim::ContourFrame *frame);
      virtual void updatePositionNormal(double t, MBSim::ContourFrame *frame);
      virtual void updatePositionFirstTangent(double t, MBSim::ContourFrame *frame);
      virtual void updatePositionSecondTangent(double t, MBSim::ContourFrame *frame);
    protected:
      virtual void buildNodelist(double t);
  };

} /* namespace MBSimFlexibleBody */
#endif
