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
      ~NeutralNurbsPosition2s() override;
      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWn(const fmatvec::Vec2 &zeta);
      void update(MBSim::ContourFrame *frame) override;
      virtual void updatePositionNormal(MBSim::ContourFrame *frame);
      virtual void updatePositionFirstTangent(MBSim::ContourFrame *frame);
      virtual void updatePositionSecondTangent(MBSim::ContourFrame *frame);
    protected:
      void buildNodelist() override;
  };

} /* namespace MBSimFlexibleBody */
#endif
