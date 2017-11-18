/*
 * neutral_nurbs_local_position_2s.h
 *
 *  Created on: 04.12.2013
 *      Author: zwang
 */

#ifndef _NEUTRAL_NURBS_LOCAL_POSITION_2S_H_
#define _NEUTRAL_NURBS_LOCAL_POSITION_2S_H_

#include "neutral_nurbs_2s.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsLocalPosition2s : public MBSimFlexibleBody::NeutralNurbs2s {
    public:
      NeutralNurbsLocalPosition2s(MBSim::Element* parent_, const fmatvec::MatVI & nodes, double nodeOffset, int degU_, int degV_, bool openStructure_);
      ~NeutralNurbsLocalPosition2s() override;
      fmatvec::Vec3 evalLocalPosition(const fmatvec::Vec2 &zeta);
      void update(MBSim::ContourFrame *frame) override;
      void surfMeshParamsClosedU(Vec& uk, Vec& vl);
      void surfMeshParams(Vec& uk, Vec& vl);
    protected:
      void buildNodelist() override;  // make this method public for calculating the lagrange parameters for nurbs interpolation
  };

} /* namespace MBSimFlexibleBody */
#endif
