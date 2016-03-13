/*
 * NeutralPosition1sNurbs.h
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */

#ifndef _NEUTRAL_NURBS_POSITION_1S_H_
#define _NEUTRAL_NURBS_POSITION_1S_H_

#include "neutral_nurbs_1s.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsPosition1s : public MBSimFlexibleBody::NeutralNurbs1s {
    public:
      NeutralNurbsPosition1s(MBSim::Element* parent_, const fmatvec::VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      virtual ~NeutralNurbsPosition1s();
      fmatvec::Vec3 getPosition(double t, double s);
      virtual void update(double t, MBSim::ContourFrame *frame);
      virtual void updatePositionNormal(double t, MBSim::ContourFrame *frame);
      virtual void updatePositionFirstTangent(double t, MBSim::ContourFrame *frame);
      virtual void updatePositionSecondTangent(double t, MBSim::ContourFrame *frame);
      virtual void setBinormalDir(const fmatvec::Vec3 & b) {
        binormalDir = b / fmatvec::nrm2(b);
      }
    protected:
      /*!
       * \brief vector to compute the normal to
       */
      fmatvec::Vec3 binormalDir;

      virtual void buildNodelist(double t);

      /*!
       * \brief compute the tangent
       */
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta);
  };

} /* namespace MBSimFlexibleBody */
#endif
