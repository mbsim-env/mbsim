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
      ~NeutralNurbsPosition1s() override;
      fmatvec::Vec3 evalPosition(double s);
      fmatvec::Vec3 evalWs(double s);
      fmatvec::Vec3 evalWt(double s);
      void update(MBSim::ContourFrame *frame) override;
      virtual void updatePositionNormal(MBSim::ContourFrame *frame);
      virtual void updatePositionFirstTangent(MBSim::ContourFrame *frame);
      virtual void updatePositionSecondTangent(MBSim::ContourFrame *frame);
      virtual void setBinormalDir(const fmatvec::Vec3 & b) { binormalDir = b / fmatvec::nrm2(b); }
    protected:
      /*!
       * \brief vector to compute the normal to
       */
      fmatvec::Vec3 binormalDir;

      void buildNodelist() override;
  };

} /* namespace MBSimFlexibleBody */
#endif
