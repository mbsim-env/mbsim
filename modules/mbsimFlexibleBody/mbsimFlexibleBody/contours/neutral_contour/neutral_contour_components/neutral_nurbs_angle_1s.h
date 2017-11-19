/*
 * NeutralAngle1sNurbs.h
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */

#ifndef NEUTRAL_NURBS_ANGLE_1S_H_
#define NEUTRAL_NURBS_ANGLE_1S_H_

#include "neutral_nurbs_1s.h"
#include "mbsimFlexibleBody/pointer.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsAngle1s : public MBSimFlexibleBody::NeutralNurbs1s {
    public:
      NeutralNurbsAngle1s(MBSim::Element* parent_, const fmatvec::VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      ~NeutralNurbsAngle1s() override;
      Vec3 calculateStaggeredAngle(double oringnalPosition);
      void update(MBSim::ContourFrame *frame) override ;
      virtual void updateAngleNormal(MBSim::ContourFrame *frame) ;
      virtual void updateAngleFirstTangent(MBSim::ContourFrame *frame) ;
      virtual void updateAngleSecondTangent(MBSim::ContourFrame *frame) ;
      void computeCurve(bool update) override;

    protected:
      void buildNodelist() override;
      /**
      * \brief angle parametrisation
      */
      CardanPtr ANGLE;
  };

} /* namespace MBSimFlexibleBody */
#endif
