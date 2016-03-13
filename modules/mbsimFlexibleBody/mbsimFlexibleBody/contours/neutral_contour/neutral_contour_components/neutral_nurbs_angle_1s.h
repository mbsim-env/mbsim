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
      virtual ~NeutralNurbsAngle1s();
      Vec3 calculateStaggeredAngle(double oringnalPosition);
      virtual void update(double t, MBSim::ContourFrame *frame) ;
      virtual void updateAngleNormal(double t, MBSim::ContourFrame *frame) ;
      virtual void updateAngleFirstTangent(double t, MBSim::ContourFrame *frame) ;
      virtual void updateAngleSecondTangent(double t, MBSim::ContourFrame *frame) ;
      virtual void computeCurve(double t, bool update);

    protected:
      virtual void buildNodelist(double t);
      /**
      * \brief angle parametrisation
      */
      CardanPtr ANGLE;
  };

} /* namespace MBSimFlexibleBody */
#endif
