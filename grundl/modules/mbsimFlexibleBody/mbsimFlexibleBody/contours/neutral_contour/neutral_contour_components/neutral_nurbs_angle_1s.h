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
      NeutralNurbsAngle1s(Element* parent_, std::vector<ContourPointData>& ContourPoints_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      virtual ~NeutralNurbsAngle1s();
      Vec3 calculateStaggeredAngle(double oringnalPosition);
      virtual void update(ContourPointData &cp) ;
      virtual void updateAngleNormal(ContourPointData &cp) ;
      virtual void updateAngleFirstTangent(ContourPointData &cp) ;
      virtual void updateAngleSecondTangent(ContourPointData &cp) ;
      virtual void computeCurve(bool update);

    protected:
      virtual void buildNodelist();
      /**
      * \brief angle parametrisation
      */
      CardanPtr ANGLE;
  };

} /* namespace MBSimFlexibleBody */
#endif
