/*
 * neutral_nurbs_dotangle_1s.h
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */

#ifndef NEUTRAL_NURBS_DOTANGLE_1S_H_
#define NEUTRAL_NURBS_DOTANGLE_1S_H_

#include "neutral_nurbs_1s.h"

namespace MBSimFlexibleBody {
  
  class NeutralNurbsDotangle1s : public MBSimFlexibleBody::NeutralNurbs1s {
    public:
      NeutralNurbsDotangle1s(Element* parent_, std::vector<ContourPointData>& rotContourPoints_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      virtual ~NeutralNurbsDotangle1s();
      virtual void update(ContourPointData &cp) ;
      virtual void computeCurve(bool update);
    protected:
      virtual void buildNodelist();
  };

} /* namespace MBSimFlexibleBody */
#endif
