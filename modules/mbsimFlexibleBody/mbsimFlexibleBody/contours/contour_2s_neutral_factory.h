/*
 * contour_2s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#ifndef CONTOUR_2S_NEUTRAL_FACTORY_H_
#define CONTOUR_2S_NEUTRAL_FACTORY_H_

#include "mbsimFlexibleBody/contours/contour2s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

namespace MBSimFlexibleBody {
  
  class Contour2sNeutralFactory : public Contour2s {
    public:
      Contour2sNeutralFactory(const std::string &name) : Contour2s(name) { }
      ~Contour2sNeutralFactory() override = default;

      MBSim::ContourFrame* createContourFrame(const std::string &name="P") override;

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override { return etaNodes.size() and xiNodes.size() and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1] or zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]); }

      /* INHERITED INTERFACE OF CONTOUR */
      MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override{ return findContactPairingFlexible(type0, type1); }

  };

} /* namespace MBSimFlexibleBody */
#endif
