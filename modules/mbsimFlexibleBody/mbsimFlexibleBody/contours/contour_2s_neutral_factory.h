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
      virtual ~Contour2sNeutralFactory() { }
      virtual std::string getType() const { return "Contour2sNeutralFactory"; }

      virtual MBSim::ContourFrame* createContourFrame(const std::string &name="P");

      virtual bool isZetaOutside(const fmatvec::Vec2 &zeta) { return etaNodes.size() and xiNodes.size() and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1] or zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]); }

      /* INHERITED INTERFACE OF CONTOUR */
      virtual MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1){ return findContactPairingFlexible(type0.c_str(), type1.c_str()); }

  };

} /* namespace MBSimFlexibleBody */
#endif
