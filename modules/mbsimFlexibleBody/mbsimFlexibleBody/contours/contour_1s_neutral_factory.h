/*
 * contour_1s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#ifndef CONTOUR_1S_NEUTRAL_FACTORY_H_
#define CONTOUR_1S_NEUTRAL_FACTORY_H_

#include "mbsimFlexibleBody/contours/contour1s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {
  
  class Contour1sNeutralFactory : public Contour1s {
    public:
      Contour1sNeutralFactory(const std::string &name) : Contour1s(name), uMin(0.), uMax(1.), degU(3), openStructure(false) { }

      MBSim::ContourFrame* createContourFrame(const std::string &name="P") override;

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override { return etaNodes.size() and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]); }

      MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override { return findContactPairingFlexible(type0, type1); }

      virtual void setOpenStructure(const bool & openStructure_) { openStructure = openStructure_; }

      virtual void setuMin(const double & uMin_) { uMin = uMin_; }

      virtual void setuMax(const double & uMax_) { uMax = uMax_; }

    protected:

      /*!
       * \brief starting parameter of the contour descriptions
       */
      double uMin;

      /*!
       * \brief ending parameter of the contour description
       */
      double uMax;

      /*!
       * \brief interpolation degree
       */
      int degU;

      /*!
       * \brief is the contour opened or closed?
       */
      bool openStructure;

  };

} /* namespace MBSimFlexibleBody */
#endif
