/*
 * contour_1s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#ifndef CONTOUR_1S_NEUTRAL_FACTORY_H_
#define CONTOUR_1S_NEUTRAL_FACTORY_H_

#include "mbsim/contours/contour1s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {
  
  class Contour1sNeutralFactory : public MBSim::Contour1s {
    public:
      Contour1sNeutralFactory(const std::string &name);

      virtual ~Contour1sNeutralFactory();

      virtual std::string getType() const {
        return "Contour1sNeutralFactory";
      }

      virtual MBSim::ContourFrame* createContourFrame(const std::string &name="P");

      virtual MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1) {
        return findContactPairingFlexible(type0.c_str(), type1.c_str());
      }

      virtual void setOpenStructure(const bool & openStructure_) {
        openStructure = openStructure_;
      }

      virtual void setuMin(const double & uMin_) {
        uMin = uMin_;
      }

      virtual void setuMax(const double & uMax_) {
        uMax = uMax_;
      }

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
