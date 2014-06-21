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

//#include "mbsimFlexibleBody/utils/contact_utils.h"

namespace MBSimFlexibleBody {
  
  class Contour1sNeutralFactory : public MBSim::Contour1s {
    public:
//      NeutralContourFactory(const std::string &name):MBSim::ContourContinuum<double>(name){};
      Contour1sNeutralFactory(const std::string &name);

      virtual ~Contour1sNeutralFactory();

      virtual std::string getType() const {
        return "Contour1sNeutralFactory";
      }
      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(MBSim::ContourPointData &cp) {
        updateKinematicsForFrame(cp, MBSim::Frame::position);
      }
      virtual void computeRootFunctionFirstTangent(MBSim::ContourPointData &cp) {
        updateKinematicsForFrame(cp, MBSim::Frame::firstTangent);
      }
      virtual void computeRootFunctionNormal(MBSim::ContourPointData &cp) {
        updateKinematicsForFrame(cp, MBSim::Frame::normal);
      }
      virtual void computeRootFunctionSecondTangent(MBSim::ContourPointData &cp) {
        updateKinematicsForFrame(cp, MBSim::Frame::secondTangent);
      }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::Frame::Feature ff) = 0;
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0) = 0;
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
