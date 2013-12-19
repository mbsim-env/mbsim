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
namespace MBSimFlexibleBody {
  
//  class NeutralContourFactory: public MBSim::ContourContinuum<double> {
  class Contour1sNeutralFactory: public MBSim::Contour1s {
    public:
//      NeutralContourFactory(const std::string &name):MBSim::ContourContinuum<double>(name){};
      Contour1sNeutralFactory(const std::string &name):MBSim::Contour1s(name){};
      virtual ~Contour1sNeutralFactory(){};
      virtual std::string getType() const {
        return "Contour1sNeutralFactory";
      }
      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(MBSim::ContourPointData &cp) { updateKinematicsForFrame(cp, MBSim::position); }
      virtual void computeRootFunctionFirstTangent(MBSim::ContourPointData &cp) { updateKinematicsForFrame(cp, MBSim::firstTangent); }
      virtual void computeRootFunctionNormal(MBSim::ContourPointData &cp) { updateKinematicsForFrame(cp, MBSim::normal); }
      virtual void computeRootFunctionSecondTangent(MBSim::ContourPointData &cp) { updateKinematicsForFrame(cp, MBSim::secondTangent); }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void init(MBSim::InitStage stage) = 0;
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff) = 0;
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0) = 0;
      virtual MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1){
        return findContactPairingFlexible(type0.c_str(), type1.c_str());
      }

  };

} /* namespace MBSimFlexibleBody */
#endif
