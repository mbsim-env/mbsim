/*
 * neutral_contour_1s_cosserat.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#ifndef CONTOUR_1S_NEUTRAL_COSSERAT_H_
#define CONTOUR_1S_NEUTRAL_COSSERAT_H_

#include <mbsimFlexibleBody/flexible_body/flexible_body_1s_cosserat.h>
#include "mbsimFlexibleBody/pointer.h"
#include "contour_1s_neutral_factory.h"
#include "neutral_contour_components/neutral_nurbs_velocity_1s.h"
#include "neutral_contour_components/neutral_nurbs_position_1s.h"
#include "neutral_contour_components/neutral_nurbs_angle_1s.h"
#include "neutral_contour_components/neutral_nurbs_dotangle_1s.h"

namespace MBSimFlexibleBody {
  
  class Contour1sNeutralCosserat : public MBSimFlexibleBody::Contour1sNeutralFactory {
    public:
      Contour1sNeutralCosserat(const std::string &name_, FlexibleBody1sCosserat* parent_, std::vector<int> transNodes_, std::vector<int> rotNodes_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      virtual ~Contour1sNeutralCosserat();
//      virtual std::string getType() const {
//        return "Contour1sNeutralCosserat";
//      }
      virtual void init(MBSim::InitStage stage);
      virtual NeutralNurbsPosition1s* createNeutralPosition();
      virtual NeutralNurbsVelocity1s* createNeutralVelocity();
      virtual NeutralNurbsAngle1s* createNeutralAngle();
      virtual NeutralNurbsDotangle1s* createNeutralDotangle();
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1);
      virtual void updateStateDependentVariables(double t);

      double getuMax() const {
        return uMax;
      }
      
      double getuMin() const {
        return uMin;
      }
      
    protected:
      std::vector<int> transNodes;  // TODO: can this be type of reference, No! as a contour, it should contains the nodes.
      std::vector<int> rotNodes;
      double nodeOffset;
      std::vector<ContourPointData> transContourPoints;
      std::vector<ContourPointData> rotContourPoints;
      int numOfTransNodes;
      int numOfRotNodes;
      double uMin;
      double uMax;
      int degU;
      bool openStructure;

      CardanPtr ANGLE;
      NeutralNurbsPosition1s* NP;
      NeutralNurbsVelocity1s* NV;
      NeutralNurbsAngle1s* NA;
      NeutralNurbsDotangle1s* NDA;

  };

} /* namespace MBSimFlexibleBody */
#endif
