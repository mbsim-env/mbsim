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
#include <mbsimFlexibleBody/contours/contour_1s_neutral_factory.h>
#include "neutral_contour_components/neutral_nurbs_velocity_1s.h"
#include "neutral_contour_components/neutral_nurbs_position_1s.h"
#include "neutral_contour_components/neutral_nurbs_angle_1s.h"
#include "neutral_contour_components/neutral_nurbs_dotangle_1s.h"

namespace MBSimFlexibleBody {
  
  class Contour1sNeutralCosserat : public MBSimFlexibleBody::Contour1sNeutralFactory {
    public:
      Contour1sNeutralCosserat(const std::string &name_);
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

      void setTransNodes(const fmatvec::VecInt & transNodes_);
      void setRotNodes(const fmatvec::VecInt & rotNodes_);
      void setNodeOffest(const double nodeOffset_);

      double getuMax() const {
        return uMax;
      }
      
      double getuMin() const {
        return uMin;
      }
      
    protected:
      /*!
       * \brief index of the translational Nodes
       */
      fmatvec::VecInt transNodes;

      /*!
       * \brief index of the rotational Nodes
       */
      fmatvec::VecInt rotNodes;

      /*!
       * \brief offset between translationa and rotational nodes
       */
      double nodeOffset;

      CardanPtr ANGLE;
      NeutralNurbsPosition1s* NP;
      NeutralNurbsVelocity1s* NV;
      NeutralNurbsAngle1s* NA;
      NeutralNurbsDotangle1s* NDA;

  };

} /* namespace MBSimFlexibleBody */
#endif
