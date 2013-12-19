/*
 * contour_2s_neutral_linear_external_FFR.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#ifndef CONTOUR_2S_NEUTRAL_LINEAR_EXTERNAL_FFR_H_
#define CONTOUR_2S_NEUTRAL_LINEAR_EXTERNAL_FFR_H_

#include <mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h>
#include "mbsimFlexibleBody/pointer.h"
#include "contour_2s_neutral_factory.h"
#include "neutral_contour_components/neutral_nurbs_velocity_2s.h"
#include "neutral_contour_components/neutral_nurbs_position_2s.h"
#include "neutral_contour_components/neutral_nurbs_local_position_2s.h"

namespace MBSimFlexibleBody {
  
  class Contour2sNeutralLinearExternalFFR : public MBSimFlexibleBody::Contour2sNeutralFactory {
    public:
      Contour2sNeutralLinearExternalFFR(const std::string &name_, FlexibleBodyLinearExternalFFR* parent_, Mat transNodes_, double nodeOffset_, int degU_, int degV_, bool openStructure_);
      virtual ~Contour2sNeutralLinearExternalFFR();
      virtual void init(MBSim::InitStage stage);
      virtual NeutralNurbsVelocity2s* createNeutralVelocity();
      virtual NeutralNurbsPosition2s* createNeutralPosition();
      virtual NeutralNurbsLocalPosition2s* createNeutralLocalPosition();
      virtual void createNeutralModeShape();
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual void updateStateDependentVariables(double t);
      protected:
      Mat transNodes;  // TODO: can this be type of reference, No! as a contour, it should contains the nodes.
      double nodeOffset;
      fmatvec::GeneralMatrix<ContourPointData> transContourPoints;
      int numOfTransNodesU;
      int numOfTransNodesV;
      Vec uk;
      Vec vl;
      int degU;
      int degV;
      bool openStructure;

      NeutralNurbsPosition2s* NP;
      NeutralNurbsLocalPosition2s* NLP;
      NeutralNurbsVelocity2s* NV;

      int qSize;

      std::vector<MBSim::NurbsSurface> surfaceModeShape; // size = number of elastic coordinates

  };

} /* namespace MBSimFlexibleBody */
#endif
