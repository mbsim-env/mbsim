/*
 * contour_1s_neutral_linear_external_FFR.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#ifndef CONTOUR_1S_NEUTRAL_LINEAR_EXTERNAL_FFR_H_
#define CONTOUR_1S_NEUTRAL_LINEAR_EXTERNAL_FFR_H_

#include <mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h>
#include "mbsimFlexibleBody/pointer.h"
#include <mbsimFlexibleBody/contours/contour_1s_neutral_factory.h>
#include "neutral_contour_components/neutral_nurbs_velocity_1s.h"
#include "neutral_contour_components/neutral_nurbs_position_1s.h"
#include "neutral_contour_components/neutral_nurbs_local_position_1s.h"

namespace MBSimFlexibleBody {
  
  class Contour1sNeutralLinearExternalFFR : public Contour1sNeutralFactory {
    public:
      Contour1sNeutralLinearExternalFFR(const std::string &name_);

//      Contour1sNeutralLinearExternalFFR(const std::string &name_, FlexibleBodyLinearExternalFFR* parent_, std::vector<int> transNodes_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_);
      virtual ~Contour1sNeutralLinearExternalFFR();
      virtual void init(InitStage stage);
      virtual NeutralNurbsVelocity1s* createNeutralVelocity();
      virtual NeutralNurbsPosition1s* createNeutralPosition();
      virtual NeutralNurbsLocalPosition1s* createNeutralLocalPosition();
      virtual void createNeutralModeShape();

      /*!
       * \brief read the node numbers from a file
       */
      void readTransNodes(std::string file);


      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::Frame::Feature ff);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual void updateStateDependentVariables(double t);

      /* GETTER / SETTER*/
      fmatvec::VecInt getTransNodes();

    protected:

      /*!
       * \brief list of nodes to be interpolated
       *
       * REMARK: MBSim starts indexing with 0. If external programs (e.g. abaqus) start indexing with 1 the user has to substract one for each node-index
       */
      fmatvec::VecInt transNodes;
//      double nodeOffset;
//      std::vector<ContourPointData> transContourPoints;
//      int degU;

      NeutralNurbsPosition1s* NP;
      NeutralNurbsLocalPosition1s* NLP;
      NeutralNurbsVelocity1s* NV;

      int qSize;

      std::vector<MBSim::NurbsCurve> curveModeShape; // size = number of elastic coordinates

  };

} /* namespace MBSimFlexibleBody */
#endif
