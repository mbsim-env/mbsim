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
      Contour1sNeutralLinearExternalFFR(const std::string &name_) : Contour1sNeutralFactory(name_), transNodes(0), NP(nullptr), NLP(nullptr), NV(nullptr), qSize(0) { }

      ~Contour1sNeutralLinearExternalFFR() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      virtual NeutralNurbsVelocity1s* createNeutralVelocity();
      virtual NeutralNurbsPosition1s* createNeutralPosition();
      virtual NeutralNurbsLocalPosition1s* createNeutralLocalPosition();
      virtual void createNeutralModeShape();

      /*!
       * \brief read the node numbers from a file
       */
      void readTransNodes(const std::string& file);

      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWu(const fmatvec::Vec2 &zeta) override { return evalWs(zeta); }
      fmatvec::Vec3 evalWv(const fmatvec::Vec2 &zeta) override { return evalWt(zeta); }

      void updatePositions(MBSim::ContourFrame *frame) override;
      void updateVelocities(MBSim::ContourFrame *frame) override;
      void updateJacobians(MBSim::ContourFrame *frame, int j=0) override;

      /* GETTER / SETTER*/
      fmatvec::VecInt getTransNodes();

      void setFrameOfReference(MBSim::Frame *frame) { R = frame; }

      void resetUpToDate() override;

    protected:

      /*!
       * \brief list of nodes to be interpolated
       *
       * REMARK: MBSim starts indexing with 0. If external programs (e.g. abaqus) start indexing with 1 the user has to substract one for each node-index
       */
      fmatvec::VecInt transNodes;

      NeutralNurbsPosition1s* NP;
      NeutralNurbsLocalPosition1s* NLP;
      NeutralNurbsVelocity1s* NV;

      int qSize;

      std::vector<MBSim::NurbsCurve> curveModeShape; // size = number of elastic coordinates

      MBSim::Frame *R;

  };

} /* namespace MBSimFlexibleBody */
#endif
