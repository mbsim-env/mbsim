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
      Contour1sNeutralLinearExternalFFR(const std::string &name_) : Contour1sNeutralFactory(name_), transNodes(0), NP(NULL), NLP(NULL), NV(NULL), qSize(0) { }

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

      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta) { return getWs(t,zeta); }
      virtual fmatvec::Vec3 getWv(double t, const fmatvec::Vec2 &zeta) { return getWt(t,zeta); }

      void updatePositions(double t, MBSim::ContourFrame *frame);
      void updateVelocities(double t, MBSim::ContourFrame *frame);
      void updateJacobians(double t, MBSim::ContourFrame *frame, int j=0);

      /* GETTER / SETTER*/
      fmatvec::VecInt getTransNodes();

      void setFrameOfReference(MBSim::Frame *frame) { R = frame; }

      void resetUpToDate();

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
