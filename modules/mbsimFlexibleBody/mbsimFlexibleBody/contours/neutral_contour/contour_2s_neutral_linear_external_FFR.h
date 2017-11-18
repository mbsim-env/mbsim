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
#include <mbsimFlexibleBody/contours/contour_2s_neutral_factory.h>
#include "neutral_contour_components/neutral_nurbs_velocity_2s.h"
#include "neutral_contour_components/neutral_nurbs_position_2s.h"
#include "neutral_contour_components/neutral_nurbs_local_position_2s.h"

namespace MBSimFlexibleBody {
  
  class Contour2sNeutralLinearExternalFFR : public MBSimFlexibleBody::Contour2sNeutralFactory {
    public:
      /*!
       * \brief standard constructor
       */
      Contour2sNeutralLinearExternalFFR(const std::string &name_) : Contour2sNeutralFactory(name_), transNodes(), nodeOffset(0.), degU(3), degV(3), openStructure(false), NP(nullptr), NLP(nullptr), NV(nullptr), qSize(0) { }

      /*!
       * \brief destructor
       */
      ~Contour2sNeutralLinearExternalFFR() override;

      /*INHERITED INTERFACE */
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      /* GETTER / SETTER*/
      int getNumberOfTransNodesU();
      int getNumberOfTransNodesV();

      /*!
       * \brief function to set the indices of the nodes for the interpolation
       * The function expects index for the first node the index 1 for the second node the index 2 etc. --> Thus index 0 is not possible here.
       *
       */
      void setTransNodes(const fmatvec::MatVI & transNodes);

      /*!
       * \brief read data from a file that consists of sorted lists for nodes that should be interpolated
       *        Every line in the has to be a row of nodes in U-direction (the first direction of the interpolation)

       * The node numbering starts with 1.
       */
      void readTransNodes(const std::string& file);

      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWu(const fmatvec::Vec2 &zeta) override { return evalWs(zeta); }
      fmatvec::Vec3 evalWv(const fmatvec::Vec2 &zeta) override { return evalWt(zeta); }
      fmatvec::Vec3 evalWn(const fmatvec::Vec2 &zeta) override;

      void updatePositions(MBSim::ContourFrame *frame) override;
      void updateVelocities(MBSim::ContourFrame *frame) override;
      void updateJacobians(MBSim::ContourFrame *frame, int j=0) override;

      /*!
       * \brief returns the nodes for interpolation
       */
      fmatvec::MatVI getTransNodes();


      /*!
       * \brief set interpolation degree in U-direction
       */
      void setdegU(int deg);

      /*!
       * \brief set interpolation degree in V-direction
       */
      void setdegV(int deg);

      /*!
       * \brief set open or closed structure
       */
      void setOpenStructure(bool openstructure);

      /*!
       * \brief get open or closed structure
       */
      bool getOpenStructure();

      void resetUpToDate() override;

    protected:
      virtual NeutralNurbsVelocity2s* createNeutralVelocity();
      virtual NeutralNurbsPosition2s* createNeutralPosition();
      virtual NeutralNurbsLocalPosition2s* createNeutralLocalPosition();
      virtual void createNeutralModeShape();

      /*!
       * \brief Matrix of all node numbers that are interpolated
       *
       * Each row contains a list of node-numbers in u-direction of the contour (first direction)
       * REMARK: MBSim starts indexing with 0. Therefore in the set-nodes routine one index is substracted
       */
      fmatvec::MatVI transNodes;

      /*!
       * \brief offset between translation and rotational nodes
       */
      double nodeOffset;

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
