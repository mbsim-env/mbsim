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
      /*!
       * \brief standard constructor
       */
      Contour2sNeutralLinearExternalFFR(const std::string &name_);

      /*!
       * \brief constructor with all necessary data given
       */
      Contour2sNeutralLinearExternalFFR(const std::string &name_, FlexibleBodyLinearExternalFFR* parent_, Mat transNodes_, double nodeOffset_, int degU_, int degV_, bool openStructure_);

      /*!
       * \brief destructor
       */
      virtual ~Contour2sNeutralLinearExternalFFR();

      /*INHERITED INTERFACE */
      virtual void init(MBSim::InitStage stage);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual void updateStateDependentVariables(double t);
      /**/

      /* GETTER / SETTER*/
      int getNumberOfTransNodesU();
      int getNumberOfTransNodesV();
      void setTransNodes(const fmatvec::Mat & transNodes);
      void setdegU(int deg);
      void setdegV(int deg);

      /*!
       * \brief read data from a file that consists of sorted lists for nodes that should be interpolated
       *        Every line in the has to be a row of nodes in U-direction (the first direction of the interpolation)

       * \return A Matrix that consists of the list of node-numbers
       */
      static fmatvec::Mat readNodes(std::string file);

    protected:
      virtual NeutralNurbsVelocity2s* createNeutralVelocity();
      virtual NeutralNurbsPosition2s* createNeutralPosition();
      virtual NeutralNurbsLocalPosition2s* createNeutralLocalPosition();
      virtual void createNeutralModeShape();

      /*!
       * \brief Matrix of all node numbers that are interpolated
       *
       * Each row contains a list of node-numbers in u-direction of the contour (first direction)
       * REMARK: MBSim starts indexing with 0. If external programs (e.g. abaqus) start indexing with 1 the user has to substract one for each node-index
       */
      Mat transNodes;

      /*!
       * \brief offset between translation and rotational nodes
       */
      double nodeOffset;

      /*!
       * \brief Matrix for the contour-point datas for the single nodes
       */
      fmatvec::GeneralMatrix<ContourPointData> transContourPoints;
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
