/* Copyright (C) 2004-2015 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef _FLEXIBLE_BODY_2S_13_MFR_MINDLIN_H_
#define _FLEXIBLE_BODY_2S_13_MFR_MINDLIN_H_

#include "mbsimFlexibleBody/flexible_body/2s_13.h"
#include "mbsimFlexibleBody/flexible_body/fe/2s_13_mfr_mindlin.h"

namespace MBSimFlexibleBody {

  class NodeFrame;

  /*! 
   * \brief plate according to Reissner-Mindlin with moving frame of reference and small tilting assumption
   * \author Kilian Grundl
   * \author Thorsten Schindler
   * \date 2009-12-23 initial commit (Schindler / Grundl)
   * \date 2010-04-21 parent class (Schindler / Grundl)
   * \date 2010-08-19 check (Schindler / Grundl)
   * \todo gravity TODO
   */
  class FlexibleBody2s13MFRMindlin : public FlexibleBody2s13 {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody2s13MFRMindlin(const std::string &name);

      /**
       * \brief destructor
       */
       ~FlexibleBody2s13MFRMindlin() override;

      /* INHERITED INTERFACE OF FLEXIBLE BODY CONTINUUM */
//      using FlexibleBodyContinuum<fmatvec::Vec>::addFrame;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
       void updateM() override;
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
       void BuildElements() override;
       void GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) override;
       void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) override;
       void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) override;

      fmatvec::Vec3 evalPosition() override;
      fmatvec::SqrMat3 evalOrientation() override;

       void updatePositions(Frame2s* frame) override;
       void updateVelocities(Frame2s* frame) override;
       void updateAccelerations(Frame2s* frame) override;
       void updateJacobians(Frame2s* frame, int j=0) override;
       void updateGyroscopicAccelerations(Frame2s* frame) override;

      void updatePositions(int node) override;
      void updateVelocities(int node) override;
      void updateAccelerations(int node) override;
      void updateJacobians(int node, int j=0) override;
      void updateGyroscopicAccelerations(int node) override;

      /* INHERITED INTERFACE OF OBJECT */
       void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLEBODY2s13 */
       fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint) override;
      /***************************************************/

    protected:
      /* INHERITED INTERFACE OF FLEXIBLEBODY2s13 */
       void initMatrices() override;
       void updateAG() override;
      /***************************************************/

      /*!
       * \brief calculate constant stiffness matrix
       */
      void computeStiffnessMatrix();

      /*! 
       * \brief calculate constant parts of the mass matrix
       */
      void computeConstantMassMatrixParts();
      
      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::Mat* N_compl;

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::SqrMat* N_ij[3][3];

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::RowVec* NR_ij[3][3];

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::Vec* R_compl;

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::SymMat* R_ij;
  };

}

#endif /* _FLEXIBLE_BODY_2S_13_MFR_MINDLIN_H_ */
