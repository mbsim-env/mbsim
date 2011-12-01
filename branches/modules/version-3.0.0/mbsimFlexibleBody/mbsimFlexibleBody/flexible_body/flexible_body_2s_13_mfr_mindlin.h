/* Copyright (C) 2004-2011 MBSim Development Team
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

#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.h"

namespace MBSimFlexibleBody {

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
      FlexibleBody2s13MFRMindlin(const std::string &name, const int & DEBUGLEVEL_ = 0);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody2s13MFRMindlin();

      /* INHERITED INTERFACE OF FLEXIBLE BODY CONTINUUM */
      using FlexibleBodyContinuum<fmatvec::Vec>::addFrame;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateM(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBody2s13MFRMindlin"; }
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLEBODY2s13 */
      virtual fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint);
      /***************************************************/

    protected:
      /* INHERITED INTERFACE OF FLEXIBLEBODY2s13 */
      virtual void initMatrices();
      virtual void updateAG();
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

  inline void FlexibleBody2s13MFRMindlin::GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) { throw MBSim::MBSimError("ERROR(FlexibleBody2s13MFRMindlin::GlobalVectorContribution): Not implemented!"); }
  inline void FlexibleBody2s13MFRMindlin::GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) { throw MBSim::MBSimError("ERROR(FlexibleBody2s13MFRMindlin::GlobalMatrixContribution): Not implemented!"); }
  inline void FlexibleBody2s13MFRMindlin::GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) { throw MBSim::MBSimError("ERROR(FlexibleBody2s13MFRMindlin::GlobalMatrixContribution): Not implemented!"); }

}

#endif /* _FLEXIBLE_BODY_2S_13_MFR_MINDLIN_H_ */

