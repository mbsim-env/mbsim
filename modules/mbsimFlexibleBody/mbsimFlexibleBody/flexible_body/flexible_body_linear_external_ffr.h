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

#ifndef _FLEXIBLE_BODY_LINEAR_EXTERNAL_FFR_H_
#define _FLEXIBLE_BODY_LINEAR_EXTERNAL_FFR_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>

#include "mbsim/frames/frame.h"

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_linear_external_lumped_node.h"

#include "mbsimFlexibleBody/frames/node_frame.h"

//namespace unitTest{
//  class linearExternalFFRTest;
//}

namespace MBSimFlexibleBody {
  
  /*! 
   * \brief flexible body model based on floating frame reference using the system information external linear FEM code
   * \author Kilian Grundl
   * \author Zhan Wang
   * \date
   */
  class FlexibleBodyLinearExternalFFR : public FlexibleBodyContinuum<fmatvec::Vec> /*, public FlexibleContourBody */{
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBodyLinearExternalFFR(const std::string &name, const bool &DEBUG_);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBodyLinearExternalFFR();

      /* INHERITED INTERFACE OF FLEIBLE BODY CONTOUR */
      virtual int getNumberElements() const { return nNodes; }
      virtual double getLength() const;
      virtual bool isOpenStructure() const { return openStructure; }
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLE BODY CONTINUUM */
      using FlexibleBodyContinuum<fmatvec::Vec>::addFrame;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateM(int k = 0);
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateh(int k = 0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      virtual void calcqSize();
      virtual void calcuSize(int j);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const {
        return "FlexibleBodyLinearExternalFFR";
      }
      /***************************************************/

      /* GETTER AND SETTER */
      void setNumberOfModes(int nf_) {
        nf = nf_;
      }

      int getNumberModes() const {
        return nf;
      }

      MBSim::Frame * getFloatingFrameOfReference() {
        return FFR;
      }

      const fmatvec::Vec3 getModeShapeVector(int node, int column) const;

      /*!
       * \brief  read u0, mij, mode shape matrix and stiffness matrix form the input file
       */
      void readFEMData(std::string inFilePath, const bool millimeterUnits, bool output = false);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableFramePlot(double size = 1e-3, fmatvec::VecInt numbers = fmatvec::VecInt(0));
#endif

      void resetUpToDate();
      const fmatvec::SqrMat3& evalA() { if(updAG) updateAGbarGbardot(); return A; }
      const fmatvec::SqrMat3& evalG_bar() { if(updAG) updateAGbarGbardot(); return G_bar; }
      const fmatvec::SqrMat3& evalG_bar_Dot() { if(updAG) updateAGbarGbardot(); return G_bar_Dot; }
      const fmatvec::Vec& evalQv() { if(updQv) updateQv(); return Qv; }

      void updatePositions(MBSim::Frame *frame);
      void updateVelocities(MBSim::Frame *frame);
      void updateAccelerations(MBSim::Frame *frame);
      void updateJacobians(MBSim::Frame *frame, int j=0);
      void updateGyroscopicAccelerations(MBSim::Frame *frame);

      void updatePositions(NodeFrame *frame);
      void updateVelocities(NodeFrame *frame);
      void updateAccelerations(NodeFrame *frame);
      void updateJacobians(NodeFrame *frame, int j=0);
      void updateGyroscopicAccelerations(NodeFrame *frame);

      fmatvec::Vec3 evalLocalPosition(int i);

    protected:
      /*!
       * \brief  initialize the mass matrix
       */
      void initM(int k = 0);

      /*!
       * \brief  initialize the quadratic velocity vector
       */
      void initQv();

      /*! 
       * \brief update the quadratic velocity vector
       */
      void updateQv();

      /*!
       * \brief compute the constant shape integrals of the whole body
       */
      void computeShapeIntegrals();

      /*!
       * \brief  update A, G_bar, and G_bar_Dot
       */
      void updateAGbarGbardot();

      /**
       * \brief total number of nodes
       */
      int nNodes;

      /*!
       * \brief Floating Frame of Reference
       */
      MBSim::Frame * FFR;

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::Vec3 I_1;

      /**
       * \brief matrix
       */
      fmatvec::SymMat3 I_kl;

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::Mat S_bar;

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::SqrMat S_kl_bar[3][3];

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::RowVec I_kl_bar[3][3];

      /*!
       * \brief inertia tensor
       */
      fmatvec::SymMat3 I_ThetaTheta_bar;

      /*!
       * \brief I_ThetaF 3*nf
       */
      fmatvec::Mat I_ThetaF_bar;

      /**
       * \brief stiffness matrix
       */
      fmatvec::SymMat K;

      /**
       * \brief full stiffness matrix
       */
      fmatvec::SymMat KFull;

      /**
       * \brief transformation matrix of the time derivates of the angles into tho angular velocity in reference coordinates
       */
      fmatvec::SqrMat3 G_bar;

      /**
       * \brief transformation matrix of the time derivates of the angles into tho angular velocity in reference coordinates
       */
      fmatvec::SqrMat3 G_bar_Dot;

      /**
       * \brief transformation matrix of coordinates of the moving frame of reference into the reference frame
       */
      fmatvec::SqrMat3 A;

      /**
       * \brief matrix for the computation of the mass-matrix (assembled part of the element matrix)
       */
      fmatvec::SymMat M_FF;

      /**
       * \brief quadratic velocity vector
       */
      fmatvec::Vec Qv;

      /**
       * \brief lumped mass for each node
       */
      fmatvec::Vec mij;

      /**
       * \brief matrix of all modes
       */
      fmatvec::Mat phiFull;

      /**
       * \brief matrix of modes used
       */
      fmatvec::Mat phi;

      /**
       * \brief initial position of each node
       */
      std::vector<fmatvec::Vec3> u0;

      /**
       * \brief number of mode shapes used to describe the deformation
       */
      int nf;

      /**
       * \brief open or closed beam structure
       */
      bool openStructure;

      /**
       * \brief debug flag
       */
      bool DEBUG;

      bool updAG, updQv;
  };
  
}

#endif /* _FLEXIBLE_BODY_LINEAR_EXTERNAL_FFR_H_ */
