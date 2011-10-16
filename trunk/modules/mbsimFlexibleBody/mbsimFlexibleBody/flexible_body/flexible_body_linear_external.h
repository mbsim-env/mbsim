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
 *          rzander@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_LINEAR_EXTERNAL_H_
#define _FLEXIBLE_BODY_LINEAR_EXTERNAL_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsim/mbsim_event.h"
#include <fstream>

namespace MBSimFlexibleBody {

  class ContourInterpolation;

  /*! 
   * \brief Linear models from external preprocessing, e.g. Finite Element model.
   * \author Roland Zander
   * \date 2009-03-26 initial kernel_dev commit (Roland Zander)
   * \date 2009-04-05 minor change: parent class is not template class (Schindler / Zander)
   * \date 2009-07-23 implicit integration (Thorsten Schindler)
   *
   * Systems equation of motion:
   * \f[ \vM{\rm d}\vu = -( \vK \vq + \vD \vu){\rm d}t - \vW{\rm d}\vLambda \f]
   * with CONSTANT matrices \f$\vM,\vK,\vD\f$. The model uses n degrees of freedom (dimension of \f$\vq,\vu\f$) and d translational directions (dimension of Jacobi-matrizes \f$n\times d\f$)
   */
  class FlexibleBodyLinearExternal : public FlexibleBody {
    public:
	  /*!
	   * \brief constructor
	   * \param name of body
	   */
      FlexibleBodyLinearExternal(const std::string &name);

      /*!
       * \brief destructor
       */
      virtual ~FlexibleBodyLinearExternal() {}

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBodyLinearExternal"; }
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage) { new MBSim::MBSimError("ERROR(FlexibleBodyLinearExternal::init): Not implemented!"); }
      virtual void facLLM() {}
      /***************************************************/
      
      /* INHERITED INTERFACE OF FLEXIBLEBODY */
      virtual void BuildElements() { new MBSim::MBSimError("ERROR(FlexibleBodyLinearExternal::BuildElements): Not implemented!"); } 
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      /*!
       * Kinematical update of postition and velocities of every attached Frame:
       * \f[ \vr_{P} = \vr_{K} + \vJ_T(\vr_{P0} + \vW^T\vq) \f]
       * \param t time
       * \todo angular kinematics
       */
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      /***************************************************/

      /*! 
       * read mass matrix: style e.g. for 2*2 matrix \n
       * n x n   \n
       *[1.0 0.0 \n
       * 0.0 1.0]
       * \param massfilename name of file holding mass matrix
       */
      void readMassMatrix(const std::string &massfilename); 

      /*! 
       * set mass matrix: 
       * \param mat mass matrix
       */
      void setMassMatrix(const fmatvec::SymMat &mat); 

      /*!
       * read stiffness matrix form given file readMassMatrix(const std::string &massfilename)
       * \param stiffnessfilename name of file holding stiffness matrix
       */
      void readStiffnessMatrix(const std::string &stiffnessfilename); 

      /*!
       * read stiffness matrix
       * \param mat stiffness matrix 
       */
      void setStiffnessMatrix(const fmatvec::SqrMat &mat); 

      /*!
       * set damping \f$\vD\f$ proportional to mass and stiffness
       * \f[ \vD = \alpha * \vM + \beta*\vK \f]
       * \param a_ \f$\alpha\f$
       * \param b_ \f$\beta \f$
       */
	  void setProportionalDamping(const double &a, const double &b);

//      /*! plot parameters of "BodyFlexibleLinearExternal"
//      */
//      void plotParameters();

      using FlexibleBody::addFrame;
      /*!
       * add Port using JACOBIAN matrix and frame location form given file
       * n x d   \n   [1.0  \n  0.0 ] \n
       * \n
       * d x 1   \n   [1.0  \n  0.0   \n  0.0 ]
       * \param name name of Port create
       * \param jacobifilename name of file holding matrices
       */
//      void addFrame(const std::string &name, const std::string &jacobifilename);

      /*!
       * add Port using JACOBIAN matrix and location of reference point
       * \param name of Port to 
       * \param J Jacobian matrix create
       * \param r undeformed reference point in body coordinate system
       */
//      void addFrame(const std::string &name, const fmatvec::Mat &J_, const fmatvec::Vec &r_);

      /*!
       * add Contour using information given in file for JACOBIAN matrix and location of reference point
       * \param contour Contour to add
       * \param jacobifilename name of file holding matrices
       */
//      void addContour(Contour *contour, const std::string &jacobifilename);

      /*!
       * add Contour using JACOBIAN matrix and location of reference point
       * \param contour Contour to add
       * \param J Jacobian matrix
       * \param r undeformed reference point in body coordinate system
       */
//      void addContour(Contour *contour, const fmatvec::Mat &J_, const fmatvec::Vec &r_);

      /*! add a ContourInterpolation, no additional information needed */
//      void addContourInterpolation(ContourInterpolation *contour);

      /*! set origin BodyFlexibleLinearExternal::WrON00, \f$\rs{_W}[_K0]{\vr}\f$ of model
      */
      void setWrON00(const fmatvec::Vec &WrON00_) {WrON00 = WrON00_;}

      /* geerbt */
      fmatvec::Mat computeJacobianMatrix(const MBSim::ContourPointData &CP);

    protected:
      /** number of Contours directly asoziated to body */
      int nContours;
	  
	  /** vector of ContourPointData controlling type of interface: node/interpolation */
      std::vector<MBSim::ContourPointData> contourType;

      /** origin \f$\rs{_W}[_K0]{\vr}\f$ of model */
      fmatvec::Vec WrON00;

      /*! update kinematical values\n
       * Call to updateFrames(double t)
       * and updateContours(double t)
       * \param t time
       */
      void updateStateDependentVariables(double t);
      /*!
       * Kinematical update of postition and velocities of every Contour:
       * \f[ \vr_{C} = \vr_{K} + \vJ_T(\vr_{P0} + \vW^T\vq) \f]
       * \param t time
       * \todo angular kinematics
       */
      void updateContours(double t);

      /*! create interface in form of ContourPointData based on file
       * \return cpData for refering to Port or Contour added
       * \param jacbifilename file containing interface data
       */
      MBSim::ContourPointData addInterface(const std::string &jacbifilename);
      /*! create interface in form of ContourPointData based on Jacobian matrix and undeformed position
       * \return cpData for refering to Port or Contour added
       * \param J Jacobian matrix
       * \param r undeformed position in body coordinate system
       */
      MBSim::ContourPointData addInterface(const fmatvec::Mat &J, const fmatvec::Vec &r);

      /* empty function since mass, damping and stiffness matrices are constant !!! */
      void updateJh_internal(double t);
   };

}

#endif /* _FLEXIBLE_BODY_LINEAR_EXTERNAL_H_ */

