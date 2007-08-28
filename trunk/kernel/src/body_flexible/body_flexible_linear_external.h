/* Copyright (C) 2005-2006  Roland Zander
 
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
 * Contact:
 *   rzander@users.berlios.de
 *
 */

#ifndef _BODY_FLEXIBLE_LINEAR_EXTERNAL_H_
#define _BODY_FLEXIBLE_LINEAR_EXTERNAL_H_

#include "body_flexible.h"
#include <fstream>

namespace MBSim {

  class ContourInterpolation;


  /*! 
   * \brief
   * Linear models from external preprocessing, e.g. Finite %Element model.
   *
   * All inputs follow fmatvec syntax, e.g. MatLab syntax.
   *
   * Systems equation of motion:
   * \f[ \vM{\rm d}\vu = -( \vK \vq + \vD \vu){\rm d}t - \vW{\rm d}\vLambda \f]
   * with constant matrices \f$\vM,\vK,\vD\f$. The model uses n degrees of freedom (dimension of \f$\vq,\vu\f$) and d translational directions (dimension of Jacobi-matrizes \f$n\times d\f$)
   * */
  class BodyFlexibleLinearExternal : public BodyFlexible {

    protected:
      /** constant mass matrix \f$\vM\f$, used as buffer before init() where dimensions of M are set */ 
      SymMat Mread;
      /** constant stiffness matrix \f$\vK\f$*/ 
      SqrMat K;
      /** constant damping matrix \f$\vD\f$, see setProportionalDamping()*/ 
      SqrMat D; 
      /** constant for damping matrix, see setProportionalDamping()*/ 
      double alpha;
      /** constant for damping matrix, see setProportionalDamping()*/ 
      double  beta;

      /** number of Contours directly asoziated to body */
      int nContours;

      /** origin \f$\rs{_W}[_K0]{\vr}\f$ of model */
      Vec WrON00;

      /** container holding constant JACOBIAN-matrizes of all Port s and Contour s */
      vector<Mat> J;
      /** container holding undeformed positions in body coordinate system of all Port s and Contour s */
      vector<Vec> KrP;

      /* geerbt */
      void init();

      /*! update kinematical values\n
       * Call to updatePorts(double t)
       * and updateContours(double t)
       * \param t time
       */
      void updateKinematics(double t);
      /*!
       * Kinematical update of postition and velocities of every Port:
       * \f[ \vr_{P} = \vr_{K} + \vJ_T(\vr_{P0} + \vW^T\vq) \f]
       * \param t time
       * \todo angular kinematics
       */
      void updatePorts(double t);
      /*!
       * Kinematical update of postition and velocities of every Contour:
       * \f[ \vr_{C} = \vr_{K} + \vJ_T(\vr_{P0} + \vW^T\vq) \f]
       * \param t time
       * \todo angular kinematics
       */
      void updateContours(double t);

      /*! 
       * update \f$\vh= -(\vK \vq + \vD \vu)\f$
       * \param t time
       */
      void updateh(double t);

      /*! create interface in form of ContourPointData based on file
       * \return cpData for refering to Port or Contour added
       * \param jacbifilename file containing interface data
       */
      ContourPointData addInterface(const string &jacbifilename);
      /*! create interface in form of ContourPointData based on Jacobian matrix and undeformed position
       * \return cpData for refering to Port or Contour added
       * \param J_ Jacobian matrix
       * \param r_ undeformed position in body coordinate system
       */
      ContourPointData addInterface(const Mat &J_, const Vec &r_);

    public:
      BodyFlexibleLinearExternal(const string &name); 

      /*! \return true
      */
      bool hasConstMass() const {return true;}

      /*! 
       * read mass matrix: style e.g. for 2*2 matrix \n
       * n x n   \n
       *[1.0 0.0 \n
       * 0.0 1.0]
       * \param massfilename name of file holding mass matrix
       */
      void readMassMatrix(const string &massfilename); 

      /*! 
       * set mass matrix: 
       * \param mat mass matrix
       */
      void setMassMatrix(const SymMat &mat); 

      /*!
       * read stiffness matrix form given file readMassMatrix(const string &massfilename)
       * \param stiffnessfilename name of file holding stiffness matrix
       */
      void readStiffnessMatrix(const string &stiffnessfilename); 

      /*!
       * read stiffness matrix
       * \param mat stiffness matrix 
       */
      void setStiffnessMatrix(const SqrMat &mat); 

      /*!
       * set damping \f$\vD\f$ proportional to mass and stiffness
       * \f[ \vD = \alpha * \vM + \beta*\vK \f]
       * \param a_ \f$\alpha\f$
       * \param b_ \f$\beta \f$
       */
      void setProportionalDamping(const double &a_, const double &b_){alpha = a_; beta = b_;}

      /* geerbt */
      double computePotentialEnergy();

      /*! plot parameters of "BodyFlexibleLinearExternal"
      */
      void plotParameters();

      using BodyFlexible::addPort;
      /*!
       * add Port using JACOBIAN matrix and port location form given file
       * n x d   \n   [1.0  \n  0.0 ] \n
       * \n
       * d x 1   \n   [1.0  \n  0.0   \n  0.0 ]
       * \param name name of Port create
       * \param jacobifilename name of file holding matrices
       */
      void addPort(const string &name, const string &jacobifilename);

      /*!
       * add Port using JACOBIAN matrix and location of reference point
       * \param name of Port to 
       * \param J Jacobian matrix create
       * \param r undeformed reference point in body coordinate system
       */
      void addPort(const string &name, const Mat &J_, const Vec &r_);

      /*!
       * add Contour using information given in file for JACOBIAN matrix and location of reference point
       * \param contour Contour to add
       * \param jacobifilename name of file holding matrices
       */
      void addContour(Contour *contour, const string &jacobifilename);

      /*!
       * add Contour using JACOBIAN matrix and location of reference point
       * \param contour Contour to add
       * \param J Jacobian matrix
       * \param r undeformed reference point in body coordinate system
       */
      void addContour(Contour *contour, const Mat &J_, const Vec &r_);

      /*! add a ContourInterpolation, no additional information needed */
      void addContourInterpolation(ContourInterpolation *contour);

      /*! set origin BodyFlexibleLinearExternal::WrON00, \f$\rs{_W}[_K0]{\vr}\f$ of model
      */
      void setWrON00(const Vec &WrON00_) {WrON00 = WrON00_;}

      /* geerbt */
      Mat computeJacobianMatrix(const ContourPointData &CP);

      /*! \return zero matrix, BodyFlexibleLinearExternal does not hold any contour, no tangent defined
      */
      Mat computeWt  (const ContourPointData& CP) {return Mat(3,2);};
      /*! \return zero matrix, BodyFlexibleLinearExternal does not hold any contour, no normal defined 
      */ 
      Vec computeWn  (const ContourPointData& CP) {return Vec(3);}
      /*! compute position \f$\rs{_W}[_{CP}]{\vr}\f$ of Contour-Point
	\param ContourPointData& C
	\return \f$\rs{_W}[_{CP}]{\vr}\f$
	*/
      Vec computeWrOC(const ContourPointData& CP);
      /*! compute absolut velocity \f$\rs{_W}[_{P}]{\vv}\f$ of Contour-Point
	\param ContourPointData& C
	\return \f$\rs{_W}[_P]{\vv}\f$
	*/
      Vec computeWvC (const ContourPointData& CP);
      /*! compute absolut angular velocity \f$\rs{_W}[_{P}]{\boldsymbol{\omega}}\f$ of Contour-Point
	\param ContourPointData& C
	\return \f$\rs{_W}[_P]{\boldsymbol{\omega}}\f$
	*/
      Vec computeWomega(const ContourPointData& CP);

      /* inherted */ 
      SqrMat computeAWK (const ContourPointData &data) {return SqrMat(3,INIT,0.0);} 
      SqrMat computeAWKp(const ContourPointData &data) {return SqrMat(3,INIT,0.0);} 
   };

}

#endif
