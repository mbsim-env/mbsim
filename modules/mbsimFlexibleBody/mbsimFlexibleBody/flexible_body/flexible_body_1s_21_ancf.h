/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _FLEXIBLE_BODY_1S_21_ANCF_H_
#define _FLEXIBLE_BODY_1S_21_ANCF_H_

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"

namespace MBSimFlexibleBody {

  /**
   * \brief Absolute Nodal Coordinate Formulation for flexible planar beams
   * \author Roland Zander
   * \author Thorsten Schindler
   *
   * \date 2014-02-27 basic revision
   * \date 2014-03-23 damping added
   * \date 2014-05-09 Euler perspective added as an option
   *
   * model based on
   * SHABANA, A. A.: Computer Implementation of the Absolute Nodal Coordinate Formulation for Flexible Multibody Dynamics. In: Nonlinear Dynamics 16 (1998), S. 293-306
   * SHABANA, A. A.: Definition of the Slopes and the Finite Element Absolute Nodal Coordinate Formulation. In: Nonlinear Dynamics 1 (1997), S. 339-348
   * SHABANE, A. A.: Dynamics of Multibody Systems. Cambridge University Press (2005)
   */
  class FlexibleBody1s21ANCF : public FlexibleBody1s {

    public:
      /*!
       * \brief constructor:
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s21ANCF(const std::string &name, bool openStructure);

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void updateM(double t, int k) { }
      virtual void updateLLM(double t, int i=0) { }

      virtual void BuildElements();

      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);

      virtual fmatvec::Vec3 getPosition(double t, double s);
      virtual fmatvec::SqrMat3 getOrientation(double t, double s);
      virtual fmatvec::Vec3 getWs(double t, double s);

      virtual void updatePositions(double t, Frame1s* frame);
      virtual void updateVelocities(double t, Frame1s* frame);
      virtual void updateAccelerations(double t, Frame1s* frame);
      virtual void updateJacobians(double t, Frame1s* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, Frame1s* frame);

      virtual void updatePositions(double t, NodeFrame* frame);
      virtual void updateVelocities(double t, NodeFrame* frame);
      virtual void updateAccelerations(double t, NodeFrame* frame);
      virtual void updateJacobians(double t, NodeFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, NodeFrame* frame);
      /****************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s21ANCF"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);
      void setEModul(double E_) { E = E_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentInertia(double I_) { I = I_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCurlRadius(double rc_);
      void setMaterialDamping(double deps_, double dkappa_);
      void setEulerPerspective(bool Euler_, double v0_);
      int getNumberElements(){ return Elements; }
      /***************************************************/

      /**
       * \brief initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

      /**
       * \brief initialise beam state concerning a straight cantilever setting or a circle shaped ring
       * \param angle of slope in case of cantilever
       */
      void initRelaxed(double alpha);

    private:
      /**
       * \brief number of finite elements used for discretisation
       */
      int Elements;

      /**
       * \brief length of one finite element
       */
      double l0;

      /**
       * \brief modulus of linear elasticity
       */
      double E;

      /**
       * \brief cross-section area
       */
      double A;

      /**
       * \brief moment of inertia of cross-section
       */
      double I;

      /**
       * \brief material density
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       */
      double rc;

      /**
       * \brief coefficient of material longitudinal damping
       */
      double deps;

      /**
       * \brief coefficient of material longitudinal damping
       */
      double dkappa;

      /**
       * \brief flag for testing if beam is initialised
       */
      bool initialised;

      /**
       * \brief Euler perspective: constant longitudinal velocity
       */
      double v0;

      /**
       * \brief Euler perspective: true if set
       */
      bool Euler;

      /**
       * \brief initialize mass matrix and calculate Cholesky decomposition
       */
      void initM();

      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);

      fmatvec::Vec3 X;
      double sOld;
  };

}

#endif /* _FLEXIBLE_BODY_1S_21_ANCF_H_ */
