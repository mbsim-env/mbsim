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

#ifndef _FINITE_ELEMENT_1S_21_ANCF_H_
#define _FINITE_ELEMENT_1S_21_ANCF_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/mbsim_event.h"
#include "fmatvec/fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief differentiates a vector defined by its norm with respect to the vector
   * \param vector to be differentiated
   * \return Jacobian matrix
   */
  fmatvec::SqrMat differentiate_normalized_vector_respective_vector(const fmatvec::Vec &vector);

  /**
   * \brief finite element for planar beam using Absolute Nodal Coordinate Formulation (ANCF)
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
  class FiniteElement1s21ANCF : public DiscretizationInterface
  {
    public:
      /*!
       * \brief constructor 
       * \param undeformed lenght of element
       * \param line-density of beam
       * \param longitudinal stiffness
       * \param bending stiffness
       * \param vector of gravitational acceleration
       */
      explicit FiniteElement1s21ANCF(double sl0, double sArho, double sEA, double sEI, fmatvec::Vec sg, bool sEuler=false, double sv0=0.);

      /**
       * \destructor
       */
      virtual ~FiniteElement1s21ANCF();

      /* INHERITED INTERFACE */
      virtual const fmatvec::SymMat& getM() const { return M; }
      virtual const fmatvec::Vec& geth() const { return h; }
      virtual const fmatvec::SqrMat& getdhdq() const { return Dhq; }
      virtual const fmatvec::SqrMat& getdhdu() const { return Dhqp; }
      virtual int getqSize() const { return 8; }
      virtual int getuSize() const { return 8; }
      virtual void computeM(const fmatvec::Vec& qElement);
      virtual void computeh(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement);
      virtual void computedhdz(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement) { throw MBSim::MBSimError("(FiniteElement1s21ANCF::computedhdz): not implemented!"); }
      virtual double computeKineticEnergy(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement) { throw MBSim::MBSimError("(FiniteElement1s21ANCF::computeKineticEnergy): not implemented!"); }
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qElement) { throw MBSim::MBSimError("(FiniteElement1s21ANCF::computeGravitationalEnergy): not implemented!"); }
      virtual double computeElasticEnergy(const fmatvec::Vec& qElement) { throw MBSim::MBSimError("(FiniteElement1s21ANCF::computeElasticEnergy): not implemented!"); }
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp) { return JGeneralized(qElement,cp.getLagrangeParameterPosition()(0)); }
      /***************************************************/
      
      /**
       * compute additional informations for element
       */
      fmatvec::Vec computeAdditionalElementData(fmatvec::Vec &qElement, fmatvec::Vec &qpElement);

      /* GETTER / SETTER */
      void setCurlRadius(double R);
      void setMaterialDamping(double depsilon_, double dkappa_);
      /***************************************************/

      /**
       * \brief calculate constant mass matrix
       */
      void initM();

      /**
       * \brief return the planar position and angle at a contour point (Lagrange/Euler)
       * \param generalised coordinates
       * \param contour point (Lagrange/Euler)
       * \return planar position and angle
       */
      fmatvec::Vec LocateBalken(const fmatvec::Vec& qElement, const double& s); 

      /**
       * \brief return the planar state at a contour point (Lagrange/Euler)
       * \param generalised positions
       * \param generalised velocities
       * \param contour point (Lagrange/Euler)
       * \return planar state
       */
      fmatvec::Vec StateBalken(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const double&s); 

      /**
       * \brief return the JACOBIAN of translation and rotation with respect to generalised coordinates
       * \param generalised coordinates
       * \param contour point (Lagrange/Euler)
       * \return JACOBIAN of translation and rotation with respect to generalised coordinates
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec& qElement, const double& s);

      /**
       * \brief return the matrix of global shape functions
       * \param contour point (Lagrange/Euler)
       * \return matrix of global shape functions
       */
      fmatvec::Mat GlobalShapeFunctions(const double& s);

      /**
       * \brief return 1st derivative of the matrix of global shape functions
       * \param contour point (Lagrange/Euler)
       * \return 1st derivative of the matrix of global shape functions
       */
      fmatvec::Mat GlobalShapeFunctions_1stDerivative(const double& s);

      /**
       * \brief returns the tangent
       * \param generalised coordinates
       * \param contour point (Lagrange/Euler)
       * \return tangent
       * */
      fmatvec::Vec tangent(const fmatvec::Vec& qElement, const double& s);

    private:
      /**
       * \brief beam element length
       */
      double l0;

      /** 
       * \brief line-density
       */
      double Arho;

      /**
       * \brief longitudinal stiffness
       */
      double EA;

      /**
       * \brief bending stiffness
       */
      double EI;

      /** 
       * \brief Euler perspective: true if set
       */
      bool Euler;

      /**
       * \brief Euler perspective: constant longitudinal velocity
       */
      double v0;

      /**
       * \brief predefined bending curvature
       */
      double wss0;

      /**
       * \brief longitudinal damping
       */
      double depsilon;

      /**
       * \brief curvature damping
       */
      double dkappa;

      /**
       * \brief gravitation
       */
      fmatvec::Vec g;

      /**
       * \brief mass matrix
       */
      fmatvec::SymMat M;

      /**
       * \brief right hand side
       */
      fmatvec::Vec h;

      /**
       * \brief derivative of right hand side with respect to positions
       */
      fmatvec::SqrMat Dhq;

      /**
       * \brief derivative of right hand side with respect to velocities
       */
      fmatvec::SqrMat Dhqp;
  };

  inline void  FiniteElement1s21ANCF::computeM(const fmatvec::Vec& qG) { throw MBSim::MBSimError("(FiniteElement1s21ANCF::computeM): Not implemented"); }
}

#endif /* _FINITE_ELEMENT_1S_21_ANCF_H_ */
