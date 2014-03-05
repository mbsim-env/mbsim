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
   * \brief finite element for planar beam using Absolute Nodal Coordinate Formulation (ANCF)
   * \author Roland Zander
   * \author Thorsten Schindler
   *
   * \date 2014-02-27 basic revision
   *
   * model based on
   * SHABANA, A. A.: Computer Implementation of the Absolute Nodal Coordinate Formulation for Flexible Multibody Dynamics. In: Nonlinear Dynamics 16 (1998), S. 293-306
   * SHABANA, A. A.: Definition of the Slopes and the Finite Element Absolute Nodal Coordinate Formulation. In: Nonlinear Dynamics 1 (1997), S. 339-348
   * SHABANE, A. A.: Dynamics of Multibody Systems. Cambridge University Press (2005)
   */
  class FiniteElement1s21ANCF : public MBSim::DiscretizationInterface
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
      explicit FiniteElement1s21ANCF(double sl0, double sArho, double sEA, double sEI, fmatvec::Vec sg);

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
      virtual void computedhdz(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s21ANCF::computedhdz): not implemented!"); }
      virtual double computeKineticEnergy(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s21ANCF::computeKineticEnergy): not implemented!"); }
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s21ANCF::computeGravitationalEnergy): not implemented!"); }
      virtual double computeElasticEnergy(const fmatvec::Vec& qElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s21ANCF::computeElasticEnergy): not implemented!"); }
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp) { throw MBSim::MBSimError("ERROR (FiniteElement1s21ANCF::computePosition): not implemented!"); }
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp) { throw MBSim::MBSimError("ERROR (FiniteElement1s21RCM::computeOrientation): not implemented!"); }
      virtual fmatvec::Vec computeVelocity (const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const MBSim::ContourPointData& cp) { throw MBSim::MBSimError("ERROR (FiniteElement1s21RCM::computeVelocity): not implemented!"); }
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const MBSim::ContourPointData& cp) { throw MBSim::MBSimError("ERROR (FiniteElement1s21RCM::computeAngularVelocity): not implemented!"); }
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp) { return JGeneralized(qElement,cp.getLagrangeParameterPosition()(0)); }
      /***************************************************/

      /* GETTER / SETTER */
      void setCurlRadius(double R);
      void setMaterialDamping(double depsilons);
      void setLehrDamping(double D);
      /***************************************************/

      /**
       * \brief calculate constant mass matrix
       */
      void initM();

      /**
       * \brief return the planar position and angle at a contour point
       * \param generalised coordinates
       * \param contour point
       * \return planar position and angle
       */
      fmatvec::Vec LocateBalken(const fmatvec::Vec& qElement, const double& s); 

      /**
       * \brief return the planar state at a contour point
       * \param generalised positions
       * \param generalised velocities
       * \param contour point
       * \return planar state
       */
      fmatvec::Vec StateBalken(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const double&s); 

      /**
       * \brief return the JACOBIAN of translation and rotation with respect to generalised coordinates
       * \param generalised coordinates
       * \param contour point
       * \return JACOBIAN of translation and rotation with respect to generalised coordinates
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec& qElement, const double& s);

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
       * \brief predefined bending curvature
       */
      double wss0;

      /**
       * \brief longitudinal damping
       */
      double depsilon;

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
       * \brief damping matrix
       */
      fmatvec::SqrMat Damp;

      /**
       * \brief derivative of right hand side with respect to positions
       */
      fmatvec::SqrMat Dhq;

      /**
       * \brief derivative of right hand side with respect to velocities
       */
      fmatvec::SqrMat Dhqp;

      /**
       * \brief copy constructor is declared private
       */
      FiniteElement1s21ANCF(const FiniteElement1s21ANCF&);

      /**
       * \brief assignment operator is declared private
       */
      FiniteElement1s21ANCF& operator=(const FiniteElement1s21ANCF&);
  };

  inline void  FiniteElement1s21ANCF::computeM(const fmatvec::Vec& qG) { throw MBSim::MBSimError("Error(FiniteElement1s21ANCF::computeM): Not implemented"); }
}

#endif /* _FINITE_ELEMENT_1S_21_ANCF_H_ */
