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

#ifndef _FINITE_ELEMENT_1S_33_ANCF_H_
#define _FINITE_ELEMENT_1S_33_ANCF_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/mbsim_event.h"
#include "fmatvec/fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for spatial beam using Absolute Nodal Coordinate Formulation (ANCF)
   *
   * cable element with no explicit torsion, i.e. angular velocity and Jacobian of rotation depend only on two bending angles
   *
   * \author Thorsten Schindler
   *
   * \date 2014-03-20 initial commit
   *
   * model based on
   * DOMBROWSKI, S.: Analysis of Large Flexible Body Deformation in Multibody Systems Using Absolute Coordinates. Multibody System Dynamics (2002)
   */
  class FiniteElement1s33ANCF : public MBSim::DiscretizationInterface
  {
    public:
      /*!
       * \brief constructor 
       * \param undeformed lenght of element
       * \param cross-section area
       * \param density of beam
       * \param Young's modulus
       * \param shear modulus
       * \param polar moment of inertia
       * \param area moment of inertia
       * \param area moment of inertia
       * \param vector of gravitational acceleration
       */
      explicit FiniteElement1s33ANCF(double sl0, double srho, double sE, double sG, double sA, double sI0, double sI1, double sI2, fmatvec::Vec sg);

      /**
       * \destructor
       */
      virtual ~FiniteElement1s33ANCF();

      /* INHERITED INTERFACE */
      virtual const fmatvec::SymMat& getM() const { return M; }
      virtual const fmatvec::Vec& geth() const { return h; }
      virtual const fmatvec::SqrMat& getdhdq() const { return Dhq; }
      virtual const fmatvec::SqrMat& getdhdu() const { return Dhqp; }
      virtual int getqSize() const { return 12; }
      virtual int getuSize() const { return 12; }
      virtual void computeM(const fmatvec::Vec& qElement);
      virtual void computeh(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement);
      virtual void computedhdz(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s33ANCF::computedhdz): not implemented!"); }
      virtual double computeKineticEnergy(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s33ANCF::computeKineticEnergy): not implemented!"); }
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s33ANCF::computeGravitationalEnergy): not implemented!"); }
      virtual double computeElasticEnergy(const fmatvec::Vec& qElement) { throw MBSim::MBSimError("ERROR (FiniteElement1s33ANCF::computeElasticEnergy): not implemented!"); }
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const MBSim::ContourPointData& cp);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qElement, const MBSim::ContourPointData& cp) { return JGeneralized(qElement,cp.getLagrangeParameterPosition()(0)); }
      /***************************************************/

      /* GETTER / SETTER */
      void setCurlRadius(double R1, double R2);
      void setMaterialDamping(double depsilons);
      /***************************************************/

      /**
       * \brief calculate constant mass matrix
       */
      void initM();

      /**
       * \brief return the position and Cardan angles at a contour point
       * \param generalised coordinates
       * \param contour point
       * \return position and Cardan angles
       */
      fmatvec::Vec LocateBalken(const fmatvec::Vec& qElement, const double& s); 

      /**
       * \brief return the state including Cardan angles at a contour point
       * \param generalised positions
       * \param generalised velocities
       * \param contour point
       * \return state including Cardan angles
       */
      fmatvec::Vec StateBalken(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const double&s); 

      /**
       * \brief return the JACOBIAN of translation and rotation with respect to generalised coordinates
       * \param generalised coordinates
       * \param contour point
       * \return JACOBIAN of translation and rotation with respect to generalised coordinates
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec& qElement, const double& s);

      /**
       * \brief return the matrix of global shape functions
       * \param contour point
       * \return matrix of global shape functions
       */
      fmatvec::Mat GlobalShapeFunctions(const double& s);

      /**
       * \brief returns the tangent
       * \param generalised coordinates
       * \param contour point
       * \return tangent
       * */
      fmatvec::Vec tangent(const fmatvec::Vec& qElement, const double& s);

      /**
       * \brief returns the normal
       * \param generalised coordinates
       * \param contour point
       * \return normal
       * */
      fmatvec::Vec normal(const fmatvec::Vec& qElement, const double& s);

      /**
       * \brief returns the binormal
       * \param generalised coordinates
       * \param contour point
       * \return binormal
       * */
      fmatvec::Vec binormal(const fmatvec::Vec& qElement, const double& s);

    private:
      /** 
       * \brief beam element length
       */
      double l0;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief Young's modulus
       */
      double E;

      /**
       * \brief shear modulus
       */
      double G;

      /** 
       * \brief cross-secion area
       */
      double A;

      /**
       * \brief polar moment of inertia
       */
      double I0;

      /**
       * \brief area moment of inertia
       */
      double I1;

      /**
       * \brief area moment of inertia
       */
      double I2;

      /**
       * \brief predefined bending curvature
       */
      double wss01;

      /**
       * \brief predefined bending curvature
       */
      double wss02;

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
      FiniteElement1s33ANCF(const FiniteElement1s33ANCF&);

      /**
       * \brief assignment operator is declared private
       */
      FiniteElement1s33ANCF& operator=(const FiniteElement1s33ANCF&);
  };

  inline void  FiniteElement1s33ANCF::computeM(const fmatvec::Vec& qG) { throw MBSim::MBSimError("Error(FiniteElement1s33ANCF::computeM): Not implemented"); }
}

#endif /* _FINITE_ELEMENT_1S_33_ANCF_H_ */
