/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: rzander@users.berlios.de
 *          thschindler@users.berlios.de
 */

#ifndef _FINITE_ELEMENT_1S_21_RCM_H_
#define _FINITE_ELEMENT_1S_21_RCM_H_

#include <mbsim/interfaces.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/mbsim_event.h>
#include "fmatvec.h"
using namespace fmatvec;

namespace MBSim {

  /**
   * \brief finite element for planar beam using Redundant Coordinate Method (RCM)
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-03-23 initial for kernel_dev
   *
   * read:\n
   * Zander, R.; Ulbrich, H.: Reference-free mixed FE-MBS approach for beam structures with constraints, Journal of Nonlinear Dynamics, Kluwer Academic Publishers, 2005\n
   * Zander, R.; Ulbrich, H.: Impacts on beam structures: Interaction of wave propagationand global dynamics, IUTAM Symposium on Multiscale Problems in Multibody System Contacts Stuttgart, Germany, 2006\n
   * Zander, R.; Ulbrich, H.: Free plain motion of flexible beams in MBS - A comparison of models, III European Conference on Computational Mechanics Lissbon, Portugal, 2006
   */
  class FiniteElement1s21RCM : public DiscretizationInterface {
    public:
      /**
       * \brief constructor
       */
      FiniteElement1s21RCM() {};

      /*!
       * \brief constructor 
       * \param undeformed lenght of element
       * \param line-density of beam
       * \param longitudinal stiffness
       * \param bending stiffness
       * \param vector of gravitational acceleration
       */
      FiniteElement1s21RCM(double l0_, double  Arho_, double EA_, double EI_, Vec g_);

      /**
       * \destructor
       */
      virtual ~FiniteElement1s21RCM() {}

      /* INHERITED INTERFACE */
      fmatvec::SymMat getMassMatrix() const { return M; }
      fmatvec::Vec getGeneralizedForceVector() const { return h; }
      fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingPosition() const { return Dhq; }    
      fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingVelocity() const { return Dhqp; }
      int getSizeOfPositions() const { return 8; }
      int getSizeOfVelocities() const { return 8; }
      void computeEquationsOfMotion(const Vec& qElement, const Vec& qpElement);
      double computeKineticEnergy(const Vec& qElement, const Vec& qpElement);
      double computeGravitationalEnergy(const Vec& qElement);
      double computeElasticEnergy(const Vec& qElement);
      Vec computeTranslation(const Vec&q, const ContourPointData& cp) { throw new MBSimError("ERROR (FiniteElement1s21RCM::computeTranslation): not implemented!"); }
      SqrMat computeOrientation(const Vec&q, const ContourPointData& cp) { throw new MBSimError("ERROR (FiniteElement1s21RCM::computeOrientation): not implemented!"); }
      Vec computeTranslationalVelocity (const Vec&q, const Vec&u, const ContourPointData& cp) { throw new MBSimError("ERROR (FiniteElement1s21RCM::computeTranslationalVelocity): not implemented!"); }
      Vec computeAngularVelocity(const Vec&q, const Vec&u, const ContourPointData& cp) { throw new MBSimError("ERROR (FiniteElement1s21RCM::computeAngularVelocity): not implemented!"); }
      Mat computeJacobianOfMinimalRepresentationRegardingPhysics(const Vec&q, const ContourPointData& cp) { return JGeneralized(q,cp.getLagrangeParameterPosition()(0)); }
      /***************************************************/

      /* GETTER / SETTER */
      void setCurlRadius(double);
      void setMaterialDamping(double);
      void setLehrDamping(double);
      void setImplicit(bool implicit_) { implicit = implicit_; }
      /***************************************************/

      /**
       * \param global positions
       * \param contour point
       * \return positional beam state
       */
      Vec LocateBeam(const Vec&q, const double &s);

      /**
       * \param global positions
       * \param global velocities
       * \param contour point
       * \return beam state
       */
      Vec StateBeam(const Vec&q, const Vec&u, const double &s);

      /**
       * \param global positions
       * \param contour point
       * \return JACOBIAN of beam cross section position with respect to generalised position in global coordinates
       */
      Mat JGeneralizedInternal(const Vec& qElement, const double& s);

      /**
       * \param global positions
       * \param contour point
       * \return JACOBIAN of beam cross section position with respect to generalised position in local coordinates
       */
      Mat JGeneralized (const Vec& qElement, const double& s);

      /**
       * \param global positions
       * \param global velocities
       * \param contour point
       * \param TODO
       * \return derivative of JACOBIAN of beam cross section position with respect to generalised position in local coordinates
       */
      Mat JpGeneralized(const Vec& qElement, const Vec& qpElement, const double& s,const double& sp);

      /**
       * \param global positions
       * \param global velocities
       * \return elongation, elongational velocity, cog position, cog velocity, bending angle sum, bending velocity sum
       */
      Vec ElementData(Vec qElement, Vec qpElement);

    protected:
      /** 
       * \brief length, density, longitudinal and bending stiffness
       */
      double l0, Arho, EA, EI;

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
      Vec g;

      /**
       * \brief mass matrix
       */
      SymMat M;

      /**
       * \brief right hand side
       */
      Vec h;

      /**
       * \brief derivative of right hand side with respect to positions and velocities
       */
      SqrMat Dhq, Dhqp;

      /**
       * \brief damping matrix
       */
      SqrMat Damp;

      /**
       * \brief FLAG for implicit integration
       */
      bool implicit;

    private:
      /**
       * \brief calculates the local beam coordinates
       * \param global coordinates
       * \param local coordinates
       */
      void BuildqLocal(const Vec& qGlobal, Vec& qLocal);

      /**
       * \brief calculates the JACOBIAN of transformation
       * \param local beam coordinates
       * \param JACOBIAN ot transformation
       */
      void BuildJacobi(const Vec& qLocal, SqrMat& Jeg);

      /**
       * \brief calculates the JACOBIAN of transformation and its time derivative 
       * \param local beam positions
       * \param local beam velocities
       * \param JACOBIAN of transformation
       * \param time derivative of JACOBIAN of transformation
       */
      void BuildJacobi(const Vec& qLocal, const Vec& qpIntern, SqrMat& Jeg, SqrMat& Jegp);

      /** 
       * \param local positions
       * \param local velocities
       * \param contour point
       * \param flag to calculate velocities
       * \return beam state
       */
      Vec LocateLocalBeam(const Vec& qLocal, const Vec& qpLocal, const double& s, const bool calcAll=true);

      /**
       * \param global positions
       * \param global velocities
       * \param local positions
       * \param local velocities
       * \param JACOBIAN of transformation
       * \param derivative of JACOBIAN of transformation
       * \param local mass matrix
       * \param local right hand side
       * \return JACOBIAN for implicit integration
       */
      Mat hFullJacobi(const Vec& qElement, const Vec& qpElement, const Vec& qLocal, const Vec& qpLocal, const SqrMat& Jeg, const SqrMat& Jegp, const SymMat& MLocal, const Vec& hIntermediate);

      /**
       * \brief powers of the beam length
       */
      double l0h2, l0h3, l0h4, l0h5, l0h7, l0h8;
  };

}

#endif

