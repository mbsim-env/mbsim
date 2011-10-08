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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _FINITE_ELEMENT_1S_33_COSSERAT_H_
#define _FINITE_ELEMENT_1S_33_COSSERAT_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for spatial beam using Cosserat model
   * \author Thorsten Schindler
   * \author Christian Käsbauer
   * \author Thomas Cebulla
   * \date 2011-09-10 initial commit (Thorsten Schindler)
   * \data 2011-10-08 basics derived and included (Thorsten Schindler)
   * \todo bending/torsion in rhs TODO
   * \todo gyroscopic rotational stuff in rhs TODO
   * \todo contact Jacobian TODO
   * \todo computeState only in FlexibleBody TODO
   * \todo implicit integration TODO
   *
   * Cosserat model based on
   * H. Lang, J. Linn, M. Arnold: Multi-body dynamics simulation of geometrically exact Cosserat rods
   * but with 
   *  - Kirchhoff assumption (-> less stiff)
   *  - Cardan parametrisation (-> less problems with condition and drift for quaternion dae system)
   *  - piecewise constant Darboux vector with evaluation according to
   *    I. Romero: The interpolation of rotations and its application to finite element models of
   *    geometrically exact beams
   */
  class FiniteElement1s33Cosserat : public MBSim::DiscretizationInterface {
    public:

      /**
       * \brief constructor
       * \param length of finite element
       * \param density
       * \param cross-sectional area
       * \param Young's modulus
       * \param shear modulus
       * \param first area moment of inertia
       * \param second area moment of inertia
       * \param torsional moment of inertia
       * \param acceleration of gravity
       */
      FiniteElement1s33Cosserat(double l0_,double rho_,double A_,double E_,double G_,double I1_,double I2_,double I0_,const fmatvec::Vec& g_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s33Cosserat();		

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */ 
      virtual const fmatvec::SymMat& getM() const;		
      virtual const fmatvec::Vec& geth() const;
      virtual const fmatvec::SqrMat& getdhdq() const;
      virtual const fmatvec::SqrMat& getdhdu() const;
      virtual int getqSize() const;
      virtual int getuSize() const;

      virtual void computeM(const fmatvec::Vec& qG);
      virtual void computeh(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qG);
      virtual double computeElasticEnergy(const fmatvec::Vec& qG);

      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG, const MBSim::ContourPointData& data);

      /* GETTER / SETTER */
      void setCurlRadius(double R1,double R2);		
      void setMaterialElongationDamping(double cEps0D_);		
      double getl0() const;

      /**
       * \brief compute state of finite element with a linear centerline
       * \param global coordinates
       * \param global velocities
       * \param LAGRANGIAN parameter in [0,l0]
       */
      const fmatvec::Vec& computeState(const fmatvec::Vec& qG, const fmatvec::Vec& qGt, double s);
      
      /**
       * \brief initialize translational part of mass matrix
       */
      void initM();

    private:
      /**
       * \brief length of finite element
       */
      double l0;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief cross sectional area 
       */
      double A;

      /**
       * \brief Young's modulus 
       */
      double E;

      /**
       * \brief shear modulus 
       */
      double G;

      /**
       * \brief geometrical moment of inertia 
       */
      double I1, I2, I0;

      /**
       * \brief gravitation
       */
      fmatvec::Vec g;

      /**
       * \brief predefined bending 
       */
      double k10, k20;

      /**
       * \brief elongational damping
       */
      double cEps0D;

      /**
       * \brief global system description 
       */
      fmatvec::SymMat M;
      fmatvec::Vec h;

      /**
       * \brief matrices for implicit integration 
       */
      fmatvec::SqrMat dhdq, dhdu;

      /**
       * \brief state at Lagrangian coordinate
       */
      fmatvec::Vec X;
      
      FiniteElement1s33Cosserat(); // standard constructor
      FiniteElement1s33Cosserat(const FiniteElement1s33Cosserat&); // copy constructor
      FiniteElement1s33Cosserat& operator=(const FiniteElement1s33Cosserat&); // assignment operator
  };

  inline const fmatvec::SymMat& FiniteElement1s33Cosserat::getM() const { return M; }
  inline const fmatvec::Vec& FiniteElement1s33Cosserat::geth() const { return h; }
  inline const fmatvec::SqrMat& FiniteElement1s33Cosserat::getdhdq() const { return dhdq; }
  inline const fmatvec::SqrMat& FiniteElement1s33Cosserat::getdhdu() const { return dhdu; }
  inline int FiniteElement1s33Cosserat::getqSize() const { return 9; }
  inline int FiniteElement1s33Cosserat::getuSize() const { return 9; }
  inline void  FiniteElement1s33Cosserat::computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt){throw MBSim::MBSimError("Error(FiniteElement1s33Cosserat::dhdz): Not implemented");}
  inline fmatvec::Vec FiniteElement1s33Cosserat::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33Cosserat::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat FiniteElement1s33Cosserat::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33Cosserat::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s33Cosserat::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33Cosserat::computeVelocity): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s33Cosserat::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33Cosserat::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElement1s33Cosserat::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33Cosserat::computeJacobianOfMotion): Not implemented!"); }
  inline double FiniteElement1s33Cosserat::getl0() const { return l0; }

}

#endif /* _FINITE_ELEMENT_1S_33_COSSERAT_H_ */
