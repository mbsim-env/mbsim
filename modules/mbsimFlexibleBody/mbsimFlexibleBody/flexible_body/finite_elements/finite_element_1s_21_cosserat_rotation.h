/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: thomas.cebulla@mytum.de
 */

#ifndef _FINITE_ELEMENT_1S_21_COSSERAT_ROTATION_H_
#define _FINITE_ELEMENT_1S_21_COSSERAT_ROTATION_H_

#include "mbsimFlexibleBody/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "mbsimFlexibleBody/pointer.h"
#include "fmatvec/fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for planar beam using Cosserat model : rotation element for bending and torsion
   * \author Thomas Cebulla
   * \author Thorsten Schindler
   * \author Robert von Zitzewitz
   * \author Thomas Cebulla
   * \date 2012-12-14 initial commit (Thomas Cebulla)
   * \todo bending/torsion in rhs TODO
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
  class FiniteElement1s21CosseratRotation : public DiscretizationInterface {
    public:

      /**
       * \brief constructor
       * \param length of finite element
       * \param Young's modulus
       * \param shear modulus
       * \param first area moment of inertia
       * \param second area moment of inertia
       * \param torsional moment of inertia
       * \param cardan object
       */
      FiniteElement1s21CosseratRotation(double l0_,double E_,double G_,double I1_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s21CosseratRotation();		

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

      virtual fmatvec::Vec3 getPosition(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::SqrMat3 getOrientation(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::Vec3 getVelocity (const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Vec3 getAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Mat getJacobianOfMotion(const fmatvec::Vec& qElement, double s) { return computeJXqG(qElement,s); }

      /* GETTER / SETTER */
      void setCurlRadius(double R1);
      double getl0() const;

      /**
       * \brief compute JACOBIAN of contact description in global coordinates
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Mat computeJXqG(const fmatvec::Vec& qG,double x);

    private:
      /**
       * \brief length of finite element
       */
      double l0;

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
      double I1;

      /**
       * \brief predefined bending
       * k10: precurvature in t-b-plane
       * k20: precurvature in t-n-plane
       */
      double k10;

      /**
       * \brief global system description 
       */
      fmatvec::Vec h;

      /**
       * \brief state at Lagrangian coordinate
       */
      fmatvec::Vec X;

      /**
       * \brief matrices for implicit integration 
       */
      fmatvec::SqrMat dhdq, dhdu;


      FiniteElement1s21CosseratRotation(); // standard constructor
      FiniteElement1s21CosseratRotation(const FiniteElement1s21CosseratRotation&); // copy constructor
      FiniteElement1s21CosseratRotation& operator=(const FiniteElement1s21CosseratRotation&); // assignment operator
  };

  inline const fmatvec::SymMat& FiniteElement1s21CosseratRotation::getM() const { throw MBSim::MBSimError("(FiniteElement1s21CosseratRotation::getM): Not implemented");  }
  inline const fmatvec::Vec& FiniteElement1s21CosseratRotation::geth() const { return h; }
  inline const fmatvec::SqrMat& FiniteElement1s21CosseratRotation::getdhdq() const { return dhdq; }
  inline const fmatvec::SqrMat& FiniteElement1s21CosseratRotation::getdhdu() const { return dhdu; }
  inline int FiniteElement1s21CosseratRotation::getqSize() const { return 4; }
  inline int FiniteElement1s21CosseratRotation::getuSize() const { return 4; }
  inline void  FiniteElement1s21CosseratRotation::computeM(const fmatvec::Vec& qG) { throw MBSim::MBSimError("(FiniteElement1s21CosseratRotation::computeM): Not implemented"); }
  inline void  FiniteElement1s21CosseratRotation::computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("(FiniteElement1s21CosseratRotation::computedhdz): Not implemented"); }
  inline double FiniteElement1s21CosseratRotation::computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("(FiniteElement1s21CosseratRotation::computeKineticEnergy): Not implemented"); }
  inline double FiniteElement1s21CosseratRotation::computeGravitationalEnergy(const fmatvec::Vec& qG) { throw MBSim::MBSimError("(FiniteElement1s21CosseratRotation::computeGravitationalEnergy): Not implemented"); }
  inline double FiniteElement1s21CosseratRotation::getl0() const { return l0; }

}

#endif /* _FINITE_ELEMENT_1S_21_COSSERAT_ROTATION_H_ */
