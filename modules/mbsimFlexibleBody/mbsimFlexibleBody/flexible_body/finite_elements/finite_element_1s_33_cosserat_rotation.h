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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef _FINITE_ELEMENT_1S_33_COSSERAT_ROTATION_H_
#define _FINITE_ELEMENT_1S_33_COSSERAT_ROTATION_H_

#include "mbsimFlexibleBody/discretization_interface.h"
#include "mbsimFlexibleBody/pointer.h"
#include "fmatvec/fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for spatial beam using Cosserat model : rotation element for bending and torsion
   * \author Thorsten Schindler
   * \author Christian Käsbauer
   * \author Thomas Cebulla
   * \date 2011-10-11 initial commit (Thorsten Schindler)
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
  class FiniteElement1s33CosseratRotation : public DiscretizationInterface {
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
      FiniteElement1s33CosseratRotation(double l0_,double E_,double G_,double I1_,double I2_,double I0_,CardanPtr ag_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s33CosseratRotation();		

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */ 
      virtual const fmatvec::SymMat& getM() const;		
      virtual const fmatvec::Vec& geth() const { return h; }
      virtual const fmatvec::SqrMat& getdhdq() const { return dhdq; }
      virtual const fmatvec::SqrMat& getdhdu() const { return dhdu; }
      virtual int getqSize() const { return 9; }
      virtual int getuSize() const { return 9; }

      virtual void computeM(const fmatvec::Vec& qG);
      virtual void computeh(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qG);
      virtual double computeElasticEnergy(const fmatvec::Vec& qG);

      virtual fmatvec::Vec3 getPosition(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::SqrMat3 getOrientation(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::Vec3 getVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Vec3 getAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG, double s) { return computeJXqG(qG,s); }

      /* GETTER / SETTER */
      void setCurlRadius(double R1,double R2);		
      double getl0() const { return l0; }

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
      double I1, I2, I0;

      /**
       * \brief predefined bending
       * k10: precurvature in t-b-plane
       * k20: precurvature in t-n-plane
       */
      double k10, k20;

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

      /**
       * \brief Cardan-object 
       */
      CardanPtr ag;

      FiniteElement1s33CosseratRotation(); // standard constructor
      FiniteElement1s33CosseratRotation(const FiniteElement1s33CosseratRotation&); // copy constructor
      FiniteElement1s33CosseratRotation& operator=(const FiniteElement1s33CosseratRotation&); // assignment operator
  };

}

#endif /* _FINITE_ELEMENT_1S_33_COSSERAT_ROTATION_H_ */
