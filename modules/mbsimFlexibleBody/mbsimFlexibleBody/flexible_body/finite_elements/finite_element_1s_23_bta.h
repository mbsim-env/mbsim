/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef _FINITE_ELEMENT_1S_23_BTA_H_
#define _FINITE_ELEMENT_1S_23_BTA_H_

#include "mbsimFlexibleBody/discretization_interface.h"
#include "fmatvec/fmatvec.h"

namespace MBSimFlexibleBody {

  /*! 
   * \brief finite element for bending torsional axis
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-11-22 initial commit kernel_dev
   * \todo implicit integration analytically TODO
   * \todo tangent, awk necessary? TODO
   */
  class FiniteElement1s23BTA : public DiscretizationInterface {
    public:
      /**
       * \brief constructor
       * \param length of finite element
       * \param cross-sectional area multiplied with density
       * \param Young's modulus multiplied with first area moment of inertia
       * \param Young's modulus multiplied with second area moment of inertia
       * \param torsional moment of inertia multiplied with density
       * \param shear modulus multiplied with torsional moment of inertia
       * \param acceleration of gravity
       */
      FiniteElement1s23BTA(double l0, double Arho, double EIyy, double EIzz, double Itrho, double GIt, fmatvec::Vec g);

      /**
       * \brief destructor
       */
      ~FiniteElement1s23BTA() override = default;

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */
      const fmatvec::SymMat& getM() const override { return M; }
      const fmatvec::Vec& geth() const override { return h; }
      const fmatvec::SqrMat& getdhdq() const override { return Dhq; }
      const fmatvec::SqrMat& getdhdu() const override { return Dhqp; }
      int getqSize() const override { return 10; }
      int getuSize() const override { return 10; }

      void computeM(const fmatvec::Vec& qG) override;
      void computeh(const fmatvec::Vec& qG,const fmatvec::Vec& qGt) override;
      void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) override;
      double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) override;
      double computeGravitationalEnergy(const fmatvec::Vec& q) override;    
      double computeElasticEnergy(const fmatvec::Vec& q) override;

      virtual fmatvec::Vec3 getPosition(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::SqrMat3 getOrientation(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::Vec3 getVelocity (const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Vec3 getAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Mat getJacobianOfMotion(const fmatvec::Vec& qElement, double s) { return JGeneralized(qElement,s); }
      /*****************************************************/ 

      /* GETTER / SETTER */
      void setMaterialDamping(double);
      void setLehrDamping(double);
      void setTorsionalDamping(double dTorsional_) { dTorsional=dTorsional_; }
      /*****************************************************/ 

      /*! 
       * \brief compute tangent
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Vec3 getTangent(const fmatvec::Vec& q, double s);

      fmatvec::Vector<fmatvec::Fixed<6>, double> getPositions(const fmatvec::Vec& qElement, double s);

      fmatvec::Vector<fmatvec::Fixed<6>, double> getVelocities(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);

//      /*! 
//       * \brief compute accompanying trihedral
//       * \param global coordinates
//       * \param LAGRANGIAN parameter
//       */
//      fmatvec::SqrMat3 getOrientation(const fmatvec::Vec& q, double s);

//      /*!
//       * \brief compute global state 
//       * \param global coordinates
//       * \param global velocities
//       * \param LAGRANGIAN parameter
//       * \return x,y,z,alpha,2x bending angle and velocities
//       */
//      fmatvec::Vec StateAxis(const fmatvec::Vec& q, const fmatvec::Vec& v, double s); 

      /**
       * \brief compute JACOBIAN of contact description
       * \param global coordinates
       * \param LAGRANGIAN parameter
       * \return JACOBIAN without x-direction
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec& q, double s);

    private:
      /**
       * \brief length of finite element
       */
      double l0;

      /**
       * \brief cross sectional area multiplied with density
       */
      double Arho;

      /**
       * \brief Young's modulus multiplied with geometrical moment of inertia
       */
      double EIyy, EIzz;

      /**
       * \brief geometrical moment of inertia multiplied with density
       */
      double Itrho;

      /**
       * \brief shear modulus multiplied with geometrical moment of inertia
       */
      double GIt;

      /**
       * \brief gravitation
       */
      fmatvec::Vec g;

      /**
       * \brief prolongational and torsional damping
       */
      double depsilon, dTorsional;

      /**
       * \brief global system description 
       */
      fmatvec::SymMat M;
      fmatvec::Vec h;

      /**
       * \brief matrices for implicit integration 
       */
      fmatvec::SqrMat Dhq, Dhqp;

      /**
       * \brief internal damping matrix
       */
      fmatvec::SqrMat Damp;

      /**
       * \brief beam length powers 
       */
      double l0h2, l0h3;
  };

  inline double Sec(double alpha) { return 1.0/cos(alpha); }
  inline double Power(double base, int exponent) { return pow(base,exponent); }
}

#endif /* _FINITE_ELEMENT_1S_23_BTA_H_ */
