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

#ifndef _FINITE_ELEMENT_1S_23_BTA_H_
#define _FINITE_ELEMENT_1S_23_BTA_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/mbsim_event.h"
#include "fmatvec.h"

namespace MBSimFlexibleBody {

  /*! 
   * \brief finite element for bending torsional axis
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-11-22 initial commit kernel_dev
   * \todo implicit integration analytically TODO
   * \todo tangent, awk necessary? TODO
   */
  class FiniteElement1s23BTA : public MBSim::DiscretizationInterface {
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
      virtual ~FiniteElement1s23BTA() {}

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */
      virtual const fmatvec::SymMat& getM() const;    
      virtual const fmatvec::Vec& geth() const;
      virtual const fmatvec::SqrMat& getdhdq() const;    
      virtual const fmatvec::SqrMat& getdhdu() const;
      virtual int getqSize() const;
      virtual int getuSize() const;

      virtual void computeM(const fmatvec::Vec& qG);
      virtual void computeh(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      virtual void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& q);    
      virtual double computeElasticEnergy(const fmatvec::Vec& q);

      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData& data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& q,const MBSim::ContourPointData &data);
      /*****************************************************/ 

      /* GETTER / SETTER */
      void setMaterialDamping(double) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA:setMaterialDamping): Not implemented!"); }
      void setLehrDamping(double) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA:setLehrDamping): Not implemented!"); }
      void setTorsionalDamping(double dTorsional_) { dTorsional=dTorsional_; }
      /*****************************************************/ 

      /*! 
       * \brief compute tangent
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Vec Tangent(const fmatvec::Vec& q, const double& s);

      /*! 
       * \brief compute accompanying trihedral
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::SqrMat AWK(const fmatvec::Vec& q, const double& s);

      /*!
       * \brief compute global state 
       * \param global coordinates
       * \param global velocities
       * \param LAGRANGIAN parameter
       * \return x,y,z,alpha,2x bending angle and velocities
       */
      fmatvec::Vec StateAxis(const fmatvec::Vec& q, const fmatvec::Vec& v, const double& s); 

      /**
       * \brief compute JACOBIAN of contact description
       * \param global coordinates
       * \param LAGRANGIAN parameter
       * \return JACOBIAN without x-direction
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec& q, const double& s);

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

  inline const fmatvec::SymMat& FiniteElement1s23BTA::getM() const { return M; }    
  inline const fmatvec::Vec& FiniteElement1s23BTA::geth() const { return h; }
  inline const fmatvec::SqrMat& FiniteElement1s23BTA::getdhdq() const { return Dhq; }    
  inline const fmatvec::SqrMat& FiniteElement1s23BTA::getdhdu() const { return Dhqp; }
  inline int FiniteElement1s23BTA::getqSize() const { return 10; }
  inline int FiniteElement1s23BTA::getuSize() const { return 10; }
  inline double FiniteElement1s23BTA::computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA:computeKineticEnergy): Not implemented!"); }
  inline double FiniteElement1s23BTA::computeGravitationalEnergy(const fmatvec::Vec& q) { throw new MBSim::MBSimError("ERROR WARNING (FiniteElement1s23BTA:computeGravitationalEnergy): Not implemented!"); }     
  inline double FiniteElement1s23BTA::computeElasticEnergy(const fmatvec::Vec& q) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA:computeElasticEnergy): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s23BTA::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) {throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat FiniteElement1s23BTA::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s23BTA::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA::computeVelocity): Not implemented!");}
  inline fmatvec::Vec FiniteElement1s23BTA::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR (FiniteElement1s23BTA::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElement1s23BTA::computeJacobianOfMotion(const fmatvec::Vec& q,const MBSim::ContourPointData &data) { return JGeneralized(q,data.getLagrangeParameterPosition()(0)); }

  inline double Sec(const double& alpha) { return 1.0/cos(alpha); }
  inline double Power(double base, int exponent) { return pow(base,exponent); }
}

#endif /* _FINITE_ELEMENT_1S_23_BTA_H_ */

