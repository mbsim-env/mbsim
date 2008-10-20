/* Copyright (C) 2005-2008  Roland Zander, Thorsten Schindler

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
 * Contact:
 *   rzander@users.berlios.de
 *
 */

#ifndef _FINITE_ELEMENT_1S_21_RCM_H_
#define _FINITE_ELEMENT_1S_21_RCM_H_

#include "discretization_interface.h"

namespace MBSim {

  /*! \brief Finite Element for bending torsional axis */
  class FiniteElement1s23BTA : public DiscretizationInterface {
    public:
      /* INTERFACE */
      FiniteElement1s23BTA(double l0, double Arho, double EIyy, double EIzz, double Itrho, double GIt, fmatvec::Vec g, int warnLevel_=0);
      virtual ~FiniteElement1s23BTA() {}

      fmatvec::SymMat getMassMatrix() const;    
      fmatvec::Vec getGeneralizedForceVector() const;
      fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingPosition() const;    
      fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingVelocity() const;
      int getSizeOfPositions() const;
      int getSizeOfVelocities() const;

      void computeEquationsOfMotion(const fmatvec::Vec& q, const fmatvec::Vec& v);
      double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u);
      double computeGravitationalEnergy(const fmatvec::Vec& q);    
      double computeElasticEnergy(const fmatvec::Vec& q);

      fmatvec::Vec computeTranslation(const fmatvec::Vec& q, const ContourPointData &data);
      fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const ContourPointData& data);
      fmatvec::Vec computeTranslationalVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data);
      fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data);
      fmatvec::Mat computeJacobianOfMinimalRepresentationRegardingPhysics(const fmatvec::Vec& q,const ContourPointData &data);
      /* INTERFACE */

      /*! set material damping */
      void setMaterialDamping(double) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA:setMaterialDamping): Not implemented!" << endl;}
      /*! set Lehr daming */
      void setLehrDamping(double) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA:setLehrDamping): Not implemented!" << endl;}
      /*! set torsional damping */ 
      void setTorsionalDamping(double dTorsional_) {dTorsional=dTorsional_;}
      /*! set flag for implicit integration */
      void setImplicitIntegration(bool implicit_) {implicit=implicit_;}

      /*! compute tangent */
      fmatvec::Vec Tangent(const fmatvec::Vec& q, const double& s);

      /*! compute accompanying trihedral */
      fmatvec::SqrMat AWK(const fmatvec::Vec& q, const double& s);

      /*! compute global state */
      fmatvec::Vec StateAxis(const fmatvec::Vec& q, const fmatvec::Vec& v, const double& s); 

      /*! compute Jacobian mininmal <- physical representation */
      fmatvec::Mat JGeneralized(const fmatvec::Vec& q, const double& s);

      /*! element data for output */
      fmatvec::Vec ElementData(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement);

    private:
      int warnLevel;
      double l0h2, l0h3;
      double l0, Arho, EIyy, EIzz, Itrho, GIt;
      fmatvec::Vec g;
      double dTorsional;
      double depsilon;

      bool implicit;

      fmatvec::SymMat M;
      fmatvec::Vec h;

      fmatvec::SqrMat Dhq, Dhqp;

      fmatvec::SqrMat Damp;
  };

  inline SymMat FiniteElement1s23BTA::getMassMatrix() const {return M;}    
  inline Vec FiniteElement1s23BTA::getGeneralizedForceVector() const {return h;}
  inline SqrMat FiniteElement1s23BTA::getJacobianForImplicitIntegrationRegardingPosition() const {return Dhq;}    
  inline SqrMat FiniteElement1s23BTA::getJacobianForImplicitIntegrationRegardingVelocity() const {return Dhqp;}
  inline int FiniteElement1s23BTA::getSizeOfPositions() const {return 10;}
  inline int FiniteElement1s23BTA::getSizeOfVelocities() const {return 10;}
  inline fmatvec::Vec FiniteElement1s23BTA::computeTranslation(const fmatvec::Vec& q, const ContourPointData &data) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA::computeTranslation): Not implemented!" << endl; return Vec(0,INIT,0.);}
  inline fmatvec::SqrMat FiniteElement1s23BTA::computeOrientation(const fmatvec::Vec& q, const ContourPointData &data) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA::computeOrientation): Not implemented!" << endl; return SqrMat(0,INIT,0.);}
  inline fmatvec::Vec FiniteElement1s23BTA::computeTranslationalVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA::computeTranslationalVelocity): Not implemented!" << endl; return Vec(0,INIT,0.);}
  inline fmatvec::Vec FiniteElement1s23BTA::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA::computeAngularVelocity): Not implemented!" << endl; return Vec(0,INIT,0.);}
}

#endif
