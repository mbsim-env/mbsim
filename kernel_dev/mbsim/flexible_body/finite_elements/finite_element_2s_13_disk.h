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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _FINITE_ELEMENT_2S_13_DISK_H_
#define _FINITE_ELEMENT_2S_13_DISK_H_

#include<cmath>
#include "mbsim/interfaces.h"
#include <mbsim/mbsim_event.h>

namespace MBSim {

  /*! 
   * \brief FE for Reissner-Mindlin Plate using MFR
   * \author Roland Zander
   * \author Thorsten Schindler
   * \author Kilian Grundl
   * \author Raphael Missel
   * \date 2009-05-22 initial commit (Grundl / Missel / Schindler)
   * \todo transform computeState to Position / Orientation / Velocity / AngularVelocity
   * \todo JacobianMinimalRepresentation
   * \todo energy
   * \todo right hand side
   * \todo equations of motion
   * \todo implicit integration
   */
  class FiniteElement2s13Disk : public DiscretizationInterface {
    public:
      /** 
       * \brief constructor
       * \param Young's modulus
       * \param Poiison ratio
       * \param density
       */
      FiniteElement2s13Disk(double E_,double nu_,double rho_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement2s13Disk() {}    

      /* INTERFACE OF DISCRETIZATIONINTERFACE */
      virtual const fmatvec::SymMat& getMassMatrix() const;    
      virtual const fmatvec::Vec& getGeneralizedForceVector() const;
      virtual const fmatvec::SqrMat& getJacobianForImplicitIntegrationRegardingPosition() const;
      virtual const fmatvec::SqrMat& getJacobianForImplicitIntegrationRegardingVelocity() const;
      virtual int getSizeOfPositions() const;
      virtual int getSizeOfVelocities() const;
      virtual void computeEquationsOfMotion(const fmatvec::Vec& q,const fmatvec::Vec& u);
      virtual double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& q);
      virtual double computeElasticEnergy(const fmatvec::Vec& q);
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMinimalRepresentationRegardingPhysics(const fmatvec::Vec& q,const ContourPointData &data);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::SymMat& getStiffnessMatrix() const;
      void setEModul(double E_);
      void setPoissonRatio(double nu_);
      void setDensity(double rho_);
      void setShearCorrectionFactor(double alp_);
      void setThickness(const fmatvec::Vec &di_,const fmatvec::Vec &da_);
      /***************************************************/

      /*!
       * \brief computes mass and stiffness matrix
       * \param radial and azimuthal coordinates of corner nodes
       * \param inner thickness of whole disk
       * \oaram outer thickness of whole disk
       */
      void computeConstantSystemMatrices(const fmatvec::Vec &NodeCoordinates,double d1,double d2);

      /*!
       * \param radial and azimuthal coordinates of corner nodes
       * \param generalised coordinates
       * \param generalised velocities
       * \param Lagrangian position of contour point
       * \param inner thickness of whole disk
       * \oaram outer thickness of whole disk
       * \return state at contour point 
       */
      fmatvec::Vec computeState(const fmatvec::Vec &NodeCoordinates,const fmatvec::Vec &qElement,const fmatvec::Vec &qpElement,const fmatvec::Vec &s,double d1,double d2);

      /*! 
       * \brief compute Jacobian of contact description at contour point
       * \param radial and azimuthal coordinates of corner nodes
       * \param Lagrangian position of contour point
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec &NodeCoordinates,const fmatvec::Vec &s);

    private:
      /**
       * \brief Young's modulus
       */
      double E;

      /**
       * \brief Poisson ratio
       */
      double nu;

      /**
       * \brief shear modulus
       */
      double G;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief shear correction factor
       */
      double alphaS;

      /**
       * \brief quadratic thickness coefficients for inner and outer direction from mid-plane
       */
      fmatvec::Vec di, da;

      /** 
       * \brief reference dof
       */ 
      int RefDofs;

      /**
       * \brief elastic dof per node
       */  
      int NodeDofs;

      /**
       * \brief number of nodes
       */
      int Nodes;

      /**
       * \brief mass and stiffness matrix
       */
      fmatvec::SymMat M, K;
  };

  inline const fmatvec::SymMat& FiniteElement2s13Disk::getMassMatrix() const { return M; }
  inline const fmatvec::Vec& FiniteElement2s13Disk::getGeneralizedForceVector() const { throw new MBSimError("ERROR(FiniteElement2s13Disk::getGeneralizedForceVector): Not implemented!"); } 
  inline const fmatvec::SqrMat& FiniteElement2s13Disk::getJacobianForImplicitIntegrationRegardingPosition() const { throw new MBSimError("ERROR(FiniteElement2s13Disk::getJacobianForImplicitIntegrationRegardingPosition): Not implemented!"); } 
  inline const fmatvec::SqrMat& FiniteElement2s13Disk::getJacobianForImplicitIntegrationRegardingVelocity() const { throw new MBSimError("ERROR(FiniteElement2s13Disk::getJacobianForImplicitIntegrationRegardingVelocity): Not implemented!"); } 
  inline int FiniteElement2s13Disk::getSizeOfPositions() const { return RefDofs + 4*NodeDofs; }
  inline int FiniteElement2s13Disk::getSizeOfVelocities() const { return RefDofs + 4*NodeDofs; }
  inline void FiniteElement2s13Disk::computeEquationsOfMotion(const fmatvec::Vec& q,const fmatvec::Vec& u) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeEquationsOfMotion): Not implemented!"); } 
  inline double FiniteElement2s13Disk::computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeKineticEnergy): Not implemented!"); } 
  inline double FiniteElement2s13Disk::computeGravitationalEnergy(const fmatvec::Vec& q) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeGravitationalEnergy): Not implemented!"); } 
  inline double FiniteElement2s13Disk::computeElasticEnergy(const fmatvec::Vec& q) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeElasticEnergy): Not implemented!"); } 
  inline fmatvec::Vec FiniteElement2s13Disk::computePosition(const fmatvec::Vec& q, const ContourPointData &data) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computePosition): Not implemented!"); } 
  inline fmatvec::SqrMat FiniteElement2s13Disk::computeOrientation(const fmatvec::Vec& q, const ContourPointData &data) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeOrientation): Not implemented!"); } 
  inline fmatvec::Vec FiniteElement2s13Disk::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeVelocity): Not implemented!"); } 
  inline fmatvec::Vec FiniteElement2s13Disk::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeAngularVelocity): Not implemented!"); } 
  inline fmatvec::Mat FiniteElement2s13Disk::computeJacobianOfMinimalRepresentationRegardingPhysics(const fmatvec::Vec& qG,const ContourPointData& data) { throw new MBSimError("ERROR(FiniteElement2s13Disk::computeJacobianOfMinimalRepresentationRegardingPhysics): Not implemented!"); } 

  inline const fmatvec::SymMat& FiniteElement2s13Disk::getStiffnessMatrix() const { return K; }
  inline void FiniteElement2s13Disk::setEModul(double E_) { E = E_; }
  inline void FiniteElement2s13Disk::setPoissonRatio(double nu_) { nu = nu_; }
  inline void FiniteElement2s13Disk::setDensity(double rho_) { rho = rho_; }
  inline void FiniteElement2s13Disk::setShearCorrectionFactor(double alphaS_) { alphaS = alphaS_; }
  inline void FiniteElement2s13Disk::setThickness(const fmatvec::Vec &di_,const fmatvec::Vec &da_) { di = di_; da=da_; }
}

#endif /* _FINITE_ELEMENT_2S_13_RCM_H_ */

