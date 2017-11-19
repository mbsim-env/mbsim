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

#ifndef _FINITE_ELEMENT_2S_13_DISK_H_
#define _FINITE_ELEMENT_2S_13_DISK_H_

#include "mbsimFlexibleBody/discretization_interface.h"
#include <cmath>

namespace MBSimFlexibleBody {

  /*! 
   * \brief FE for Reissner-Mindlin Plate using MFR
   * \author Roland Zander
   * \author Thorsten Schindler
   * \author Kilian Grundl
   * \author Raphael Missel
   * \date 2009-05-22 initial commit (Grundl / Missel / Schindler)
   * \date 2009-07-24 implicit integration (Thorsten Schindler)
   * \date 2010-05-25 fixed minus sign in azimuthal stiffness directions (Thorsten Schindler)
   */
  class FiniteElement2s13Disk : public DiscretizationInterface {
    public:
      /** 
       * \brief constructor
       * \param Young's modulus
       * \param Poiison ratio
       * \param density
       */
      FiniteElement2s13Disk(double E_, double nu_, double rho_);

      /**
       * \brief destructor
       */
      ~FiniteElement2s13Disk() override = default;

      /* INTERFACE OF DISCRETIZATIONINTERFACE */
      const fmatvec::SymMat& getM() const override { return M; }
      const fmatvec::Vec& geth() const override;
      const fmatvec::SqrMat& getdhdq() const override;
      const fmatvec::SqrMat& getdhdu() const override;
      int getqSize() const override { return RefDofs + 4*NodeDofs; }
      int getuSize() const override { return RefDofs + 4*NodeDofs; }
      void computeM(const fmatvec::Vec& q) override;
      void computeh(const fmatvec::Vec& q,const fmatvec::Vec& u) override;
      void computedhdz(const fmatvec::Vec& q,const fmatvec::Vec& u) override;
      double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) override;
      double computeGravitationalEnergy(const fmatvec::Vec& q) override;
      double computeElasticEnergy(const fmatvec::Vec& q) override;
      virtual fmatvec::Vec3 getPosition(const fmatvec::Vec& qElement, const fmatvec::Vec2 &s);
      virtual fmatvec::SqrMat3 getOrientation(const fmatvec::Vec& qElement, const fmatvec::Vec2 &s);
      virtual fmatvec::Vec3 getVelocity (const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const fmatvec::Vec2 &s);
      virtual fmatvec::Vec3 getAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, const fmatvec::Vec2 &s);
      virtual fmatvec::Mat getJacobianOfMotion(const fmatvec::Vec& qElement, const fmatvec::Vec2 &s);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::SymMat& getK() const { return K; }
      void setEModul(double E_) { E = E_; }
      void setPoissonRatio(double nu_) { nu = nu_; }
      void setDensity(double rho_) { rho = rho_; }
      void setShearCorrectionFactor(double alphaS_) { alphaS = alphaS_; }
      /***************************************************/

      /*!
       * \brief computes mass and stiffness matrix
       * \param radial and azimuthal coordinates of corner nodes
       * \param inner thickness of whole disk
       * \oaram outer thickness of whole disk
       */
      void computeConstantSystemMatrices(const fmatvec::Vec &NodeCoordinates, double d1, double d2);

      /*!
       * \param radial and azimuthal coordinates of corner nodes
       * \param generalised coordinates
       * \param generalised velocities
       * \param Lagrangian position of contour point
       * \param inner thickness of whole disk
       * \oaram outer thickness of whole disk
       * \return state at contour point 
       */
      fmatvec::Vector<fmatvec::Fixed<6>, double> getPositions(const fmatvec::Vec &NodeCoordinates, const fmatvec::Vec &qElement, const fmatvec::Vec2 &s, double d1, double d2);

      fmatvec::Vector<fmatvec::Fixed<6>, double> getVelocities(const fmatvec::Vec &NodeCoordinates, const fmatvec::Vec &qElement, const fmatvec::Vec &qpElement, const fmatvec::Vec2 &s, double d1, double d2);

      /*! 
       * \brief compute Jacobian of contact description at contour point
       * \param radial and azimuthal coordinates of corner nodes
       * \param Lagrangian position of contour point
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec &NodeCoordinates, const fmatvec::Vec2 &s);

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

}

#endif /* _FINITE_ELEMENT_2S_13_RCM_H_ */
