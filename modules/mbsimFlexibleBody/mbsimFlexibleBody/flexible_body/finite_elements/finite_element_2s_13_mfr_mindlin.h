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

#ifndef _FINITE_ELEMENT_2S_13_MFR_MINDLIN_H_
#define _FINITE_ELEMENT_2S_13_MFR_MINDLIN_H_

#include "mbsimFlexibleBody/discretization_interface.h"
#include<cmath>

namespace MBSimFlexibleBody {

  /*! 
   * \brief FE for Reissner-Mindlin Plate using MFR
   * \author Kilian Grundl
   * \author Thorsten Schindler
   * \date 2009-12-23 initial commit (Grundl / Schindler)
   * \date 2010-04-23 check (Schindler)
   * \date 2010-08-18 check (Schindler)
   */
  class FiniteElement2s13MFRMindlin : public DiscretizationInterface {
    public:
      /** 
       * \brief constructor
       * \param Young's modulus
       * \param Poisson ratio
       * \param density
       * \param thickness parametrisation
       * \param radial and azimuthal coordinates of corner nodes
       */
      FiniteElement2s13MFRMindlin(double E_,double nu_,double rho_,double d0_,double d1_,double d2_,const fmatvec::Vec &NodeCoordinates);

      /**
       * \brief destructor
       */
      ~FiniteElement2s13MFRMindlin() override;    

      /* INTERFACE OF DISCRETIZATIONINTERFACE */
      const fmatvec::Vec& geth() const override;
      const fmatvec::SqrMat& getdhdq() const override;
      const fmatvec::SqrMat& getdhdu() const override;
      inline int getqSize() const override { return RefDofs + 4*NodeDofs; }
      inline int getuSize() const override { return RefDofs + 4*NodeDofs; }
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
      const fmatvec::SymMat& getM() const override; 
      const fmatvec::SymMat& getK() const { return *K; }
      const fmatvec::SymMat& getM_RR() const { return *M_RR; }
      const fmatvec::Mat& getN_compl() const { return *N_compl; }
      const fmatvec::SqrMat& getN_ij(int i, int j) const { return *(N_ij[i][j]); }
      const fmatvec::RowVec& getNR_ij(int i, int j) const { return *(NR_ij[i][j]); }
      const fmatvec::Vec& getR_compl() const { return *R_compl; }
      const fmatvec::SymMat& getR_ij() const { return *R_ij; }
      void setEModul(double E_) { E = E_; }
      void setPoissonRatio(double nu_) { nu = nu_; }
      void setDensity(double rho_) { rho = rho_; }
      void setShearCorrectionFactor(double alphaS_) { alphaS = alphaS_; }
      /***************************************************/

      /* Freeer */
      void freeK() { delete K; K=nullptr; }
      void freeM_RR() { delete M_RR; M_RR=nullptr; }
      void freeN_ij(int i, int j) { delete N_ij[i][j]; N_ij[i][j]=nullptr; }
      void freeN_compl() { delete N_compl; N_compl=nullptr; }
      void freeNR_ij(int i, int j) { delete NR_ij[i][j]; NR_ij[i][j]=nullptr; }
      void freeR_compl() { delete R_compl; R_compl=nullptr; }
      void freeR_ij() { delete R_ij; R_ij=nullptr; }
      /***************************************************/

      /*!
       * \brief computes stiffnes matrix
       */
      void computeStiffnessMatrix();

      /*!
       * \brief computes a part of the mass matrix
       */
      void computeM_RR();

      /*!
       * \brief computes a part of the mass matrix
       */
      void computeN_compl();

      /*!
       * \brief computes a part of the mass matrix
       * \param index of ansatz functions
       */
      void computeN_ij(int i, int j);
      void computeN_11();
      void computeN_12();
      void computeN_13();
      void computeN_21();
      void computeN_22();
      void computeN_23();
      void computeN_31();
      void computeN_32();
      void computeN_33();

      /*!
       * \brief computes a part of the mass matrix
       * \param index of ansatz functions
       */
      void computeNR_ij(int i, int j);
      void computeNR_11();
      void computeNR_12();
      void computeNR_13();
      void computeNR_21();
      void computeNR_22();
      void computeNR_23();
      void computeNR_31();
      void computeNR_32();
      void computeNR_33();

      /*!
       * \brief computes a part of the mass matrix
       */
      void computeR_compl();

      /*!
       * \brief computes a part of the mass matrix
       */
      void computeR_ij();

      /*!
       * \param radial and azimuthal coordinates of corner nodes
       * \param generalised coordinates
       * \param generalised velocities
       * \param Lagrangian position of contour point
       * \param inner thickness of whole disk
       * \param outer thickness of whole disk
       * \return state at contour point 
       */
      fmatvec::Vector<fmatvec::Fixed<6>, double> getPositions(const fmatvec::Vec &NodeCoordinates, const fmatvec::Vec &qElement, const fmatvec::Vec2 &s, double d1, double d2);

      fmatvec::Vector<fmatvec::Fixed<6>, double> getVelocities(const fmatvec::Vec &NodeCoordinates, const fmatvec::Vec &qElement, const fmatvec::Vec &qpElement, const fmatvec::Vec2 &s, double d1, double d2);

      /*! 
       * \brief compute Jacobian of contact description at contour point
       * \param radial and azimuthal coordinates of corner nodes
       * \param Lagrangian position of contour point
       */
      fmatvec::Mat JGeneralized(const fmatvec::Vec &NodeCoordinates,const fmatvec::Vec2 &s);

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
       * \brief geometric factors of the disk 
       */
      double d0, d1, d2;

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
       * \brief radial and azimuthal coordinates of corner nodes
       */
      fmatvec::Vec NodeCoordinates;

      /**
       * \brief stiffness matrix
       */
      fmatvec::SymMat *K;

      /**
       * \brief part of the mass matrix
       */
      fmatvec::SymMat *M_RR;

      /**
       * \brief part of the mass matrix
       */
      fmatvec::Mat *N_compl;

      /**
       * \brief part of the mass matrix
       */
      fmatvec::SqrMat *N_ij[3][3];

      /**
       * \brief part of the mass matrix
       */
      fmatvec::RowVec *NR_ij[3][3];

      /**
       * \brief part of the mass matrix
       */
      fmatvec::Vec *R_compl;

      /**
       * \brief part of the mass matrix
       */
      fmatvec::SymMat *R_ij;
  };

}

#endif /* _FINITE_ELEMENT_2S_13_MFR_MINDLIN_H_ */

