/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef _FINITE_ELEMENT_2S_13_MFR_MINDLIN_H_
#define _FINITE_ELEMENT_2S_13_MFR_MINDLIN_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"

#include<cmath>

namespace MBSimFlexibleBody {

  /*! 
   * \brief FE for Reissner-Mindlin Plate using MFR
   * \author Kilian Grundl
   * \author Thorsten Schindler
   * \date 2009-12-23 initial commit (Grundl / Schindler)
   * \date 2010-04-23 check (Schindler)
   * \todo transform computeState to Position / Orientation / Velocity / AngularVelocity
   * \todo JacobianMinimalRepresentation
   * \todo energy
   * \todo right hand side
   * \todo equations of motion
   * \todo implicit integration
   */
  class FiniteElement2s13MFRMindlin : public MBSim::DiscretizationInterface {
    public:
      /** 
       * \brief constructor
       * \param Young's modulus
       * \param Poisson ratio
       * \param density
       * \param thickness parametrisation
       */
      FiniteElement2s13MFRMindlin(double E_,double nu_,double rho_,double d0_,double d1_,double d2_, const fmatvec::Vec &NodeCoordinates);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement2s13MFRMindlin() {}    

      /* INTERFACE OF DISCRETIZATIONINTERFACE */
      virtual const fmatvec::Vec& geth() const;
      virtual const fmatvec::SqrMat& getdhdq() const;
      virtual const fmatvec::SqrMat& getdhdu() const;
      virtual int getqSize() const;
      virtual int getuSize() const;
      virtual void computeh(const fmatvec::Vec& q,const fmatvec::Vec& u);
      virtual void computedhdz(const fmatvec::Vec& q,const fmatvec::Vec& u);
      virtual double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& q);
      virtual double computeElasticEnergy(const fmatvec::Vec& q);
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& q,const MBSim::ContourPointData &data);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::SymMat& getK() const;
      const fmatvec::SymMat& getM() const; 
      const fmatvec::SymMat& getM_RR() const;
      const fmatvec::Mat& getN_compl() const;
      const fmatvec::SqrMat& getN_ij(int i, int j) const; 
      const fmatvec::RowVec& getNR_ij(int i, int j) const;
      const fmatvec::Vec& getR_compl() const;
      const fmatvec::SymMat& getR_ij() const; 
      void setEModul(double E_);
      void setPoissonRatio(double nu_);
      void setDensity(double rho_);
      void setShearCorrectionFactor(double alphaS_);
      /***************************************************/

      /* Freeer */
      void freeK();
      void freeM_RR();
      void freeN_compl();
      void freeN_ij(int i, int j);
      void freeNR_ij(int i, int j);
      void freeR_compl();
      void freeR_ij();
      /***************************************************/


      /*!
       * \brief computes stiffnes matrix
       * \param radial and azimuthal coordinates of corner nodes
       */
      void computeStiffnesMatrix(const fmatvec::Vec &NodeCoordinates);
      void computeStiffnesMatrix();

      /*!
       * \brief computes mass matrix
       * \param radial and azimuthal coordinates of corner nodes
       */
      void computeM(const fmatvec::Vec &NodeCoordinates);    

      /*!
       * \brief computes a part of the mass matrix
       * \param radial and azimuthal coordinates of corner nodes
       */
      void computeM_RR();

      /*!
       * \brief computes a part of the mass matrix
       * \param radial and azimuthal coordinates of corner nodes
       */
      void computeN_compl();

      /*!
       * \brief computes a part of the mass matrix
       * \param radial and azimuthal coordinates of corner nodes
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
       * \param radial and azimuthal coordinates of corner nodes
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
       * \param radial and azimuthal coordinates of corner nodes
       */
      void computeR_compl();

      /*!
       * \brief computes a part of the mass matrix
       * \param radial and azimuthal coordinates of corner nodes
       */
      void computeR_ij();



      //TODO:
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
       * \brief number of nodes
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

  inline const fmatvec::SymMat& FiniteElement2s13MFRMindlin::getM() const { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::getM): Not implemented!"); }
  inline void FiniteElement2s13MFRMindlin::computeM(const fmatvec::Vec& NodeCoordinates){ throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeM): Not implemented!"); }
  inline void FiniteElement2s13MFRMindlin::computeStiffnesMatrix(const fmatvec::Vec& NodeCoordinates){ throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeStiffnessMatrix): Not implemented!"); }
  inline const fmatvec::Vec& FiniteElement2s13MFRMindlin::geth() const { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::geth): Not implemented!"); } 
  inline const fmatvec::SqrMat& FiniteElement2s13MFRMindlin::getdhdq() const { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::getdhdq): Not implemented!"); } 
  inline const fmatvec::SqrMat& FiniteElement2s13MFRMindlin::getdhdu() const { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::getdhdu): Not implemented!"); } 
  inline int FiniteElement2s13MFRMindlin::getqSize() const { return RefDofs + 4*NodeDofs; }
  inline int FiniteElement2s13MFRMindlin::getuSize() const { return RefDofs + 4*NodeDofs; }
  inline void FiniteElement2s13MFRMindlin::computeh(const fmatvec::Vec& q,const fmatvec::Vec& u) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeh): Not implemented!"); } 
  inline void FiniteElement2s13MFRMindlin::computedhdz(const fmatvec::Vec& q,const fmatvec::Vec& u) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computedhdz): Not implemented!"); } 
  inline double FiniteElement2s13MFRMindlin::computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeKineticEnergy): Not implemented!"); } 
  inline double FiniteElement2s13MFRMindlin::computeGravitationalEnergy(const fmatvec::Vec& q) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeGravitationalEnergy): Not implemented!"); } 
  inline double FiniteElement2s13MFRMindlin::computeElasticEnergy(const fmatvec::Vec& q) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeElasticEnergy): Not implemented!"); } 
  inline fmatvec::Vec FiniteElement2s13MFRMindlin::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computePosition): Not implemented!"); } 
  inline fmatvec::SqrMat FiniteElement2s13MFRMindlin::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeOrientation): Not implemented!"); } 
  inline fmatvec::Vec FiniteElement2s13MFRMindlin::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeVelocity): Not implemented!"); } 
  inline fmatvec::Vec FiniteElement2s13MFRMindlin::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeAngularVelocity): Not implemented!"); } 
  inline fmatvec::Mat FiniteElement2s13MFRMindlin::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { throw new MBSim::MBSimError("ERROR(FiniteElement2s13MFRMindlin::computeJacobianOfMotion): Not implemented!"); } 

  inline const fmatvec::SymMat& FiniteElement2s13MFRMindlin::getK() const { return *K; }
  inline const fmatvec::SymMat& FiniteElement2s13MFRMindlin::getM_RR() const { return *M_RR; }
  inline const fmatvec::Mat& FiniteElement2s13MFRMindlin::getN_compl() const { return *N_compl; }
  inline const fmatvec::SqrMat& FiniteElement2s13MFRMindlin::getN_ij(int i, int j) const { return *(N_ij[i][j]); }
  inline const fmatvec::RowVec& FiniteElement2s13MFRMindlin::getNR_ij(int i, int j) const {return *(NR_ij[i][j]); }
  inline const fmatvec::Vec& FiniteElement2s13MFRMindlin::getR_compl() const { return *R_compl; }
  inline const fmatvec::SymMat& FiniteElement2s13MFRMindlin::getR_ij() const { return *R_ij; }
  inline void FiniteElement2s13MFRMindlin::setEModul(double E_) { E = E_; }
  inline void FiniteElement2s13MFRMindlin::setPoissonRatio(double nu_) { nu = nu_; }
  inline void FiniteElement2s13MFRMindlin::setDensity(double rho_) { rho = rho_; }
  inline void FiniteElement2s13MFRMindlin::setShearCorrectionFactor(double alphaS_) { alphaS = alphaS_; }
}

#endif /* _FINITE_ELEMENT_2S_13_MFR_MINDLIN_H_ */

