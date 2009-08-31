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

#ifndef _FLEXIBLE_BODY_2S_13_DISK_H_
#define _FLEXIBLE_BODY_2S_13_DISK_H_

#include "mbsim/flexible_body.h"
#include "mbsim/flexible_body/finite_elements/finite_element_2s_13_disk.h"

namespace MBSim {

  class NurbsDisk2s;

  /**
   * \brief condensation setting for clamping to rigid body motion
   */
  enum LockType { innerring,outerring };

  /*!
   * \brief condenses rows of matrix concerning index
   * \param input matrix
   * \param indices to be condensed
   * \return condensed matrix
   */
  fmatvec::Mat condenseMatrixRows(fmatvec::Mat A, fmatvec::Index I);

  /*! 
   * \brief condenses symmetric matrix concerning index
   * \param input matrix
   * \param indices to be condensed
   * \return condensed matrix
   */
  fmatvec::SymMat condenseMatrix(fmatvec::SymMat A,fmatvec::Index I);

  /*! 
   * \brief calculates planar angle in [0,2\pi] with respect to cartesian coordinates 
   * \param Cartesian x-coordinate
   * \param Cartesian y-coordinate
   * \return angle
   */
  double ArcTan(double x,double y);

  /*! 
   * \brief plate according to Reissner-Mindlin with moving frame of reference
   * \author Roland Zander
   * \author Thorsten Schindler
   * \author Kilian Grundl
   * \author Raphael Missel
   * \author Adrian Staszkiewicz
   * \date 2009-05-14 initial commit (Schindler / Grundl / Missel)
   * \date 2009-08-15 contour / visualisation (Schindler / Grundl / Missel)
   * \todo gravity TODO
   */
  class FlexibleBody2s13Disk : public MBSim::FlexibleBodyContinuum<fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody2s13Disk(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody2s13Disk() {}

      /* INHERITED INTERFACE OF FLEXIBLE BODY CONTINUUM */
      using FlexibleBodyContinuum<fmatvec::Vec>::addFrame;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t);
      virtual void updateM(double t);
      virtual void updatedhdz(double t);
      virtual void updateStateDependentVariables(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame=0);
      virtual void updateJacobiansForFrame(ContourPointData &data, Frame *frame=0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      virtual void facLLM() {};
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody2s13Disk"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double Ri_,double Ra_) { Ri = Ri_; Ra = Ra_; }		
      void setEModul(double E_) { E = E_; }				
      void setPoissonRatio(double nu_) { nu = nu_; }			
      void setThickness(const fmatvec::Vec &d_) { d = d_; }		    
      void setDensity(double rho_) { rho = rho_; }	
      int getRadialNumberOfElements() const { return nr; }
      int getAzimuthalNumberOfElements() const { return nj; }
      double getInnerRadius() const { return Ri; }
      double getOuterRadius() const { return Ra; }
      void setReferenceInertia(double m0_, double J0_) { m0 = m0_; J0 = J0_; }	
      void setLockType(LockType LT_) { LType = LT_; }
      /***************************************************/

      /*! 
       * \brief set number of elements in radial and azimuthal direction
       * \param radial number of elements
       * \param azimuthal number of elements
       */
      void setNumberElements(int nr_,int nj_);

      /*!
       * \return potential energy
       */
      double computePotentialEnergy() { return 0.5*trans(q)*K*q; }

      /*! 
       * \brief transformation cartesian to cylinder system
       * \param cartesian vector in world system
       * \return cylindrical coordinates
       */
      fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint);

    private:
      /** 
       * \brief total number of elements
       */
      int Elements;
      
      /** 
       * \brief elastic dof per node
       */
      int NodeDofs;
      
      /**
       * \brief dof of moving frame of reference
       */
      int RefDofs;

      /**
       * \brief Young's modulus
       */
      double E;
      
      /**
       * \brief Poisson ratio
       */
      double nu;
      
      /** 
       * \brief density
       */
      double rho;
      
      /**
       * \brief inner and outer thickness
       */
      fmatvec::Vec d;
      
      /**
       * \brief inner and outer radius of disk
       */
      double Ri, Ra;
      
      /**
       * \brief radial and azimuthal length of an FE
       */
      double dr, dj;
      
      /**
       * \brief mass and inertia of the attached shaft
       */
      double m0, J0;
      
      /**
       * \brief degree of surface interpolation in radial and azimuthal direction
       *
       * both degrees have to be smaller than 8
       */
      int degV, degU;

      /**
       * \brief number of points drawn between nodes
       */
      int drawDegree;

      /**
       * \brief vector of boundary data of the FE (r1,j1,r2,j2)
       */
      std::vector<fmatvec::Vec> ElementalNodes;
      
      /**
       * \brief number of element currently involved in contact calculations
       */
      int currentElement;
      
      /**
       * \brief mass matrix
       */
      fmatvec::SymMat MSave; 

      /**
       * \brief stiffness matrix
       */
      fmatvec::SymMat K; 

      /**
       * \brief number of elements in radial and azimuthal direction, number of FE nodes
       */
      int nr, nj, Nodes;
      
      /**
       * \brief matrix mapping nodes and coordinates (size number of nodes x number of node coordinates)
       */
      fmatvec::Mat NodeCoordinates; 
      
      /** 
       * \brief matrix mapping elements and nodes (size number of elements x number of nodes per elements) 
       */
      fmatvec::Matrix<fmatvec::General,int> ElementNodeList;

      /** 
       * \brief total dof of disk with reference movement and elastic deformation but without including bearing
       */
      int Dofs;
      
      /**
       * \brief dirichlet boundary condition concerning reference movement
       *
       * possible settings: innering/outerring
       */
      LockType LType;
      
      /**
       * \brief index of condensated dofs
       */
      fmatvec::Index ILocked; 
      
      /** 
       * \brief position and velocity with respect to Dofs
       */
      fmatvec::Vec qext, uext; 
      
      /** 
       * \brief Jacobian for condensation with size Dofs x qSize
       */
      fmatvec::Mat Jext;

      /**
       * \brief contour for contact description
       */
      NurbsDisk2s *contour;

      /*! 
       * \brief detect involved element for contact description
       * \param parametrisation vector (radial / azimuthal)
       */
      void BuildElement(const fmatvec::Vec &s);

      /*! 
       * \brief calculate constant mass and stiffness matrix
       */
      void initMatrices();

      /*! 
       * \return thickness of disk at radial coordinate
       * \param radial coordinate
       */
      double computeThickness(const double &r_);
  };

  inline void FlexibleBody2s13Disk::GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) { throw new MBSimError("ERROR(FlexibleBody2s13Disk::GlobalVectorContribution): Not implemented!"); }
  inline void FlexibleBody2s13Disk::GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) { throw new MBSimError("ERROR(FlexibleBody2s13Disk::GlobalMatrixContribution): Not implemented!"); }
  inline void FlexibleBody2s13Disk::GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) { throw new MBSimError("ERROR(FlexibleBody2s13Disk::GlobalMatrixContribution): Not implemented!"); }

}

#endif /* _FLEXIBLE_BODY_2S_13_DISK_H_ */

