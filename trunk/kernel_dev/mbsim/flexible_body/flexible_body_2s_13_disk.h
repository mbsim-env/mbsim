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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#endif

namespace MBSim {

  class NurbsDisk2s;
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
   * \todo gravity, contour, visualisation
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
      void initPlot();
      using FlexibleBodyContinuum<fmatvec::Vec>::addFrame;
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
      virtual void init();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody2s13Disk"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double Ri_,double Ra_) { Ri = Ri_; Ra = Ra_; }		
      void setEModul(double E_) { E = E_; }				
      void setPoissonRatio(double nu_) { nu = nu_; }			
      void setThickness(const fmatvec::Vec &di_,const fmatvec::Vec &da_) { di = di_; da = da_; }		    
      void setDensity(double rho_) { rho = rho_; }	
      int getNr() const {return nr;}
      int getNj() const {return nj;}
      double getRi() const {return Ri;}
      double getRa() const {return Ra;}
      void setRefInertias(double m0_, double J0_) { m0 = m0_; J0 = J0_; }	
      void setLockType(LockType LT_) { LType = LT_; }
      /***************************************************/

      /*! set number of elements in radial and azimuthal direction */
      void setNumberElements(int nr_,int nj_);

      /*! computes the potential energy */
      double computePotentialEnergy() {return 0.5*trans(q)*K*q;}

      /*! transformation kartesian to cylinder system */
      fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint);

    private:
      /** total number of elements */
      int Elements;
      /** elastic dof per node */
      int NodeDofs;
      /** dof of moving frame of reference */
      int RefDofs;

      /** Young's modulus */
      double E;
      /** Poisson ratio */
      double nu;
      /** density */
      double rho;
      /** inner and outer thickness */
      fmatvec::Vec di, da;
      /** inner and outer radius of disk */
      double Ri, Ra;
      /** radial and azimuthal length of an FE */
      double dr, dj;
      /** mass and inertia of the attached shaft */
      double m0, J0;
      /** Degree of surface interpolation in radial and azimuthal direction */
      int degV, degU;

      /** vector of boundary data of the FE (r1,j1,r2,j2) */
      std::vector<fmatvec::Vec> ElementalNodes;
      /** number of element currently involved in contact calculations */
      int currentElement;
      /** stiffness matrix */
      fmatvec::SymMat K; // TODO not initialised in constructor 

      /** organisation of node coordinates */
      /** number of elements in radial and azimuthal direction, number of FE nodes */
      int nr, nj, Nodes;
      /** matrix mapping nodes and coordinates (size number of nodes x number of node coordinates) */
      fmatvec::Mat NodeCoordinates; // TODO not initialised in constructor 
      /** matrix mapping elements and nodes (size number of elements x  number of nodes per elements) */
      fmatvec::Matrix<fmatvec::General,int> ElementNodeList; // TODO not initialised in constructor 

      /** condensation setting */
      /** total dof of disk with reference movement and elastic deformation but without including bearing */
      int Dofs;
      /** dirichlet boundary condition concerning reference movement 
       * possible settings: innering/outerring
       */
      LockType LType;
      /** index of condensated dofs */
      fmatvec::Index ILocked; // TODO not initialised in constructor 
      /** position and velocity with respect to Dofs */
      fmatvec::Vec qext, uext; // TODO not initialised in constructor 
      /** Jacobian for condensation with size Dofs x qSize */
      fmatvec::Mat Jext; // TODO not initialised in constructor 

      /** contour for contact description */
      NurbsDisk2s *contour;

      /*! detect involved element for contact description */
      void BuildElement(const fmatvec::Vec &s);

      /*! calculate constant mass and stiffness matrix */
      void initMatrices();

      /*! updates kinematics of disk */
      void updateStateDependentVariables(double t);
      /*! updates vector of generalized forces */
      void updateh(double t);

      /*! computes thickness of disk at radial coordinate */
      fmatvec::Vec computeThickness(const double &r_);
  };

  inline void FlexibleBody2s13Disk::GlobalMatrixContribution(int CurrentElement) { throw new MBSimError("ERROR(FlexibleBody2s13Disk::GlobalMatrixContribution): Not implemented!"); }

}

#endif /* _FLEXIBLE_BODY_2S_13_DISK_H_ */

