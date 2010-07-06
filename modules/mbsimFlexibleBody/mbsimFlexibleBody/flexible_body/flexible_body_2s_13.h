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

#ifndef FLEXIBLEBODY2S13_H_
#define FLEXIBLEBODY2S13_H_

#include "mbsim/flexible_body.h"

namespace MBSimFlexibleBody {

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
  fmatvec::Mat condenseMatrixRows(fmatvec::Mat A, fmatvec::Index I); //TODO

  /*!
   * \brief condenses cols of matrix concerning index
   * \param input matrix
   * \param indices to be condensed
   * \return condensed matrix
   */
  fmatvec::Mat condenseMatrixCols(fmatvec::Mat A, fmatvec::Index I); //TODO

  /*!
   * \brief condenses symmetric matrix concerning index
   * \param input matrix
   * \param indices to be condensed
   * \return condensed matrix
   */
  fmatvec::SymMat condenseMatrix(fmatvec::SymMat A,fmatvec::Index I); //TODO

  /*!
   * \brief calculates planar angle in [0,2\pi] with respect to Cartesian coordinates
   * \param Cartesian x-coordinate
   * \param Cartesian y-coordinate
   * \return angle
   */
  double ArcTan(double x,double y); //TODO: ArcTan da rausziehen??

  /*!
   * \brief plate according to Reissner-Mindlin with moving frame of reference
   * \author Kilian Grundl
   * \author Thorsten Schindler
   * \date 2010-04-23 initial commit (Schindler / Grundl)
   */
  class FlexibleBody2s13 : public MBSim::FlexibleBodyContinuum<fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody2s13(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody2s13() {}

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t);
      virtual void updatedhdz(double t);
      virtual void updateStateDependentVariables(double t);
      /******************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void facLLM() {};

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody2s13"; }
      /***************************************************/

      /* GETTER/SETTER */
      void setRadius(double Ri_,double Ra_) { Ri = Ri_; Ra = Ra_; }
      void setEModul(double E_) { E = E_; }
      void setPoissonRatio(double nu_) { nu = nu_; }
      void setThickness(const fmatvec::Vec &d_) { d = d_; }
      void setDensity(double rho_) { rho = rho_; }
      int getReferenceDegreesOfFreedom() const { return RefDofs; }
      int getRadialNumberOfElements() const { return nr; }
      int getAzimuthalNumberOfElements() const { return nj; }
      double getInnerRadius() const { return Ri; }
      double getOuterRadius() const { return Ra; }
      double getAzimuthalDegree() const { return degU; }
      double getRadialDegree() const { return degV; }
      fmatvec::SqrMat getA() const { return A; }
      fmatvec::SqrMat getG() const { return G; }
      void setReferenceInertia(double m0_, fmatvec::SymMat J0_) { m0 = m0_; J0 = J0_; }
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
      double computePotentialEnergy() { return 0.5*q.T()*K*q; }

      /*!
       * \brief transform Cartesian to cylinder system
       * \param Cartesian vector in world system of plate
       * \return cylindrical coordinates
       */
      virtual fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint) = 0;

    protected:
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
       * \brief mass of the attached shaft
       */
      double m0;

      /**
       * \brief inertia of the attached shaft
       */
      fmatvec::SymMat J0;

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
      fmatvec::SymMat MConst;

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
       *
       * NodeCoordinates(GlobalNodeNumber,0) = radius (at the node)
       * NodeCoordinates(GlobalNodeNumber,1) = angle (at the node)
       */
      fmatvec::Mat NodeCoordinates;

      /**
       * \brief matrix mapping elements and nodes (size number of elements x number of nodes per elements)
       *
       * ElementNodeList(Element,LocalNodeNumber) = globalNodeNumber;
       */
      fmatvec::Matrix<fmatvec::General,int> ElementNodeList;

      /**
       * \brief total dof of disk with reference movement and elastic deformation but without including bearing
       */
      int Dofs;

      /**
       * \brief Dirichlet boundary condition concerning reference movement
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
       * \brief transformation matrix of coordinates of the moving frame of reference into the reference frame
       */
      fmatvec::SqrMat A;

      /**
       * \brief transformation matrix of the time derivates of the angles into tho angular velocity in reference coordinates
       */
      fmatvec::SqrMat G;

      /*
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
       * \brief calculate the matrices for the first time
       */
      virtual void initMatrices() = 0;

      /*!
       * \brief update the transformation matrices A and G
       */
      virtual void updateAG() = 0;

      /*!
       * \brief rotation matrix about z-axis
       * \param angle of rotation
       */
      fmatvec::SqrMat TransformationMatrix(const double &phi);

      /*!
       * \return thickness of disk at radial coordinate
       * \param radial coordinate
       */
      double computeThickness(const double &r_);
  };

}

#endif /* FLEXIBLEBODY2S13_H_ */

