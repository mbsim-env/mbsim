/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: thomas.cebulla@mytum.de
 */

#ifndef _FLEXIBLE_BODY_1S_21_COSSERAT_H_
#define _FLEXIBLE_BODY_1S_21_COSSERAT_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_cosserat_translation.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_cosserat_rotation.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

namespace MBSimFlexibleBody {

  class NurbsCurve1s;

  /**
   * \brief flexible body for planar beam using Cosserat model
   * \author Thomas Cebulla
   * \author Thorsten Schindler
   * \author Robert von Zitzewitz
   * \date 2012-12-14 initial commit (Thomas Cebulla)
   * \todo compute boundary conditions TODO
   * \todo check open structure in contact kinematics TODO
   *
   * Cosserat model based on
   * H. Lang, J. Linn, M. Arnold: Multi-body dynamics simulation of geometrically exact Cosserat rods
   * but with 
   *  - Kirchhoff assumption (-> less stiff)
   *  - Cardan parametrisation (-> less problems with condition and drift for quaternion dae system)
   *  - piecewise constant Darboux vector with evaluation according to
   *    I. Romero: The interpolation of rotations and its application to finite element models of
   *    geometrically exact beams
   */
  class FlexibleBody1s21Cosserat : public FlexibleBodyContinuum<fmatvec::Ref, double> { //TODO: avoid Ref-type here (and elsewhere in class...)
    public:

      /**
       * \brief constructor
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s21Cosserat(const std::string &name, bool openStructure); 

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody1s21Cosserat();

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      virtual void exportPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string( ), const int & deg = 3, const bool & writePsFile = false);
      virtual void importPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string( ));
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      virtual double computePotentialEnergy();
      virtual void facLLM(int i=0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t, int i=0);
      virtual void updateStateDependentVariables(double t);

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s21Cosserat"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);
      void setLength(double L_);
      void setEGModuls(double E_,double G_);
      void setDensity(double rho_);
      void setCrossSectionalArea(double A_);
      void setMomentsInertia(double I1_,double I2_,double I0_);
      void setCurlRadius(double R1_,double R2_);
      void setMaterialDamping(double cEps0D_,double cEps1D_,double cEps2D_);
      void setCylinder(double cylinderRadius_);
      void setCuboid(double cuboidBreadth_,double cuboidHeight_);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif

      int getNumberElements() const { return Elements; }   	
      int getNumberDOFs() const { return qSize; }
      double getLength() const { return L; }
      bool isOpenStructure() const { return openStructure; }
      /***************************************************/

      /**
       * \brief compute state (positions, angles, velocities, differentiated angles) at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vec computeState(double s);

      /**
       * \brief compute angles at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vec3 computeAngles(double s);

      /**
       * \brief initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

    private:
      /** 
       * \brief stl-vector of finite elements for rotation grid
       */
      std::vector<MBSim::DiscretizationInterface<fmatvec::Ref>*> rotationDiscretization;

      /** 
       * \brief stl-vector of finite element positions for rotation grid
       */
      std::vector<fmatvec::Vec> qRotationElement;

      /** 
       * \brief stl-vector of finite element wise velocities for rotation grid
       */
      std::vector<fmatvec::Vec> uRotationElement;

      /**
       * \brief contours
       */
      CylinderFlexible<fmatvec::Ref> *cylinder;
      FlexibleBand<fmatvec::Ref> *top, *bottom, *left, *right;
      Contour1sFlexible<fmatvec::Ref> *neutralFibre;

      /**
       * \brief angle parametrisation
       */
      CardanPtr angle;

      /**
       * \brief number of translational elements
       */
      int Elements;

      /**
       * \brief number of rotational elements =Elements (for a closed structure) or =Elements+1 (for an open structure)
       */
      int rotationalElements;

      /**
       * \brief length of entire beam and finite elements
       */
      double L, l0;

      /**
       * \brief elastic modules
       */
      double E, G;

      /**
       * \brief area of cross-section
       */
      double A;

      /**
       * \brief area moments of inertia
       * I0: around torsional axis
       * I1: in t-b-plane
       * I2: in t-n-plane
       */
      double I1, I2, I0;

      /**
       * \brief density 
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       * R1: in t-b-plane
       * R2: in t-n-plane
       */
      double R1, R2;

      /**
       * \brief strain damping 
       */
      double cEps0D, cEps1D, cEps2D;

      /**
       * \brief open or closed beam structure
       */
      bool openStructure;

      /**
       * \brief initialised FLAG 
       */
      bool initialised;

      /**
       * \brief boundary conditions for rotation grid
       * first and last finite difference rotation beam element refer to values not directly given by dof in open structure
       * they have to be estimated by the following values calculated in computeBoundaryCondition() 
       */
      fmatvec::Vec bound_ang_start;
      fmatvec::Vec bound_ang_end;
      fmatvec::Vec bound_ang_vel_start;
      fmatvec::Vec bound_ang_vel_end;

      /**
       * \brief contour data 
       */
      double cuboidBreadth, cuboidHeight, cylinderRadius;

      /**
       * \brief contour for state description
       */
      NurbsCurve1s *curve;

      FlexibleBody1s21Cosserat(); // standard constructor
      FlexibleBody1s21Cosserat(const FlexibleBody1s21Cosserat&); // copy constructor
      FlexibleBody1s21Cosserat& operator=(const FlexibleBody1s21Cosserat&); // assignment operator

      /**
       * \brief detect current finite element (translation)
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElementTranslation(const double& sGlobal, double& sLocal, int& currentElementTranslation);

      /**
       * \brief initialize translational part of mass matrix and calculate Cholesky decomposition
       */
      void initM();

      /**
       * \brief compute boundary conditions for rotation grid
       * first and last finite difference rotation beam element refer to values not directly given by dof in open structure
       * they have to be estimated in the following function
       */
      void computeBoundaryCondition();

      /** 
       * \brief insert 'local' information in global vectors for rotation grid
       * \param number of finite element
       * \param local vector
       * \param global vector
       */
      void GlobalVectorContributionRotation(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
  };

  inline void FlexibleBody1s21Cosserat::setLength(double L_) { L = L_; }
  inline void FlexibleBody1s21Cosserat::setEGModuls(double E_,double G_) { E = E_; G = G_; }    	
  inline void FlexibleBody1s21Cosserat::setDensity(double rho_) { rho = rho_;}     	
  inline void FlexibleBody1s21Cosserat::setCrossSectionalArea(double A_) { A = A_; }    	
  inline void FlexibleBody1s21Cosserat::setMomentsInertia(double I1_, double I2_, double I0_) { I1 = I1_; I2 = I2_; I0 = I0_; }    	
  inline void FlexibleBody1s21Cosserat::setCurlRadius(double R1_,double R2_) { R1 = R1_; R2 = R2_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21CosseratRotation*>(rotationDiscretization[i])->setCurlRadius(R1,R2); }
  inline void FlexibleBody1s21Cosserat::setMaterialDamping(double cEps0D_,double cEps1D_,double cEps2D_) { cEps0D = cEps0D_; cEps1D = cEps1D_; cEps2D = cEps2D_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21CosseratTranslation*>(discretization[i])->setMaterialDamping(Elements*cEps0D,cEps1D,cEps2D); }
  inline void FlexibleBody1s21Cosserat::setCylinder(double cylinderRadius_) { cylinderRadius = cylinderRadius_; }
  inline void FlexibleBody1s21Cosserat::setCuboid(double cuboidBreadth_,double cuboidHeight_) { cuboidBreadth = cuboidBreadth_; cuboidHeight = cuboidHeight_; }

}

#endif /* _FLEXIBLE_BODY_1S_21_COSSERAT_H_ */

