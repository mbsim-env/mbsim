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

#include <fmatvec.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_cosserat.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_cosserat_translation.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_cosserat_rotation.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

using namespace fmatvec;

namespace MBSimFlexibleBody {

  class NurbsCurve1s;

  /**
   * \brief flexible body for planar beam using Cosserat model
   * \author Thomas Cebulla
   * \author Thorsten Schindler
   * \author Robert von Zitzewitz
   * \date 2012-12-14 initial commit (Thomas Cebulla)
   * \date 2013-02-04 completed 2D Cosserat beam for closed structure (Robert von Zitzewitz)
   * \date 2013-02-04 completed POD model reduction for TIMESTEPPING integrator (Robert von Zitzewitz)
   * \todo compute boundary conditions TODO
   * \todo open structure TODO
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
  class FlexibleBody1s21Cosserat : public FlexibleBody1sCosserat {
    public:


      /**
       * \brief bool true: execute POD, false: without POD
       */
      bool SVD;

      /**
       * \brief POM projection matrix
       */
      fmatvec::Mat U;

      /**
       * \brief LU decomposition of reduced mass matrix
       */
      fmatvec::SymMat LLUMUT[2];

      /**
       * \brief reduced differentiated velocity
       */
      fmatvec::Vec udSVD[2];

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
      virtual void updateM(double t, int k=0){};
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
      virtual void updatedu(double t, double dt);
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

      void setMomentsInertia(double I1_);
      void setCurlRadius(double R1_);
      void setMaterialDamping(double cEps0D_,double cEps1D_);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif

      int getNumberElements() const { return Elements; }   	
      int getNumberDOFs() const { return qSize; }
      double getLength() const { return L; }
      bool isOpenStructure() const { return openStructure; }

      /**
       * \brief initialize POM projection matrix
       * \param  POM matrix
       */
      void setU(Mat U_){U << U_;}

      /**
       * \brief initialize LU decomposed constant reduced mass  matrix
       * \param  reduced mass matrix
       */
      void setLLUMUT(SymMat LLUMUT_){
        LLUMUT[0].resize(LLUMUT_.size(),INIT,0.);
        LLUMUT[0] << LLUMUT_;
      }

      /* \brief Set SVD=true for POD model reduction
       * \param  reduced mass matrix
       */
      void setSVD(){SVD=true;}
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

  inline void FlexibleBody1s21Cosserat::setMomentsInertia(double I1_) { I1 = I1_;}

  inline void FlexibleBody1s21Cosserat::setCurlRadius(double R1_) { R1 = R1_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21CosseratRotation*>(rotationDiscretization[i])->setCurlRadius(R1); }
  inline void FlexibleBody1s21Cosserat::setMaterialDamping(double cEps0D_,double cEps1D_) { cEps0D = cEps0D_; cEps1D = cEps1D_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21CosseratTranslation*>(discretization[i])->setMaterialDamping(Elements*cEps0D,cEps1D); }

}

#endif /* _FLEXIBLE_BODY_1S_21_COSSERAT_H_ */

