/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _FLEXIBLE_BODY_1S_33_ANCF_H_
#define _FLEXIBLE_BODY_1S_33_ANCF_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_ancf.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif


namespace MBSimFlexibleBody {

  /**
   * \brief Absolute Nodal Coordinate Formulation for flexible spatial beams
   *
   *  based on cable element with no explicit torsion, i.e. angular velocity and Jacobian of rotation depend only on two bending angles
   *
   * \author Thorsten Schindler
   *
   * \date 2014-03-20 initial commit
   *
   * \todo angular velocity and Jacobian of rotation for NODE case
   *
   * model based on
   * DOMBROWSKI, S.: Analysis of Large Flexible Body Deformation in Multibody Systems Using Absolute Coordinates. Multibody System Dynamics (2002)
   */
  class FlexibleBody1s33ANCF : public FlexibleBodyContinuum<double> {

    public:
      /*!
       * \brief constructor:
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s33ANCF(const std::string &name, bool openStructure);

      /*!
       * \brief destructor
       */
      virtual ~FlexibleBody1s33ANCF() {}

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void updateM(double t, int k) {}
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      /****************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      virtual void facLLM(int i = 0) {}
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s33ANCF"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);
      void setLength(double L_) { L = L_; }
      void setEModul(double E_) { E = E_; }
      void setShearModul(double G_) { G = G_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentInertia(double I0_,double I1_,double I2_) { I0 = I0_; I1 = I1_; I2 = I2_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCurlRadius(double rc1_,double rc2_);
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif
      int getNumberElements(){ return Elements; }
      double getLength(){ return L; }
      /***************************************************/

      /**
       * \brief compute planar state at Lagrangian coordinate
       * \param Lagrangian coordinate
       */
      fmatvec::Vec computeState(double x);

      /**
       * \brief initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

      /**
       * \brief initialise beam state concerning a straight cantilever setting or a circle shaped ring
       * \param angle of slope in case of cantilever
       */
      void initRelaxed(double alpha);

    private:
      /**
       * \brief number of finite elements used for discretisation
       */
      int Elements;

      /**
       * \brief length of beam
       */
      double L;

      /**
       * \brief length of one finite element
       */
      double l0;

      /**
       * \brief modulus of linear elasticity
       */
      double E;

      /**
       * \brief shear modulus
       */
      double G;

      /**
       * \brief cross-section area
       */
      double A;

      /**
       * \brief polar moment of inertia of cross-section
       */
      double I0;

      /**
       * \brief moment of inertia of cross-section
       */
      double I1;

      /**
       * \brief moment of inertia of cross-section
       */
      double I2;

      /**
       * \brief material density
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       */
      double rc1;

      /**
       * \brief radius of undeformed shape
       */
      double rc2;

      /**
       * \brief flag for open (cantilever beam) or closed (rings) structures
       */
      bool openStructure;

      /**
       * \brief flag for testing if beam is initialised
       */
      bool initialised;

      /**
       * \brief initialize mass matrix and calculate Cholesky decomposition
       */
      void initM();

      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);

      /**
       * \brief copy constructor is declared private
       */
      FlexibleBody1s33ANCF(const FlexibleBody1s33ANCF&);

      /**
       * \brief assignment operator is declared private
       */
      FlexibleBody1s33ANCF& operator=(const FlexibleBody1s33ANCF&);
  };

  inline void FlexibleBody1s33ANCF::setCurlRadius(double rc1_, double rc2_) {
    rc1 = rc1_;
    rc2 = rc2_;
    if(initialised)
      for(int i = 0; i < Elements; i++)
        static_cast<FiniteElement1s33ANCF*>(discretization[i])->setCurlRadius(rc1,rc2);
  }

}
#endif /* _FLEXIBLE_BODY_1S_33_ANCF_H_ */

