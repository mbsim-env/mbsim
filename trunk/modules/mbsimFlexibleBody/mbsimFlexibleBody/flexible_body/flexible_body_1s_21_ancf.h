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

#ifndef _FLEXIBLE_BODY_1S_21_ANCF_H_
#define _FLEXIBLE_BODY_1S_21_ANCF_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_ancf.h"
#include "mbsim/mbsim_event.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif


namespace MBSimFlexibleBody {

  /**
   * \brief Absolute Nodal Coordinate Formulation for flexible planar beams
   * \author Roland Zander
   * \author Thorsten Schindler
   *
   * \date 2014-02-27 basic revision
   * \date 2014-03-23 damping added
   * \date 2014-05-09 Euler perspective added as an option
   *
   * model based on
   * SHABANA, A. A.: Computer Implementation of the Absolute Nodal Coordinate Formulation for Flexible Multibody Dynamics. In: Nonlinear Dynamics 16 (1998), S. 293-306
   * SHABANA, A. A.: Definition of the Slopes and the Finite Element Absolute Nodal Coordinate Formulation. In: Nonlinear Dynamics 1 (1997), S. 339-348
   * SHABANE, A. A.: Dynamics of Multibody Systems. Cambridge University Press (2005)
   */
  class FlexibleBody1s21ANCF : public FlexibleBodyContinuum<double> {

    public:
      /*!
       * \brief constructor:
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s21ANCF(const std::string &name, bool openStructure);

      /*!
       * \brief destructor
       */
      virtual ~FlexibleBody1s21ANCF() {}

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
      virtual void init(InitStage stage);
      virtual void facLLM(int i = 0) {}
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s21ANCF"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);
      void setLength(double L_) { L = L_; }
      void setEModul(double E_) { E = E_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentInertia(double I_) { I = I_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCurlRadius(double rc_);
      void setMaterialDamping(double deps_, double dkappa_);
      void setEulerPerspective(bool Euler_, double v0_);
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
       * \brief cross-section area
       */
      double A;

      /**
       * \brief moment of inertia of cross-section
       */
      double I;

      /**
       * \brief material density
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       */
      double rc;

      /**
       * \brief coefficient of material longitudinal damping
       */
      double deps;

      /**
       * \brief coefficient of material longitudinal damping
       */
      double dkappa;

      /**
       * \brief flag for open (cantilever beam) or closed (rings) structures
       */
      bool openStructure;

      /**
       * \brief flag for testing if beam is initialised
       */
      bool initialised;

      /**
       * \brief Euler perspective: constant longitudinal velocity
       */
      double v0;

      /**
       * \brief Euler perspective: true if set
       */
      bool Euler;

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
  };

  inline void FlexibleBody1s21ANCF::setCurlRadius(double rc_) {
    rc = rc_;
    if(initialised)
      for(int i = 0; i < Elements; i++)
        static_cast<FiniteElement1s21ANCF*>(discretization[i])->setCurlRadius(rc);
  }

  inline void FlexibleBody1s21ANCF::setMaterialDamping(double deps_, double dkappa_) {
    deps = deps_;
    dkappa = dkappa_;
    if(initialised)
      for(int i = 0; i < Elements; i++) 
        static_cast<FiniteElement1s21ANCF*>(discretization[i])->setMaterialDamping(deps,dkappa);
  }

  inline void FlexibleBody1s21ANCF::setEulerPerspective(bool Euler_, double v0_) { 
    if(openStructure) {
      throw(new MBSim::MBSimError("ERROR (FlexibleBody1s21ANCF::setEulerPerspective): implemented only for closed structures!"));
    }
    else {
      Euler = Euler_; 
      v0 = v0_;
    }
  }
}
#endif /* _FLEXIBLE_BODY_1S_21_ANCF_H_ */

