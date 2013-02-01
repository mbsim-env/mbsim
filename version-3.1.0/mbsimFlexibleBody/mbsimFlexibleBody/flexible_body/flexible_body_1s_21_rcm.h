/* Copyright (C) 2004-2011 MBSim Development Team
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
 *          rzander@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_1S_21_RCM_H_
#define _FLEXIBLE_BODY_1S_21_RCM_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

namespace MBSimFlexibleBody {

  /*!
   * \brief model for planar beams with large deflection using Redundant Coordinate Method (RCM)
   * \author Roland Zander
   * \date 2009-03-23 initial kernel_dev commit (Thorsten Schindler)
   * \date 2009-03-26 cosmetics on doxygen (*! **) and alignements; some renames (Roland Zander)
   * \date 2009-04-05 minor change: parent class now is FlexibleBodyContinuum (Schindler / Zander)
   * \date 2009-04-20 binormals of contours can be time variant (Thorsten Schindler)
   * \date 2009-05-08 visualisation (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-23 implicit integration (Thorsten Schindler)
   * \date 2010-03-07 element polt list for additional element data (Roland Zander)
   * \date 2010-04-07 added initInfo() function same as in FlexibleBody1s33RCM (Thomas Cebulla)
   * \todo gyroscopic accelerations TODO
   * \todo inverse kinetics TODO
   *
   * read:\n
   * Zander, R.; Ulbrich, H.: Reference-free mixed FE-MBS approach for beam structures with constraints, Journal of Nonlinear Dynamics, Kluwer Academic Publishers, 2005 \n
   * Zander, R.; Ulbrich, H.: Impacts on beam structures: Interaction of wave propagationand global dynamics, IUTAM Symposium on Multiscale Problems in Multibody System Contacts Stuttgart, Germany, 2006 \n
   * Zander, R.; Ulbrich, H.: Free plain motion of flexible beams in MBS - A comparison of models, III European Conference on Computational Mechanics Lissbon, Portugal, 2006
   */
  class FlexibleBody1s21RCM : public FlexibleBodyContinuum<fmatvec::Fixed<8>, double> {
    public:
      /*!
       * \brief constructor:
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s21RCM(const std::string &name, bool openStructure);

      /*!
       * \brief destructor
       */
      virtual ~FlexibleBody1s21RCM() {}

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat8& locMat, fmatvec::SymMat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SqrMat8& locMat, fmatvec::Mat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      virtual void exportPositionVelocity(const std::string& filenamePos, const std::string& filenameVel = std::string( ), const int & deg = 3, const bool &writePsFile = false);
      virtual void importPositionVelocity(const std::string& filenamePos, const std::string& filenameVel = std::string( ));
      /****************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s21RCM"; }
      /***************************************************/

      /* GETTER / SETTER */
      /**
       * \brief sets size of positions and velocities
       */
      void setNumberElements(int n);
      int getNumberElements(){return Elements;}
      void setLength(double L_) { L = L_; }
      double getLength(){return L;}
      void setEModul(double E_) { E = E_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentInertia(double I_) { I = I_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCurlRadius(double r);
      void setMaterialDamping(double d);
      void setLehrDamping(double d);
      void setElementPlotList(fmatvec::VecInt plotElements_) {plotElements = plotElements_;}
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif
      double getLength() const { return L; }
      /***************************************************/

      /**
       * \brief compute state (positions, angles, velocities, differentiated angles) at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vec computeState(double x);

      /**
       * \brief initialise beam state concerning a straight cantilever setting or a circle shaped ring
       * \param angle of slope in case of cantilever
       */
      void initRelaxed(double alpha);

      /**
       * \brief initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

    protected:
      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);

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
       * \brief curl radius
       */
      double rc;

      /**
       * \brief coefficient of material damping
       */
      double dm;

      /**
       * \brief coefficient of Lehr-damping
       */
      double dl;

      /**
       * \brief coefficient of Lehr-damping
       */
      fmatvec::VecInt plotElements;

      /**
       * \brief flag for open (cantilever beam) or closed (rings) structures
       */
      bool openStructure;

      /**
       * \brief flag for testing if beam is initialised
       */
      bool initialized;

      /**
       * \brief contour of body
       */
      Contour1sFlexible<fmatvec::Fixed<8> > *contour1sFlexible;
  };

}

#endif /* _FLEXIBLE_BODY_1S_21_RCM_H_ */

