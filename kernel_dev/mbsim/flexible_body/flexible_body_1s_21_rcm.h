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
 * Contact: rzander@users.berlios.de
 *          thschindler@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_1S_21_RCM_H_
#define _FLEXIBLE_BODY_1S_21_RCM_H_

#include <mbsim/flexible_body.h>

namespace AMVis { class ElasticBody1s21RCM; }

namespace MBSim {

  class FiniteElement1s21RCM;
  class Contour1sFlexible;

  /*!
   * \brief model for planar beams with large deflection using Redundant Coordinate Method (RCM)
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-03-23 initial kernel_dev commit (Thorsten Schindler)
   * \date 2009-03-26 cosmetics on doxygen (*! **) and alignements; some renames (Roland Zander)
   *
   * read:\n
   * Zander, R.; Ulbrich, H.: Reference-free mixed FE-MBS approach for beam structures with constraints, Journal of Nonlinear Dynamics, Kluwer Academic Publishers, 2005 \n
   * Zander, R.; Ulbrich, H.: Impacts on beam structures: Interaction of wave propagationand global dynamics, IUTAM Symposium on Multiscale Problems in Multibody System Contacts Stuttgart, Germany, 2006 \n
   * Zander, R.; Ulbrich, H.: Free plain motion of flexible beams in MBS - A comparison of models, III European Conference on Computational Mechanics Lissbon, Portugal, 2006
   */
  class FlexibleBody1s21RCM : public FlexibleBody<fmatvec::Vec> {
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

      /* INHERITED INTERFACE */
      /* FLEXIBLEBODY */
      virtual std::string getType() const { return "FlexibleBody1s21RCM"; }
      virtual void BuildElements();
      virtual void GlobalMatrixContribution(int n);
      virtual void updateKinematicsForFrame(ContourPointData &S_, Frame *frame=0);
      virtual void updateJacobiansForFrame(ContourPointData &data, Frame *frame=0);

      /* OBJECT */
      virtual void init();
      /***************************************************/

      /* GETTER / SETTER */
      /**
       * \brief sets size of positions and velocities
       */
      void setNumberElements(int n); 
      void setLength(double L_) { L = L_; }
      void setEModul(double E_) { E = E_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentInertia(double I_) { I = I_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCurlRadius(double r);
      void setMaterialDamping(double d);
      void setLehrDamping(double d);
      /***************************************************/

      /**
       * \brief initialise beam state concerning a straight cantilever setting or a circle shaped ring
       * \param angle of slope in case of cantilever
       */
      void initRelaxed(double alpha);

#ifdef HAVE_AMVIS
      /**
       * \param radius of cylinder for AMVis visualisation
       */
      void setAMVisCylinder(float r) { setPlotFeature(amvis, enabled); AMVisRadius=r; }

      /**
       * \param breadth of cuboid for AMVis visualisation
       * \param height of cuboid for AMVis visualisation
       */
      void setAMVisCuboid(float breadth, float height) { setPlotFeature(amvis, enabled); AMVisBreadth=breadth; AMVisHeight=height; }
#endif

    protected:
      /**
       * \brief detect current finite element
       * \param global parametrisation
       */
      double BuildElement(const double& sGlobal);

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
       * \brief flag for open (cantilever beam) or closed (rings) structures
       */ 
      bool openStructure;

      /**
       * \brief current finite element being evaluated
       */
      int CurrentElement;

      /**
       * \brief flag for testing if beam is initialised
       */
      bool initialized;

      /** 
       * \brief right and left side contour of body
       */
      Contour1sFlexible *contourR, *contourL;

#ifdef HAVE_AMVIS
      /** 
       * \brief AMVis visualisation for cylinder and cuboid
       */
      float AMVisRadius, AMVisBreadth, AMVisHeight;
#endif

  };

}

#endif

