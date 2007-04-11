/* Copyright (C) 2005-2006  Roland Zander
 
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
 * Contact:
 *   rzander@users.berlios.de
 *
 */

#ifndef _BODY_FLEXIBLE_1S_21_RCM_H_
#define _BODY_FLEXIBLE_1S_21_RCM_H_

#include "body_flexible.h"

namespace AMVis {class ElasticBody1s21RCM;};

namespace MBSim {

  class FiniteElement1s21RCM;

  /*! \brief Model for beams ...
   *
   * BodyFlebile describing beams with additional longitudinal elasticity. Based upon 
   * FiniteElement1s21RCM for 2D-Beam using Redundant Coordinate Method(RCM).\n
   * Roland Zander, Lehrstuhl fuer Angewandte Mechanik, TU Muenchen
   *
   * read:\n
   * Zander, R.; Ulbrich, H.: Reference-free mixed FE-MBS approach for beam structures withconstraints, Journal of Nonlinear Dynamics, Kluwer Academic Publishers,2005 \n
   * Zander, R.; Ulbrich, H.: Impacts on beam structures: Interaction of wave propagationand global dynamics, IUTAM Symposium on Multiscale Problems inMultibody System Contacts Stuttgart, Germany, 2006 \n
   * Zander, R.; Ulbrich, H.: Free plain motion of flexible beams in MBS - A comparison ofmodels, III European Conference on Comutational Mechanics Lissbon,Portugal, 2006
   *
   *
   * */
  class BodyFlexible1s21RCM : public BodyFlexible1s {

    protected:
      /** the one finite element used for all calculations */
      FiniteElement1s21RCM *balken;

      /** number of finite elements used for discretisation */
      int Elements;
      /** length of beam */ 
      double L;
      /** modulus of linear elasticity */
      double E;
      /** cross-section area */
      double A;
      /** moment of inertia of cross-section */
      double I;
      /** meterial density */
      double rho;
      /** curle radius */
      double rc;
      /** coefficient of material damping */
      double dm;
      /** coefficient of Lehr-damping */
      double dl;
      /** flag for open (cantilever beam) or closed (rings) structures */ 
      bool openStructure;
      /** true if terms for implicit time integration shall be computed, unused so far */ 
      bool implicit;

      Vec qElement,uElement;
      int CurrentElement;
      SqrMat Dhq, Dhqp;

      // KOS-Definition und Lage des ersten Knoten im Weltsystem
      Vec WrON00,WrON0;

      void   BuildElement(const int&);
      double BuildElement(const double&);

      /* geerbt */
      void updateKinematics(double t);
      /* geerbt */
      void updatePorts(double t);
      /* geerbt */
      Mat computeJacobianMatrix(const ContourPointData &S);
      /* geerbt */
      void updateh(double t);

      /* geerbt */
      void init();
      bool initialized;
      double alphaRelax0, alphaRelax;

      double sTangent;
      Vec Wt, Wn, WrOC, WvC, Womega;

    public:
      /*! Constructor:
       * \param name body name
       * \param openStructure bool to specify open (catilever) = true or closed (ring) = false structure
       * */
      BodyFlexible1s21RCM(const string &name, bool openStructure); 

      /*! set number of Elements used for discretisation */
      void setNumberElements(int n); 
      /*! set lenght of beam */
      void setLength(double L_)             {L = L_;}
      /*! set modulus of elasticity */
      void setEModul(double E_)             {E = E_;}
      /*! set area of cross-section */
      void setCrossSectionalArea(double A_) {A = A_;}
      /*! set moment of inertia of cross-section */
      void setMomentInertia(double I_)      {I = I_;}
      /*! set homogenous density */
      void setDensity(double rho_)          {rho = rho_;}
      /*! set radius of undeformed shape */
      void setCurleRadius(double r);
      /*! set material damping coeffizient */
      void setMaterialDamping(double d);
      /*! set damping as Lehr-coefficient */
      void setLehrDamping(double d);

      using BodyFlexible1s::addPort;
      /*! add Port 
       * \param name name of Port for referenzation
       * \param node number of FE node
       */
      void addPort(const string &name, const int &node);

      /* geerbt */
      Mat computeWt  (const ContourPointData &S_);
      Vec computeWn  (const ContourPointData &S_);
      Vec computeWrOC(const ContourPointData &S_);
      Vec computeWvC (const ContourPointData &S_);
      Vec computeWomega(const ContourPointData &S_);

      /*! compute state (position and velocities) of cross-section at \f$$\vs$\f$
       * \param s contour parameter
       * */
      Vec computeState(const double &s);

      /* geerbt */
      double computePotentialEnergy();
      /* geerbt */
      bool hasConstMass() const {return false;}

      /* geerbt */
      void setJT(const Mat &JT_) {assert(JT_.cols()==2); JT = JT_;};
      void setJR(const Mat &JR_) {assert(JR_.cols()==1); JR = JR_;};

      /*! define origin of describing coordinate system in world coordinates 
       * */
      void setWrON00(const Vec &WrON00_) {WrON00 = WrON00_;}
      void initRelaxed(double alpha);

      /* geerbt */
      void plotParameters();

  };

}

#endif
