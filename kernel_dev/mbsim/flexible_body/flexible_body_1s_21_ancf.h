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

#ifndef _BODY_FLEXIBLE_1S_21_ANCF_H_
#define _BODY_FLEXIBLE_1S_21_ANCF_H_

#include "body_flexible.h"

namespace MBSim {

  class FiniteElement1s21ANCF;
  class Contour1sFlexible;

  /*! Absolute Nodal Coordinate Formulation (Prof. A.A. Shabana) for flexible planar beam systems, implementation analogous to BodyFlexible1s21RCM
   *
   * */
  class BodyFlexible1s21ANCF : public BodyFlexible1s {

    protected:
      FiniteElement1s21ANCF *balken;
      static const int NodeDOFs;
      static const int ElementalDOFs;

      int Elements;
      double L, E, A, I, rho;
      bool openStructure;
      bool implicit;

      Vec qElement,uElement;
      int CurrentElement;
      SqrMat Dhq, Dhqp;

      // KOS-Definition und Lage des ersten Knoten im Weltsystem
      Vec WrON00,WrON0;

      void   BuildElement(const int&);
      double BuildElement(const double&);

      void updateStateDependentVariables(double t);
      void updatePorts(double t);

      void updateh(double t);
      void init();
      void initM();
      bool initialized;
      double alphaRelax0, alphaRelax;

      double sTangent;
      Vec Wt, Wn, WrOC, WvC, Womega;

      /** right and left side contour of body: defined using binormal of contour */
      Contour1sFlexible *contourR, *contourL;

    public:
      BodyFlexible1s21ANCF(const string &name, bool openStructure); 

      void setNumberElements(int n); 
      void setLength(double L_)             {L = L_;}
      void setEModul(double E_)             {E = E_;}
      void setCrossSectionalArea(double A_) {A = A_;}
      void setMomentInertia(double I_)      {I = I_;}
      void setDensity(double rho_)          {rho = rho_;}
      /*     void setCurleRadius(double r)         {rc = r;if(initialized) balken->setCurleRadius(rc);} */
      /*     void setMaterialDamping(double d)     {dm = d;if(initialized) balken->setMaterialDamping(dm);} */
      /*     void setLehrDamping(double d)         {dl = d;if(initialized) balken->setLehrDamping(dl);} */

      using BodyFlexible1s::addPort;
      /*! add Port at
       * \param node
       */
      void addPort(const string &name, const int &node);

      Mat computeJacobianMatrix(const ContourPointData &data);

      Mat computeWt  (const ContourPointData &S_);
      Vec computeWn  (const ContourPointData &S_);
      Vec computeWrOC(const ContourPointData &S_);
      Vec computeWvC (const ContourPointData &S_);
      Vec computeWomega(const ContourPointData &S_);

      bool hasConstMass() const {return true;}

      void setJT(const Mat &JT_) {assert(JT_.cols()==2); JT = JT_;};
      void setJR(const Mat &JR_) {assert(JR_.cols()==1); JR = JR_;};

      void setWrON00(const Vec &WrON00_) {WrON00 = WrON00_;}
      void initRelaxed(double alpha);
  };

}

#endif
