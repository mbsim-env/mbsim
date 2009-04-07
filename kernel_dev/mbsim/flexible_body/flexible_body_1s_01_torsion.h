/* Copyright (C) 2005-2006  Rainer Britz, Roland Zander
 
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
 *    rzander@users.berlios.de
 *
 */

#ifndef BODY_FLEXIBLE_1S_01_TORSION_H_
#define BODY_FLEXIBLE_1S_01_TORSION_H_

#include "body_flexible.h"

namespace MBSim {

//  class Contour1sFlexible;

  /*! \brief torsional axis with polinomial ansatz system of order \f$n\f$ */
  class BodyFlexible1s01Torsion : public BodyFlexible1s {
    protected:
      int n;
      double E, rho, A, I , l;
      //    Vec JGeneralized(const double &s); // nur zur Kompatibilitaet

      Vec sTangent;
      Mat Wt;
      Vec Wn, CrOC, CvC;

      // KOS-Definition und Lage des ersten Knoten im Weltsystem
      //    Mat Jges;
      Vec Axis; /* Enspricht der neutralen Faser */
      Vec WrON00,WrON0;

      void init();
      void initMatrizes();

      void updateStateDependentVariables(double t);
      void updatePorts(double t);
      //void sumUpForceElements(double t);
      //void updateW(double t);

      void updateh(double t);


    public:

      BodyFlexible1s01Torsion(const string &name);
      /*! set number of polynom shape functions
      */
      void setNumberShapeFunctions(int n_); 
      /*! set cross-sectional area
      */
      void setCrossSectionalArea(double A_){A     = A_;}
      /*! set moment of inertia
      */ 
      void setMomentInertia(double I_){I     = I_;}
      /*! set E-modul
      */ 
      void setEModul(double E_){E     = E_;}
      /*! set mass density
      */ 
      void setDensity(double rho_){rho   = rho_;}
      /*! set length of axis
      */ 
      void setLength(double l_){l   = l_;}
      /*! set initial rotational velocity
      */
      void setInitialRotationVelocity(double omega0)  {u0(1) = omega0;}
      /*! \attic set initial rotational velocity, method kept for compatibility
      */ 
      void setInitialRotation(double omega0)  {setInitialRotationVelocity(omega0);}

      /*     void addPort(const string &name, const double &S); */
      /*     using BodyFlexible1s::addPort; */

      Mat computeJacobianMatrix(const ContourPointData &S_); // virtual of body_flexible

      Mat computeWt  (const ContourPointData &S_){return Axis; }
      Vec computeWn  (const ContourPointData &S_){return Vec(3); }
      Vec computeWrOC(const ContourPointData &S_){return WrON00 + S_.alpha(0) * Axis;}
      Vec computeWvC (const ContourPointData &S_){return Vec(3); }


      /*! \return true
      */
      bool hasConstMass() const {return true;}

      /*! NULL-function, since mass matrix is constant*/
      void facLLM() {}

      /* Jacobi Matrix der Rotation */
      void setJR(const Vec &JR_) {assert(JR_.cols()==1); JR = JR_; Axis = JR_;};

      // Lage des Ursprungs des Wellensystems im WKOS 
      // Per Default liegt es im Ursprung des WKOS
      void setWrON00(const Vec &WrON00_) {WrON00 = WrON00_;}

      /** Steifigkeitsmatrix */
      SymMat K;

  };

}

#endif
