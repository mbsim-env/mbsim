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

#ifndef _FINITE_ELEMENT_1S_21_ANCF_H_
#define _FINITE_ELEMENT_1S_21_ANCF_H_

#include "fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for planar beam using Absolute Nodal Coordinate Formulation (ANCF)
   * \author Roland Zander
   *
   * model based on
   * SHABANA, A. A.: Computer Implementation of the Absolute Nodal Coordinate Formulation for Flexible Multibody Dynamics. In: Nonlinear Dynamics 16 (1998), S. 293-306
   * SHABANA, A. A.: Definition of the Slopes and the Finite Element Absolute Nodal Coordinate Formulation. In: Nonlinear Dynamics 1 (1997), S. 339-348
   */
  class FiniteElement1s21ANCF
  {
    protected:
      static const int NodeDOFs;
      static const int ElementalDOFs;

    public:
      double l0, Arho, EA, EI;
      double wss0;
      double depsilon;
      fmatvec::Vec g;

      double l0h2;

      FiniteElement1s21ANCF();
      ~FiniteElement1s21ANCF();
      //        (double l0, double Arho, double EA, double EI, double g);
      FiniteElement1s21ANCF(double   , double     , double   , double   , fmatvec::Vec  );

      //              (fmatvec::Vec qElement, fmatvec::Vec qpElement);
      void berechneM();
      void berechneh(fmatvec::Vec qElement, fmatvec::Vec qpElement);

      void setCurleRadius(double){};
      void setMaterialDamping(double){};//, const double&);
      void setLehrDamping(double){};

      fmatvec::SymMat M;
      fmatvec::Vec h;

      bool implicit;
      void Implicit(bool implicit_) {implicit = implicit_;}
      fmatvec::SqrMat Dhq, Dhqp;

      fmatvec::SqrMat Damp;

      // Balkenort
      fmatvec::Vec LocateBalken(fmatvec::Vec&,double&);      // globale Koordinaten

      fmatvec::Vec StateBalken (fmatvec::Vec&,fmatvec::Vec&,double&); // Zustand - global
      //Tangente an s
      fmatvec::Vec Tangente    (fmatvec::Vec&,double&);
      // Eingriffsmatrizen
      fmatvec::Mat JGeneralized(fmatvec::Vec&,const double&);

      // Mechanik des Elements mit diesen Koordinaten/Geschwindigkeiten 
      fmatvec::Vec ElementData(fmatvec::Vec qElement, fmatvec::Vec qpElement) {return fmatvec::Vec(0);}

    private:
      static const double NumPrec   ;// = 1.e-12;
      static const double epsRel   ;// = 1.e-12;
      static const int    Iterations;// = 24;

  };

}

#endif
