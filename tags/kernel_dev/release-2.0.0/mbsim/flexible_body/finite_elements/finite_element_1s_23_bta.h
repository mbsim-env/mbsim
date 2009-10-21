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

#ifndef _FINITE_ELEMENT_1S_21_RCM_H_
#define _FINITE_ELEMENT_1S_21_RCM_H_

#include "fmatvec.h"

namespace MBSim {

  /*! \brief
   * Finite %Element for bending torsional axis
   * */
  class FiniteElement1s23BTA
  {
    protected:

    public:
      double l0, Arho, EIyy, EIzz, Itrho, GIt;
      double depsilon;
      fmatvec::Vec g;

      //                  (double l0, double Arho, double EIyy, double EIzz, double Itrho, double GIt, double g);
      FiniteElement1s23BTA(double   , double     , double     , double     , double      , double    , fmatvec::Vec     );
      //    ~FiniteElement1s23BTA();

      //              (fmatvec::Vec qElement, fmatvec::Vec qpElement);
      int update(fmatvec::Vec &, fmatvec::Vec &);

      void setMaterialDamping(double){}
      void setLehrDamping(double){}

      fmatvec::SymMat M;
      fmatvec::Vec h;

      double gN, gTp;
      fmatvec::Mat WN, WT;
      fmatvec::Vec n, t;

      bool implicit;
      void Implicit(bool implicit_) {implicit = implicit_;}
      fmatvec::SqrMat Dhq, Dhqp;

      fmatvec::SqrMat Damp;

      // Tangent
      fmatvec::Vec Tangent (fmatvec::Vec &q, double &s);

      // Tangent
      fmatvec::SqrMat AWK (fmatvec::Vec &q, double &s);

      // Balkenort
      fmatvec::Vec StateAxis (fmatvec::Vec &q, fmatvec::Vec &v, double &s); // Zustand - global

      // Eingriffsmatrizen
      fmatvec::Mat JGeneralized(fmatvec::Vec &, const double&);

      // Mechanik des Elements mit diesen Koordinaten/Geschwindigkeiten 
      fmatvec::Vec ElementData(fmatvec::Vec qElement, fmatvec::Vec qpElement);

      static const int nodalDOFs;

    private:

      double l0h2, l0h3;/*, l0h4, l0h5, l0h7, l0h8; */

  };

}

#endif
