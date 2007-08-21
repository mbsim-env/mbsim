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
using namespace fmatvec;

namespace MBSim {

  /*! \brief
   * Finite %Element for 2D-Beam using Redundant Coordinate Method(RCM) 
   * 
   * Roland Zander, Lehrstuhl fuer Angewandte Mechanik, TU Muenchen\par
   *
   * read:\n
   * Zander, R.; Ulbrich, H.: Reference-free mixed FE-MBS approach for beam structures withconstraints, Journal of Nonlinear Dynamics, Kluwer Academic Publishers,2005 \n
   * Zander, R.; Ulbrich, H.: Impacts on beam structures: Interaction of wave propagationand global dynamics, IUTAM Symposium on Multiscale Problems inMultibody System Contacts Stuttgart, Germany, 2006  \n
   * Zander, R.; Ulbrich, H.: Free plain motion of flexible beams in MBS - A comparison ofmodels, III European Conference on Comutational Mechanics Lissbon,Portugal, 2006  
   *
   * \see BodyFlexible1s21RCM
   */
  class FiniteElement1s21RCM
  {
    protected:

    public:
      double l0, Arho, EA, EI;
      double wss0;
      double depsilon;
      Vec g;

      FiniteElement1s21RCM(){};
      //    ~FiniteElement1s21RCM();
      //        (double l0, double Arho, double EA, double EI, double g);
      /*! full quallified constructor 
       * \param l0_   undeformed lenght of %element
       * \param Arho_ line-density \f$\rho_l=A\,\rho\f$ of beam
       * \param EA_   longitudinal stiffness \f$E\,A\f$
       * \param EI_   bending stiffness \f$E\,I\f$
       * \param g_    vector of gravitational acceleration
       */
      FiniteElement1s21RCM(double l0_, double  Arho_, double EA_, double EI_, Vec g_);

      /*! update mass-matrix and vector of generalised force directions
       * \param qElement  generalised positions of element
       * \param qpElement generalised velocities of element
       */
      int berechne(Vec qElement, Vec qpElement);

      void setCurleRadius(double);
      void setMaterialDamping(double);//, const double&);
      void setLehrDamping(double);

      SymMat M;
      Vec h;

      bool implicit;
      void Implicit(bool implicit_) {implicit = implicit_;}
      SqrMat Dhq, Dhqp;

      SqrMat Damp;

      // Balkenort
      Vec LocateBalken(Vec&,double&);      // globale Koordinaten
      Vec StateBalken (Vec&,Vec&,double&); // Zustand - global

      // Eingriffsmatrizen
      Mat JGeneralizedInternal(Vec&,const double&);
      Mat JGeneralized (Vec&,const double&);
      Mat JpGeneralized(Vec&,Vec&,const double&,const double&);

      // Mechanik des Elements mit diesen Koordinaten/Geschwindigkeiten 
      Vec ElementData(Vec qElement, Vec qpElement);

      //kinetische Energie berechenen
      double computeT(Vec qElement, Vec qpElement);

      //potentielle Energie berechenen
      double computeV(Vec qElement);

    private:
      //              (qGlobal,->qIntern)
      void BuildqLokal(Vec&,Vec&);
      //              (qIntern,->Jeg)
      void BuildJacobi(Vec&,SqrMat&);
      //              (qIntern,qpIntern,->Jeg,->Jegp)
      void BuildJacobi(Vec&,Vec&,SqrMat&,SqrMat&);

      // Balkenort
      Vec LocateLokalBalken(Vec&,Vec&,double&,bool);
      Vec LocateLokalBalken(Vec&,Vec&,double&);

      // Jacobis für impliziete Integratoren
      Mat hFullJacobi(Vec&,Vec&,Vec&,Vec&,SqrMat&,SqrMat&,SymMat&,Vec&);

      double l0h2, l0h3, l0h4, l0h5, l0h7, l0h8;

  };

}

#endif
