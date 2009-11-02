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
#include "discretization_interface.h"
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
  class FiniteElement1s21RCM : public DiscretizationInterface
  {
    protected:
      double l0, Arho, EA, EI;
      double wss0;
      double depsilon;
      Vec g;
      SymMat M;
      Vec h;

    public:
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

	  fmatvec::SymMat getMassMatrix() const {return M;}
	  fmatvec::Vec    getGeneralizedForceVector() const {return h;}

	  fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingPosition() const {return Dhq;}    
	  fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingVelocity() const {return Dhqp;}
	  int getSizeOfPositions() const {return 8;}
	  int getSizeOfVelocities() const {return 8;}

      /*! update mass-matrix and vector of generalised force directions
       * \param qElement  generalised positions of element
       * \param qpElement generalised velocities of element
       */
      void computeEquationsOfMotion(const Vec& qElement,const Vec& qpElement);

      void setCurleRadius(double);
      void setMaterialDamping(double);//, const double&);
      void setLehrDamping(double);

      bool implicit;
      void Implicit(bool implicit_) {implicit = implicit_;}
      SqrMat Dhq, Dhqp;

      SqrMat Damp;

      // Balkenort
      Vec LocateBalken (const Vec&q, const double &s);      // globale Koordinaten
      Vec StateBalken  (const Vec&q, const Vec&u, const double &s); // Zustand - global
// TODO TODO TODO TODO: die drei sind nur pro-forma implementier, um lauffaehigkeit zu
// erreichen; das MUSS angepasst werden, gleichzeitig management von Jacobis JT und JR
// des gesamten BodyFlexible...
	  Vec computeTranslation(const Vec&q, const ContourPointData& cp) {return LocateBalken(q,cp.alpha(0));}
	  SqrMat computeOrientation(const Vec&q, const ContourPointData& cp) {return SqrMat(0,INIT,0.);}
	  Vec computeTranslationalVelocity (const Vec&q, const Vec&u, const ContourPointData& cp) {return LocateBalken(q,cp.alpha(0));}
	  Vec computeAngularVelocity(const Vec&q, const Vec&u, const ContourPointData& cp) {return LocateBalken(q,cp.alpha(0));}

      // Eingriffsmatrizen
      Mat JGeneralizedInternal(const Vec&,const double&);
      Mat JGeneralized (const Vec&,const double&);
	  Mat computeJacobianOfMinimalRepresentationRegardingPhysics(const Vec&q,const ContourPointData& cp) {return JGeneralized(q,cp.alpha(0));}
      Mat JpGeneralized(const Vec&,const Vec&,const double&,const double&);

      Vec DrDs (Vec&,const double&);
      Vec DrDsp(Vec&,Vec&,const double&,const double&);
      double Kcurvature (Vec&,const double&);
      double Kpcurvature(Vec&,Vec&,const double&,const double&);
     
      // Mechanik des Elements mit diesen Koordinaten/Geschwindigkeiten 
      Vec ElementData(Vec qElement, Vec qpElement);

      //kinetische Energie berechenen
      double computeKineticEnergy(const Vec& qElement, const Vec& qpElement);

      //potentielle Energie berechenen
      double computeV(const Vec& qElement);
      double computeElasticEnergy(const Vec& qElement);
      double computeGravitationalEnergy(const Vec& qElement);

    private:
      //              (qGlobal,->qIntern)
      void BuildqLokal(const Vec&,Vec&);
      //              (qIntern,->Jeg)
      void BuildJacobi(const Vec&,SqrMat&);
      //              (qIntern,qpIntern,->Jeg,->Jegp)
      void BuildJacobi(const Vec&,const Vec&,SqrMat&,SqrMat&);

      // Balkenort
      Vec LocateLokalBalken(const Vec& qLokal, const Vec& qpLokal, const double& s, const bool calcAll=true);

      // Jacobis für impliziete Integratoren
      Mat hFullJacobi(const Vec&,const Vec&,const Vec&,const Vec&,const SqrMat&,const SqrMat&,const SymMat&,const Vec&);

      double l0h2, l0h3, l0h4, l0h5, l0h7, l0h8;

  };

}

#endif
