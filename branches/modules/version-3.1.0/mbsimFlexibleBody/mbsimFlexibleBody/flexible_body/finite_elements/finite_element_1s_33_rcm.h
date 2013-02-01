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
 */

#ifndef _FINITE_ELEMENT_1S_33_RCM_H_
#define _FINITE_ELEMENT_1S_33_RCM_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contour_pdata.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/weight33RCM.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm.h"
#include "mbsimFlexibleBody/utils/revcardan.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/trafo33RCM.h"
#include "mbsim/utils/eps.h"

#include "fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for spatial beam using Redundant Coordinate Method (RCM)
   * \author Thorsten Schindler
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-07-27 implicit integration (Thorsten Schindler)
   * \todo transform computeState to Position / Velocity / Orientation / AngularVelocity 
   * \todo JacobianOfMotion 
   * \todo computeM 
   */
  class FiniteElement1s33RCM : public MBSim::DiscretizationInterface<fmatvec::Fixed<16> > {
    public:
      /**
       * \brief constructor
       * \param length of finite element
       * \param density
       * \param cross-sectional area
       * \param Young's modulus
       * \param shear modulus
       * \param first area moment of inertia
       * \param second area moment of inertia
       * \param torsional moment of inertia
       * \param acceleration of gravity
       * \param cardan object
       */
      FiniteElement1s33RCM(double l0_,double rho_,double A_,double E_,double G_,double I1_,double I2_,double I0_,const fmatvec::Vec& g_,RevCardanPtr ag_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s33RCM() {}

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */
      virtual const fmatvec::SymMat16& getM() const;
      virtual const fmatvec::Vec16& geth() const;
      virtual const fmatvec::SqrMat16& getdhdq() const;
      virtual const fmatvec::SqrMat16& getdhdu() const;
      virtual int getqSize() const;
      virtual int getuSize() const;

      virtual void computeM(const fmatvec::Vec& qG);
      virtual void computeh(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      virtual void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeKineticEnergy(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qG);
      virtual double computeElasticEnergy(const fmatvec::Vec& qG);

      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data);
      /*****************************************************/

      /* GETTER / SETTER */
      void setGauss(int nGauss);
      void setCurlRadius(double R1,double R2);
      void setMaterialDamping(double epstD_,double k0D_);
      void setLehrDamping(double epstL,double k0L);
      double getl0() const;
      /*****************************************************/

      /**
       * \brief compute state of finite element
       * \param global coordinates
       * \param global velocities
       * \param LAGRANGIAN parameter
       */
      const fmatvec::Vec12& computeState(const fmatvec::Vec& qG,const fmatvec::Vec& qGt,double x);

      /**
       * \brief compute JACOBIAN of contact description in global coordinates
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Mat computeJXqG(const fmatvec::Vec& qG,double x);

      /**
       * \brief compute interesting data 
       * \param global coordinates
       * \param global velocities
       */
      fmatvec::Vec computeData(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);

    private:
      /**
       * \brief length of finite element
       */
      double l0;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief cross sectional area 
       */
      double A;

      /**
       * \brief Young's modulus 
       */
      double E;

      /**
       * \brief shear modulus 
       */
      double G;

      /**
       * \brief geometrical moment of inertia 
       */
      double I1, I2, I0;

      /**
       * \brief gravitation
       */
      fmatvec::Vec g;

      /**
       * \brief predefined bending 
       */
      double k10, k20;

      /**
       * \brief prolongational and torsional damping
       */
      double epstD, k0D;

      /**
       * \brief global system description 
       */
      fmatvec::SymMat16 M;
      fmatvec::Vec16 h;

      /**
       * \brief matrices for implicit integration 
       */
      fmatvec::SqrMat16 dhdq, dhdu;

      /**
       * \brief internal system description 
       */
      fmatvec::SymMat16 MI;

      /**
       * \brief internal damping matrix
       */
      fmatvec::SqrMat16 Damp;

      /**
       * \brief beam length powers 
       */
      double l0h2, l0h3;

      /**
       * \brief last Lagrangian coordinate in state calculation 
       */
      double x_Old;

      /**
       * \brief state at Lagrangian coordinate
       */
      fmatvec::Vec12 X;

      /**
       * \brief global and local state of the last time step 
       */
      fmatvec::Vec16 qG_Old, qGt_Old;

      /**
       * \brief tolerance for comparison of state with old state 
       */
      double tol_comp;

      /**
       * \brief delta matrices
       */
      fmatvec::Mat3x16 drS;
      fmatvec::Mat16x3 drSH;
      fmatvec::RowVec16 depstil, dk0;

      /**
       * \brief reversed Cardan-object 
       */
      RevCardanPtr ag;

      /**
       * \brief trafo-object
       */
      Trafo33RCMPtr tf;

      /**
       * \brief weight-function-object
       */
      Weight33RCMPtr wt;

      /**
       * \brief compute delta matrix for CP with respect to translation
       */
      void computedrS();

      /**
       * \brief compute delta matrix for elongation
       */
      void computedepstil();

      /**
       * \brief compute delta matrix for torsion
       */
      void computedk0();
  };

  inline void FiniteElement1s33RCM::setGauss(int nGauss) { wt->setGauss(nGauss); }
  inline double FiniteElement1s33RCM::getl0() const { return l0; }
  inline const fmatvec::SymMat16& FiniteElement1s33RCM::getM() const { return M; }
  inline const fmatvec::Vec16& FiniteElement1s33RCM::geth() const { return h; }
  inline const fmatvec::SqrMat16& FiniteElement1s33RCM::getdhdq() const { return dhdq; }
  inline const fmatvec::SqrMat16& FiniteElement1s33RCM::getdhdu() const { return dhdu; }
  inline int FiniteElement1s33RCM::getqSize() const { return 16; }
  inline int FiniteElement1s33RCM::getuSize() const { return 16; }
  inline fmatvec::Vec FiniteElement1s33RCM::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33RCM::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat FiniteElement1s33RCM::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33RCM::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s33RCM::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33RCM::computeVelocity): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s33RCM::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33RCM::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElement1s33RCM::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { return computeJXqG(qG,data.getLagrangeParameterPosition()(0)); }

  inline FiniteElement1s33RCM::FiniteElement1s33RCM(double l0_, double rho_, double A_, double E_, double G_, double I1_, double I2_, double I0_, const fmatvec::Vec& g_, RevCardanPtr ag_) :
      l0(l0_), rho(rho_), A(A_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), g(g_), k10(0.), k20(0.), epstD(0.), k0D(0.), M(), h(), dhdq(), dhdu(), Damp(), l0h2(l0 * l0), l0h3(l0h2 * l0), x_Old(-l0), X(), qG_Old(), qGt_Old(), tol_comp(1e-8), drS(), depstil(), dk0(), ag(ag_), tf(new Trafo33RCM(ag, l0)), wt(new Weight33RCM(l0, l0h2, l0h3, tf)) {
    computedrS();
    computedepstil();
    computedk0();
  }

  inline void FiniteElement1s33RCM::setCurlRadius(double R1, double R2) {
    if (fabs(R1) > MBSim::epsroot())
      k10 = 1. / R1;
    if (fabs(R2) > MBSim::epsroot())
      k20 = 1. / R2;

    wt->setCurvature(k10, k20);
  }

  inline void FiniteElement1s33RCM::setMaterialDamping(double epstD_, double k0D_) {
    epstD = epstD_;
    k0D = k0D_;
    Damp(6, 6) += -epstD;
    Damp(15, 15) += -k0D;
  }

  inline void FiniteElement1s33RCM::setLehrDamping(double epstL, double k0L) {
    /* elongation */
    double omgepst = sqrt(12. * E / (rho * l0h2)); // eigenfrequency
    epstD += rho * A * l0h3 * epstL * omgepst / 6.;
    Damp(6, 6) += -epstD;

    /* torsion */
    double omgk0 = sqrt(12. * G / (rho * l0h2)); // eigenfrequency
    k0D += rho * I0 * l0h3 * k0L * omgk0 / 6.;
    Damp(15, 15) += -k0D;
  }

  inline void FiniteElement1s33RCM::computeM(const fmatvec::Vec& qG) {
    if (nrm2(qG - qG_Old) > tol_comp) {
      // /* symmetric mass matrix */
      // fmatvec::Mat Q = tf->gettS()*depstil+(1.+tf->getepstil())*wt->gettSqI();
      // fmatvec::Mat QH = trans(Q);

      // MI = A*static_cast<SymMat>(drSH*(drS*l0+tf->getnS()*wt->getvvt()*wt->getwh1coefqI()+wt->getnSqI()*wt->getIwh1()
      //       + tf->getbS()*wt->getvvt()*wt->getwh2coefqI()+wt->getbSqI()*wt->getIwh2())
      //     + QH*(Q*l0h3/12.+tf->getnS()*wt->getxvvt()*wt->getwh1coefqI()+wt->getnSqI()*wt->getIxwh1()+tf->getbS()*wt->getxvvt()*wt->getwh2coefqI()+wt->getbSqI()*wt->getIxwh2())
      //     + wt->getwh1coefqIH()*(wt->getvvtH()*tf->getnSH()*drS+wt->getxvvtH()*tf->getnSH()*Q+wt->getvvtwwt()*wt->getwh1coefqI()
      //       + wt->getIwh1wwtH()*tf->getnSH()*wt->getnSqI()+wt->getIwh2wwtH()*tf->getnSH()*wt->getbSqI())
      //     + wt->getnSqIH()*(drS*wt->getIwh1()+Q*wt->getIxwh1()+tf->getnS()*wt->getIwh1wwt()*wt->getwh1coefqI()+wt->getnSqI()*wt->getIwh1wh1()
      //       + tf->getbS()*wt->getIwh1wwt()*wt->getwh2coefqI()+wt->getbSqI()*wt->getIwh1wh2())
      //     + wt->getwh2coefqIH()*(wt->getvvtH()*tf->getbSH()*drS+wt->getxvvtH()*tf->getbSH()*Q+wt->getvvtwwt()*wt->getwh2coefqI()
      //       + wt->getIwh1wwtH()*tf->getbSH()*wt->getnSqI()+wt->getIwh2wwtH()*tf->getbSH()*wt->getbSqI())
      //     + wt->getbSqIH()*(drS*wt->getIwh2()+Q*wt->getIxwh2()+tf->getnS()*wt->getIwh2wwt()*wt->getwh1coefqI()+wt->getnSqI()*wt->getIwh1wh2()
      //       + tf->getbS()*wt->getIwh2wwt()*wt->getwh2coefqI()+wt->getbSqI()*wt->getIwh2wh2()));
      // MI += I0*wt->getTtilqItqIt();
      // MI = rho*MI;

      // qG_Old = qG.copy();

      throw MBSim::MBSimError("ERROR (FiniteElement1s33RCM::computeM): Not implemented!");
    }

    /* global description */
    M = JTMJ(MI, tf->getJIG());
  }

  inline void FiniteElement1s33RCM::computeh(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) {
    if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp) {
      wt->computeintD(qG, qGt);

      /* preliminaries for EoM */
      fmatvec::Mat3x16 tSqIH = wt->gettSqI().T();
      fmatvec::Mat3x16 nSqIH = wt->getnSqI().T();
      fmatvec::Mat3x16 bSqIH = wt->getbSqI().T();

      /* gravitational part VgqI */
      fmatvec::RowVec16 VgqI = -rho * A * g.T() * (drS * l0 + tf->getnS() * wt->getvvt() * wt->getwh1coefqI() + wt->getIwh1() * wt->getnSqI() + tf->getbS() * wt->getvvt() * wt->getwh2coefqI() + wt->getIwh2() * wt->getbSqI());

      /* elastic part VeqI */
      double eps = tf->getepstil() + (wt->getIwh1xwh1x() + wt->getIwh2xwh2x()) / (2. * l0);
      fmatvec::RowVec16 epsqI = depstil + (wt->getIwh1xwxwt() * wt->getwh1coefqI() + wt->getIwh2xwxwt() * wt->getwh2coefqI()) / l0;
      fmatvec::RowVecV VeqI = E * A * l0 * eps * epsqI + E * I1 * wt->getIwh1xxwxxwt() * wt->getwh1coefqI() + E * I2 * wt->getIwh2xxwxxwt() * wt->getwh2coefqI() + G * I0 * l0 * tf->getk0() * dk0;

      /* kinetic part TqI */
      fmatvec::Vec S = tf->getepstilt() * tf->gettS() + (1. + tf->getepstil()) * tf->gettSt();
      fmatvec::Mat3x16 P = tf->getepstilt() * wt->gettSqI() + tf->gettSt() * depstil + (1. + tf->getepstil()) * wt->gettStqI();

      fmatvec::RowVec16 TqI = A * (tf->getrSt().T() * (tf->getnS() * wt->getvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1t() + tf->getnSt() * wt->getvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1() + tf->getbS() * wt->getvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2t() + tf->getbSt() * wt->getvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2()) + S.T() * (P * l0h3 / 12. + tf->getnS() * wt->getxvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIxwh1t() + tf->getnSt() * wt->getxvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIxwh1() + tf->getbS() * wt->getxvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIxwh2t() + tf->getbSt() * wt->getxvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIxwh2()) + tf->getnSH() * (P * wt->getIxwh1t() + wt->getnSqI() * wt->getIwh1twh1t() + tf->getnSt() * wt->getIwh1twwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1twh1() + wt->getbSqI() * wt->getIwh1twh2t() + tf->getbSt() * wt->getIwh1twwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh1twh2()) + wt->getIwh1twwt() * wt->getwh1tcoefqI() + tf->getnStH() * (P * wt->getIxwh1() + tf->getnS() * wt->getIwh1wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh1() + tf->getnSt() * wt->getIwh1wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh1wh2t() + tf->getbSt() * wt->getIwh1wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh1wh2()) + tf->getbSH() * (P * wt->getIxwh2t() + wt->getnSqI() * wt->getIwh1twh2t() + tf->getnSt() * wt->getIwh2twwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh2t() + wt->getbSqI() * wt->getIwh2twh2t() + tf->getbSt() * wt->getIwh2twwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2twh2()) + wt->getIwh2twwt() * wt->getwh2tcoefqI() + tf->getbStH() * (P * wt->getIxwh2() + tf->getnS() * wt->getIwh2wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh2() + tf->getnSt() * wt->getIwh2wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2twh2() + tf->getbSt() * wt->getIwh2wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2wh2()));
      TqI += I0 * wt->getTtilqI();
      TqI = rho * TqI;

      /* symmetric mass matrix */
      fmatvec::Mat3x16 Q = tf->gettS() * depstil + (1. + tf->getepstil()) * wt->gettSqI();
      fmatvec::Mat16x3 QH = Q.T();

      MI = A * static_cast<fmatvec::SymMat>(drSH * (drS * l0 + tf->getnS() * wt->getvvt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIwh1() + tf->getbS() * wt->getvvt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIwh2()) + QH * (Q * l0h3 / 12. + tf->getnS() * wt->getxvvt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIxwh1() + tf->getbS() * wt->getxvvt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIxwh2()) + wt->getwh1coefqIH() * (wt->getvvtH() * tf->getnSH() * drS + wt->getxvvtH() * tf->getnSH() * Q + wt->getvvtwwt() * wt->getwh1coefqI() + wt->getIwh1wwtH() * tf->getnSH() * wt->getnSqI() +
          wt->getIwh2wwtH() * tf->getnSH() * wt->getbSqI())
          + wt->getnSqIH() * (drS * wt->getIwh1() + Q * wt->getIxwh1() + tf->getnS() * wt->getIwh1wwt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wwt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIwh1wh2()) + wt->getwh2coefqIH() * (wt->getvvtH() * tf->getbSH() * drS + wt->getxvvtH() * tf->getbSH() * Q + wt->getvvtwwt() * wt->getwh2coefqI() + wt->getIwh1wwtH() * tf->getbSH() * wt->getnSqI() + wt->getIwh2wwtH() * tf->getbSH() * wt->getbSqI()) + wt->getbSqIH() * (drS * wt->getIwh2() + Q * wt->getIxwh2() + tf->getnS() * wt->getIwh2wwt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2wwt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIwh2wh2()));
      MI += I0 * wt->getTtilqItqIt();
      MI = rho * MI;

      /* kinetic part TqItqIqIt */
      fmatvec::RowVec16 qItH = tf->getqIt().T();
      fmatvec::Mat Qnunut = depstil.T() * qItH * tSqIH + tf->getepstilt() * tSqIH + (1. + tf->getepstil()) * wt->getdpSH() * tf->gettSpSt().T();

      fmatvec::Vec TqItqIqIt = A * (drSH * (tf->getnS() * wt->getvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1t() + tf->getnSt() * wt->getvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1() + tf->getbS() * wt->getvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2t() + tf->getbSt() * wt->getvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2()) * tf->getqIt() + QH * (P * l0h3 / 12. + tf->getnS() * wt->getxvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIxwh1t() + tf->getnSt() * wt->getxvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIxwh1() + tf->getbS() * wt->getxvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIxwh2t() + tf->getbSt() * wt->getxvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIxwh2()) * tf->getqIt() + (tf->getnSH() * P * tf->getqIt()) * wt->getwh1coefqIH() * wt->getxvvtH() + (tf->getnSH() * wt->getnSqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh1twwtH() + (tf->getnSH() * tf->getnSt()) * wt->getwh1coefqIH() * wt->getvvtwwt() * wt->getwh1coefqI() * tf->getqIt() + (tf->getnSH() * wt->getnStqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh1wwtH() + (tf->getnSH() * wt->getbSqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh2twwtH() + (tf->getnSH() * tf->getbSt()) * wt->getwh1coefqIH() * wt->getvvtwwt() * wt->getwh2coefqI() * tf->getqIt() + (tf->getnSH() * wt->getbStqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh2wwtH() + wt->getwh1coefqIH() * wt->getvvtwwt() * wt->getwh1tcoefqI() * tf->getqIt() + wt->getnSqIH() * (P * wt->getIxwh1() + tf->getnS() * wt->getIwh1wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh1() + tf->getnSt() * wt->getIwh1wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh1wh2t() + tf->getbSt() * wt->getIwh1wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh1wh2()) * tf->getqIt() + (tf->getbSH() * P * tf->getqIt()) * wt->getwh2coefqIH() * wt->getxvvtH() + (tf->getbSH() * wt->getnSqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh1twwtH() + (tf->getbSH() * tf->getnSt()) * wt->getwh2coefqIH() * wt->getvvtwwt() * wt->getwh1coefqI() * tf->getqIt() + (tf->getbSH() * wt->getnStqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh1wwtH() + (tf->getbSH() * wt->getbSqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh2twwtH() + (tf->getbSH() * tf->getbSt()) * wt->getwh2coefqIH() * wt->getvvtwwt() * wt->getwh2coefqI() * tf->getqIt() + (tf->getbSH() * wt->getbStqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh2wwtH() + wt->getwh2coefqIH() * wt->getvvtwwt() * wt->getwh2tcoefqI() * tf->getqIt() + wt->getbSqIH() * (P * wt->getIxwh2() + tf->getnS() * wt->getIwh2wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh2() + tf->getnSt() * wt->getIwh2wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2twh2() + tf->getbSt() * wt->getIwh2wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2wh2()) * tf->getqIt()

      + Qnunut * (S * l0h3 / 12. + tf->getnS() * wt->getIxwh1t() + tf->getnSt() * wt->getIxwh1() + tf->getbS() * wt->getIxwh2t() + tf->getbSt() * wt->getIxwh2()) + (qItH * wt->getnSqIH() * tf->getrSt()) * wt->getwh1coefqIH() * wt->getvvtH() + (qItH * wt->getnSqIH() * S) * wt->getwh1coefqIH() * wt->getxvvtH() + (qItH * wt->getnSqIH() * tf->getnS()) * wt->getwh1coefqIH() * wt->getIwh1twwtH() + (qItH * wt->getnSqIH() * tf->getnSt()) * wt->getwh1coefqIH() * wt->getIwh1wwtH() + (qItH * wt->getnSqIH() * tf->getbS()) * wt->getwh1coefqIH() * wt->getIwh2twwtH() + (qItH * wt->getnSqIH() * tf->getbSt()) * wt->getwh1coefqIH() * wt->getIwh2wwtH() + wt->getwh1coefqInunutH() * ((tf->getnSH() * tf->getrSt()) * wt->getvvtH() + (tf->getnSH() * S) * wt->getxvvtH() + wt->getIwh1twwtH() + (tf->getnSH() * tf->getnSt()) * wt->getIwh1wwtH() + (tf->getnSH() * tf->getbSt()) * wt->getIwh2wwtH()) + wt->getnSqIH() * (tf->getrSt() * wt->getvvt() + S * wt->getxvvt() + tf->getnS() * wt->getIwh1twwt() + tf->getnSt() * wt->getIwh1wwt() + tf->getbS() * wt->getIwh2twwt() + tf->getbSt() * wt->getIwh2wwt()) * wt->getwh1coefqI() * tf->getqIt() + wt->getdpSH() * tf->getnSpSt().T() * (tf->getrSt() * wt->getIwh1() + S * wt->getIxwh1() + tf->getnS() * wt->getIwh1twh1() + tf->getnSt() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wh2t() + tf->getbSt() * wt->getIwh1wh2()) + (qItH * wt->getbSqIH() * tf->getrSt()) * wt->getwh2coefqIH() * wt->getvvtH() + (qItH * wt->getbSqIH() * S) * wt->getwh2coefqIH() * wt->getxvvtH() + (qItH * wt->getbSqIH() * tf->getnS()) * wt->getwh2coefqIH() * wt->getIwh1twwtH() + (qItH * wt->getbSqIH() * tf->getnSt()) * wt->getwh2coefqIH() * wt->getIwh1wwtH() + (qItH * wt->getbSqIH() * tf->getbS()) * wt->getwh2coefqIH() * wt->getIwh2twwtH() + (qItH * wt->getbSqIH() * tf->getbSt()) * wt->getwh2coefqIH() * wt->getIwh2wwtH() + wt->getwh2coefqInunutH() * ((tf->getbSH() * tf->getrSt()) * wt->getvvtH() + (tf->getbSH() * S) * wt->getxvvtH() + (tf->getbSH() * tf->getnSt()) * wt->getIwh1wwtH() + wt->getIwh2twwtH() + (tf->getbSH() * tf->getbSt()) * wt->getIwh2wwtH()) + wt->getbSqIH() * (tf->getrSt() * wt->getvvt() + S * wt->getxvvt() + tf->getnS() * wt->getIwh1twwt() + tf->getnSt() * wt->getIwh1wwt() + tf->getbS() * wt->getIwh2twwt() + tf->getbSt() * wt->getIwh2wwt()) * wt->getwh2coefqI() * tf->getqIt() + wt->getdpSH() * tf->getbSpSt().T() * (tf->getrSt() * wt->getIwh2() + S * wt->getIxwh2() + tf->getnS() * wt->getIwh1twh2() + tf->getnSt() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2twh2() + tf->getbSt() * wt->getIwh2wh2()));
      TqItqIqIt += I0 * wt->getTtilqItqIqIt();
      TqItqIqIt = rho * TqItqIqIt;

      /* summarizing RHS */
      fmatvec::Vec hIZ = (TqI - VeqI - VgqI).T() - TqItqIqIt;
      hIZ(6) -= epstD * tf->getepstilt();
      hIZ(15) -= k0D * tf->getk0t();
      hIZ -= MI * tf->getJIGt() * qGt;

      /* global description */
      h = tf->getJIG().T() * hIZ;

      qG_Old = qG.copy();
      qGt_Old = qGt.copy();
    }
  }

  inline void FiniteElement1s33RCM::computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) {
    fmatvec::Vec16 h0 = h;

    fmatvec::Vec16 qG_tmp = qG;
    fmatvec::Vec16 qGt_tmp = qGt;

    /**************** velocity dependent calculations ********************/
    for (int i = 0; i < qGt.size(); i++) {
      double qGti = qGt_tmp(i); // save correct position

      qGt_tmp(i) += MBSim::epsroot(); // update with disturbed positions assuming same active links
      computeh(qG_tmp, qGt_tmp);

      dhdu.set(i, (h - h0) / MBSim::epsroot());
      qGt_tmp(i) = qGti;
    }

    /***************** position dependent calculations ********************/
    for (int i = 0; i < qG.size(); i++) {
      double qGi = qG_tmp(i); // save correct position

      qG_tmp(i) += MBSim::epsroot(); // update with disturbed positions assuming same active links
      computeh(qG_tmp, qGt_tmp);

      dhdq.set(i, (h - h0) / MBSim::epsroot());
      qG_tmp(i) = qGi;
    }

    /******************* back to initial state **********************/
    computeh(qG, qGt);
  }

  double FiniteElement1s33RCM::computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) {
    if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp)
      wt->computeint(qG, qGt);
    fmatvec::Vec S = tf->getepstilt() * tf->gettS() + (1. + tf->getepstil()) * tf->gettSt();

    return 0.5 * rho * (A * (tf->getrSt().T() * (tf->getrSt() * l0 + 2. * tf->getnS() * wt->getIwh1t() + 2. * tf->getnSt() * wt->getIwh1() + 2. * tf->getbS() * wt->getIwh2t() + 2. * tf->getbSt() * wt->getIwh2()) + S.T() * (S * l0h3 / 12. + 2. * tf->getnS() * wt->getIxwh1t() + 2. * tf->getnSt() * wt->getIxwh1() + 2. * tf->getbS() * wt->getIxwh2t() + 2. * tf->getbSt() * wt->getIxwh2()) + tf->getnSH() * (2. * tf->getnSt() * wt->getIwh1twh1() + 2. * tf->getbSt() * wt->getIwh1twh2()) + wt->getIwh1twh1t() + tf->getnStH() * (tf->getnSt() * wt->getIwh1wh1() + 2. * tf->getbS() * wt->getIwh1wh2t() + 2. * tf->getbSt() * wt->getIwh1wh2()) + tf->getbSH() * (2. * tf->getbSt() * wt->getIwh2twh2()) + wt->getIwh2twh2t() + tf->getbStH() * (tf->getbSt() * wt->getIwh2wh2())) + I0 * wt->getTtil());
  }

  double FiniteElement1s33RCM::computeGravitationalEnergy(const fmatvec::Vec& qG) {
    if (nrm2(qG - qG_Old) > tol_comp)
      wt->computewhcoefPos(qG);

    double Iwh1 = wt->intv(wt->getwh1coef());
    double Iwh2 = wt->intv(wt->getwh2coef());

    return -rho * A * g.T() * (l0 * tf->getrS() + Iwh1 * tf->getnS() + Iwh2 * tf->getbS());
  }

  double FiniteElement1s33RCM::computeElasticEnergy(const fmatvec::Vec& qG) {
    if (nrm2(qG - qG_Old) > tol_comp)
      wt->computewhcoefPos(qG);

    double Iwh1xwh1x = wt->intvw(wt->getwh1coef(), wt->getwh1coef());
    double Iwh2xwh2x = wt->intvw(wt->getwh2coef(), wt->getwh2coef());
    double Iwh1xxwh1xx = wt->intvxxvxx(wt->getwh1coef(), k10);
    double Iwh2xxwh2xx = wt->intvxxvxx(wt->getwh2coef(), k20);
    double eps = tf->getepstil() + (Iwh1xwh1x + Iwh2xwh2x) / (2. * l0);

    return 0.5 * (E * A * eps * eps * l0 + E * I1 * Iwh1xxwh1xx + E * I2 * Iwh2xxwh2xx + G * I0 * l0 * tf->getk0() * tf->getk0());
  }

  fmatvec::Mat FiniteElement1s33RCM::computeJXqG(const fmatvec::Vec& qG, double x) {
    fmatvec::Mat JXqG(16, 6);

    if (nrm2(qG - qG_Old) > tol_comp)
      wt->computewhcoefPosD(qG);

    fmatvec::RowVec wwt(4);
    wwt(3) = x * x;
    wwt(2) = wwt(3) * x;
    wwt(1) = wwt(2) * x;
    wwt(0) = wwt(1) * x;

    fmatvec::RowVec16 wh1qI = wwt * wt->getwh1coefqI();
    fmatvec::RowVec16 wh2qI = wwt * wt->getwh2coefqI();

    fmatvec::Vec wh1 = wt->computew(wt->getwh1coef(), x);
    fmatvec::Vec wh2 = wt->computew(wt->getwh2coef(), x);

    JXqG(0, 0, 15, 2) = (drS + (tf->gettS() * depstil + (1. + tf->getepstil()) * wt->gettSqI()) * x + tf->getnS() * wh1qI + wh1(0) * wt->getnSqI() + tf->getbS() * wh2qI + wh2(0) * wt->getbSqI()).T(); /* JT */

    fmatvec::Vec w1 = wt->computew(wt->getw1coef(), x);
    fmatvec::Vec w2 = wt->computew(wt->getw2coef(), x);

    fmatvec::Vec p = tf->getpS();
    p(0) += sin((tf->getpS())(1)) * w2(1) + tf->getk0() * x;
    p(1) += w1(1);
    p(2) += w2(1);

    fmatvec::Vec t = ag->computet(p);
    fmatvec::Vec n = ag->computen(p);
    fmatvec::Vec b = ag->computeb(p);

    fmatvec::SqrMat tp = ag->computetq(p);
    fmatvec::SqrMat np = ag->computenq(p);
    fmatvec::SqrMat bp = ag->computebq(p);

    fmatvec::RowVec dwxdwt(4);
    dwxdwt(3) = 2 * x;
    dwxdwt(2) = wwt(3) * 3.;
    dwxdwt(1) = wwt(2) * 4.;
    dwxdwt(0) = wwt(1) * 5.;

    fmatvec::Mat Z(3, 16);
    Z(0, 0, 0, 15) = sin((tf->getpS())(1)) * dwxdwt * wt->getw2coefqI() + x * dk0;
    Z(0, 4) += cos((tf->getpS())(1)) * w2(1);
    Z(1, 0, 1, 15) = dwxdwt * wt->getw1coefqI();
    Z(2, 0, 2, 15) = dwxdwt * wt->getw2coefqI();

    fmatvec::Mat3x16 ptqIt = wt->getdpS() + Z;
    fmatvec::MatV16 ttqIt = tp * ptqIt;
    fmatvec::MatV16 ntqIt = np * ptqIt;
    fmatvec::MatV16 btqIt = bp * ptqIt;

    JXqG(0, 3, 15, 3) = (t(1) * ttqIt(fmatvec::Index(2,2), fmatvec::Index(0,15)) + n(1) * ntqIt(fmatvec::Index(2,2), fmatvec::Index(0,15)) + b(1) * btqIt(fmatvec::Index(2,2), fmatvec::Index(0,15))).T();
    JXqG(0, 4, 15, 4) = (t(2) * ttqIt(fmatvec::Index(0,0), fmatvec::Index(0,15)) + n(2) * ntqIt(fmatvec::Index(0,0), fmatvec::Index(0,15)) + b(2) * btqIt(fmatvec::Index(0,0), fmatvec::Index(0,15))).T();
    JXqG(0, 5, 15, 5) = (t(0) * ttqIt(fmatvec::Index(1,1), fmatvec::Index(0,15)) + n(0) * ntqIt(fmatvec::Index(1,1), fmatvec::Index(0,15)) + b(0) * btqIt(fmatvec::Index(1,1), fmatvec::Index(0,15))).T(); /* JR */

    JXqG = tf->getJIG().T() * JXqG; /* transformation global coordinates */

    return JXqG.copy();
  }

  const fmatvec::Vec12& FiniteElement1s33RCM::computeState(const fmatvec::Vec& qG, const fmatvec::Vec& qGt, double x) {
    if (fmatvec::nrm2(qG - qG_Old) < tol_comp && nrm2(qGt - qGt_Old) < tol_comp && fabs(x - x_Old) < MBSim::epsroot())
      return X;
    else {
      if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp)
        wt->computewhcoefVel(qG, qGt);

      fmatvec::Vec wh1 = wt->computew(wt->getwh1coef(), x); // position and velocity
      fmatvec::Vec wh2 = wt->computew(wt->getwh2coef(), x); // position and velocity
      fmatvec::Vec wh1t = wt->computew(wt->getwh1tcoef(), x); // only velocity
      fmatvec::Vec wh2t = wt->computew(wt->getwh2tcoef(), x); // only velocity

      fmatvec::Vec w1 = wt->computew(wt->getw1coef(), x); // only position
      fmatvec::Vec w2 = wt->computew(wt->getw2coef(), x); // position and velocity
      fmatvec::Vec w1t = wt->computew(wt->getw1tcoef(), x); // only velocity
      fmatvec::Vec w2t = wt->computew(wt->getw2tcoef(), x); // only velocity

      X.set(fmatvec::Index(0, 2), tf->getrS() + (1. + tf->getepstil()) * x * tf->gettS() + wh1(0) * tf->getnS() + wh2(0) * tf->getbS()); /* pos */
      X.set(fmatvec::Index(3, 5), tf->getpS());
      X(3) += sin((tf->getpS())(1)) * w2(1) + tf->getk0() * x;
      X(4) += w1(1);
      X(5) += w2(1);/* angles */
      X.set(fmatvec::Index(6, 8), tf->getrSt() + (tf->getepstilt() * tf->gettS() + (1. + tf->getepstil()) * tf->gettSt()) * x + wh1t(0) * tf->getnS() + wh1(0) * tf->getnSt() + wh2t(0) * tf->getbS() + wh2(0) * tf->getbSt()); /* vel */
      X.set(fmatvec::Index(9, 11), tf->getpSt());
      X(9) += cos((tf->getpS())(1)) * (tf->getpSt())(1) * w2(1) + sin((tf->getpS())(1)) * w2t(1) + tf->getk0t() * x;
      X(10) += w1t(1);
      X(11) += w2t(1); /* time differentiated angles */

      return X;
    }
  }

  fmatvec::Vec FiniteElement1s33RCM::computeData(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) {
    if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp)
      tf->computezI(qG, qGt);

    fmatvec::Vec Data(16);
    Data(0, 2) = tf->getrS(); /* rS */
    Data(3, 5) = qG(13, 15) - qG(3, 5); /* dp */
    Data(6) = tf->getk0(); /* k0 */
    Data(7) = tf->getepstil(); /* epstil */
    Data(8, 10) = tf->getrSt(); /* rSt */
    Data(11, 13) = qGt(13, 15) - qGt(3, 5); /* dpt */
    Data(14) = tf->getk0t(); /* k0t */
    Data(15) = tf->getepstilt(); /* epstilt */

    return Data.copy();
  }

  inline void FiniteElement1s33RCM::computedrS() {
    drS(0, 0) = 1.;
    drS(1, 1) = 1.;
    drS(2, 2) = 1.;
    drSH = drS.T();
  }

  inline void FiniteElement1s33RCM::computedepstil() {
    depstil(6) = 1.;
  }

  inline void FiniteElement1s33RCM::computedk0() {
    dk0(15) = 1.;
  }

}

#endif /* _FINITE_ELEMENT_1S_33_RCM_H_ */

