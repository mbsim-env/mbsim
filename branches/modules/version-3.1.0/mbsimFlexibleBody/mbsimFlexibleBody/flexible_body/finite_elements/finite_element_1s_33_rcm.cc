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

#include<config.h>
#include "finite_element_1s_33_rcm.h"


using namespace std;
using namespace fmatvec;

namespace MBSimFlexibleBody {
  FiniteElement1s33RCM::FiniteElement1s33RCM(double l0_, double rho_, double A_, double E_, double G_, double I1_, double I2_, double I0_, const Vec& g_, RevCardanPtr ag_) :
      l0(l0_), rho(rho_), A(A_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), g(g_), k10(0.), k20(0.), epstD(0.), k0D(0.), M(), h(), dhdq(), dhdu(), Damp(), l0h2(l0 * l0), l0h3(l0h2 * l0), x_Old(-l0), X(), qG_Old(), qGt_Old(), tol_comp(1e-8), drS(), depstil(), dk0(), ag(ag_), tf(new Trafo33RCM(ag, l0)), wt(new Weight33RCM(l0, l0h2, l0h3, tf)) {
    computedrS();
    computedepstil();
    computedk0();
  }

  void FiniteElement1s33RCM::setCurlRadius(double R1, double R2) {
    if (fabs(R1) > MBSim::epsroot())
      k10 = 1. / R1;
    if (fabs(R2) > MBSim::epsroot())
      k20 = 1. / R2;

    wt->setCurvature(k10, k20);
  }

  void FiniteElement1s33RCM::setMaterialDamping(double epstD_, double k0D_) {
    epstD = epstD_;
    k0D = k0D_;
    Damp(6, 6) += -epstD;
    Damp(15, 15) += -k0D;
  }

  void FiniteElement1s33RCM::setLehrDamping(double epstL, double k0L) {
    /* elongation */
    double omgepst = sqrt(12. * E / (rho * l0h2)); // eigenfrequency
    epstD += rho * A * l0h3 * epstL * omgepst / 6.;
    Damp(6, 6) += -epstD;

    /* torsion */
    double omgk0 = sqrt(12. * G / (rho * l0h2)); // eigenfrequency
    k0D += rho * I0 * l0h3 * k0L * omgk0 / 6.;
    Damp(15, 15) += -k0D;
  }

  void FiniteElement1s33RCM::computeM(const Vec16& qG) {
    if (nrm2(qG - qG_Old) > tol_comp) {
      // /* symmetric mass matrix */
      // Mat Q = tf->gettS()*depstil+(1.+tf->getepstil())*wt->gettSqI();
      // Mat QH = trans(Q);

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

  void FiniteElement1s33RCM::computeh(const Vec16& qG, const Vec16& qGt) {
    if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp) {
      wt->computeintD(qG, qGt);

      /* preliminaries for EoM */
      Mat16x3 tSqIH = wt->gettSqI().T();
      //Mat16x3 nSqIH = wt->getnSqI().T(); //TODO: why not error in Ref-case?
      //Mat16x3 bSqIH = wt->getbSqI().T();

      /* gravitational part VgqI */
      RowVec16 VgqI = -rho * A * g.T() * (drS * l0 + tf->getnS() * wt->getvvt() * wt->getwh1coefqI() + wt->getIwh1() * wt->getnSqI() + tf->getbS() * wt->getvvt() * wt->getwh2coefqI() + wt->getIwh2() * wt->getbSqI());

      /* elastic part VeqI */
      double eps = tf->getepstil() + (wt->getIwh1xwh1x() + wt->getIwh2xwh2x()) / (2. * l0);
      RowVec16 epsqI = depstil + (wt->getIwh1xwxwt() * wt->getwh1coefqI() + wt->getIwh2xwxwt() * wt->getwh2coefqI()) / l0;
      RowVecV VeqI = E * A * l0 * eps * epsqI + E * I1 * wt->getIwh1xxwxxwt() * wt->getwh1coefqI() + E * I2 * wt->getIwh2xxwxxwt() * wt->getwh2coefqI() + G * I0 * l0 * tf->getk0() * dk0;

      /* kinetic part TqI */
      Vec3 S = tf->getepstilt() * tf->gettS() + (1. + tf->getepstil()) * tf->gettSt();
      Mat3x16 P = tf->getepstilt() * wt->gettSqI() + tf->gettSt() * depstil + (1. + tf->getepstil()) * wt->gettStqI();

      RowVec16 TqI = A * (tf->getrSt().T() * (tf->getnS() * wt->getvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1t() + tf->getnSt() * wt->getvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1() + tf->getbS() * wt->getvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2t() + tf->getbSt() * wt->getvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2()) + S.T() * (P * l0h3 / 12. + tf->getnS() * wt->getxvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIxwh1t() + tf->getnSt() * wt->getxvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIxwh1() + tf->getbS() * wt->getxvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIxwh2t() + tf->getbSt() * wt->getxvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIxwh2()) + tf->getnSH() * (P * wt->getIxwh1t() + wt->getnSqI() * wt->getIwh1twh1t() + tf->getnSt() * wt->getIwh1twwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1twh1() + wt->getbSqI() * wt->getIwh1twh2t() + tf->getbSt() * wt->getIwh1twwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh1twh2()) + wt->getIwh1twwt() * wt->getwh1tcoefqI() + tf->getnStH() * (P * wt->getIxwh1() + tf->getnS() * wt->getIwh1wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh1() + tf->getnSt() * wt->getIwh1wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh1wh2t() + tf->getbSt() * wt->getIwh1wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh1wh2()) + tf->getbSH() * (P * wt->getIxwh2t() + wt->getnSqI() * wt->getIwh1twh2t() + tf->getnSt() * wt->getIwh2twwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh2t() + wt->getbSqI() * wt->getIwh2twh2t() + tf->getbSt() * wt->getIwh2twwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2twh2()) + wt->getIwh2twwt() * wt->getwh2tcoefqI() + tf->getbStH() * (P * wt->getIxwh2() + tf->getnS() * wt->getIwh2wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh2() + tf->getnSt() * wt->getIwh2wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2twh2() + tf->getbSt() * wt->getIwh2wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2wh2()));
      TqI += I0 * wt->getTtilqI();
      TqI = rho * TqI;

      /* symmetric mass matrix */
      Mat3x16 Q = tf->gettS() * depstil + (1. + tf->getepstil()) * wt->gettSqI();
      Mat16x3 QH = Q.T();

      MI = static_cast<SymMat16>(A * (drSH * (drS * l0 + tf->getnS() * wt->getvvt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIwh1() + tf->getbS() * wt->getvvt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIwh2()) + QH * (Q * l0h3 / 12. + tf->getnS() * wt->getxvvt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIxwh1() + tf->getbS() * wt->getxvvt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIxwh2()) + wt->getwh1coefqIH() * (wt->getvvtH() * tf->getnSH() * drS + wt->getxvvtH() * tf->getnSH() * Q + wt->getvvtwwt() * wt->getwh1coefqI() + wt->getIwh1wwtH() * tf->getnSH() * wt->getnSqI() + wt->getIwh2wwtH() * tf->getnSH() * wt->getbSqI()) + wt->getnSqIH() * (drS * wt->getIwh1() + Q * wt->getIxwh1() + tf->getnS() * wt->getIwh1wwt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wwt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIwh1wh2()) + wt->getwh2coefqIH() * (wt->getvvtH() * tf->getbSH() * drS + wt->getxvvtH() * tf->getbSH() * Q + wt->getvvtwwt() * wt->getwh2coefqI() + wt->getIwh1wwtH() * tf->getbSH() * wt->getnSqI() + wt->getIwh2wwtH() * tf->getbSH() * wt->getbSqI()) + wt->getbSqIH() * (drS * wt->getIwh2() + Q * wt->getIxwh2() + tf->getnS() * wt->getIwh2wwt() * wt->getwh1coefqI() + wt->getnSqI() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2wwt() * wt->getwh2coefqI() + wt->getbSqI() * wt->getIwh2wh2())));
      MI += I0 * wt->getTtilqItqIt();
      MI = rho * MI;

      /* kinetic part TqItqIqIt */
      RowVec16 qItH = tf->getqIt().T();
      Mat16x3 Qnunut = depstil.T() * qItH * tSqIH + tf->getepstilt() * tSqIH + (1. + tf->getepstil()) * wt->getdpSH() * tf->gettSpSt().T();

      Vec16 TqItqIqIt = A * (drSH * (tf->getnS() * wt->getvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1t() + tf->getnSt() * wt->getvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1() + tf->getbS() * wt->getvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2t() + tf->getbSt() * wt->getvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2()) * tf->getqIt() + QH * (P * l0h3 / 12. + tf->getnS() * wt->getxvvt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIxwh1t() + tf->getnSt() * wt->getxvvt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIxwh1() + tf->getbS() * wt->getxvvt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIxwh2t() + tf->getbSt() * wt->getxvvt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIxwh2()) * tf->getqIt() + (tf->getnSH() * P * tf->getqIt()) * wt->getwh1coefqIH() * wt->getxvvtH() + (tf->getnSH() * wt->getnSqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh1twwtH() + (tf->getnSH() * tf->getnSt()) * wt->getwh1coefqIH() * wt->getvvtwwt() * wt->getwh1coefqI() * tf->getqIt() + (tf->getnSH() * wt->getnStqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh1wwtH() + (tf->getnSH() * wt->getbSqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh2twwtH() + (tf->getnSH() * tf->getbSt()) * wt->getwh1coefqIH() * wt->getvvtwwt() * wt->getwh2coefqI() * tf->getqIt() + (tf->getnSH() * wt->getbStqI() * tf->getqIt()) * wt->getwh1coefqIH() * wt->getIwh2wwtH() + wt->getwh1coefqIH() * wt->getvvtwwt() * wt->getwh1tcoefqI() * tf->getqIt() + wt->getnSqIH() * (P * wt->getIxwh1() + tf->getnS() * wt->getIwh1wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh1() + tf->getnSt() * wt->getIwh1wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh1wh2t() + tf->getbSt() * wt->getIwh1wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh1wh2()) * tf->getqIt() + (tf->getbSH() * P * tf->getqIt()) * wt->getwh2coefqIH() * wt->getxvvtH() + (tf->getbSH() * wt->getnSqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh1twwtH() + (tf->getbSH() * tf->getnSt()) * wt->getwh2coefqIH() * wt->getvvtwwt() * wt->getwh1coefqI() * tf->getqIt() + (tf->getbSH() * wt->getnStqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh1wwtH() + (tf->getbSH() * wt->getbSqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh2twwtH() + (tf->getbSH() * tf->getbSt()) * wt->getwh2coefqIH() * wt->getvvtwwt() * wt->getwh2coefqI() * tf->getqIt() + (tf->getbSH() * wt->getbStqI() * tf->getqIt()) * wt->getwh2coefqIH() * wt->getIwh2wwtH() + wt->getwh2coefqIH() * wt->getvvtwwt() * wt->getwh2tcoefqI() * tf->getqIt() + wt->getbSqIH() * (P * wt->getIxwh2() + tf->getnS() * wt->getIwh2wwt() * wt->getwh1tcoefqI() + wt->getnSqI() * wt->getIwh1twh2() + tf->getnSt() * wt->getIwh2wwt() * wt->getwh1coefqI() + wt->getnStqI() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2wwt() * wt->getwh2tcoefqI() + wt->getbSqI() * wt->getIwh2twh2() + tf->getbSt() * wt->getIwh2wwt() * wt->getwh2coefqI() + wt->getbStqI() * wt->getIwh2wh2()) * tf->getqIt() + Qnunut * (S * l0h3 / 12. + tf->getnS() * wt->getIxwh1t() + tf->getnSt() * wt->getIxwh1() + tf->getbS() * wt->getIxwh2t() + tf->getbSt() * wt->getIxwh2()) + (qItH * wt->getnSqIH() * tf->getrSt()) * wt->getwh1coefqIH() * wt->getvvtH() + (qItH * wt->getnSqIH() * S) * wt->getwh1coefqIH() * wt->getxvvtH() + (qItH * wt->getnSqIH() * tf->getnS()) * wt->getwh1coefqIH() * wt->getIwh1twwtH() + (qItH * wt->getnSqIH() * tf->getnSt()) * wt->getwh1coefqIH() * wt->getIwh1wwtH() + (qItH * wt->getnSqIH() * tf->getbS()) * wt->getwh1coefqIH() * wt->getIwh2twwtH() + (qItH * wt->getnSqIH() * tf->getbSt()) * wt->getwh1coefqIH() * wt->getIwh2wwtH() + wt->getwh1coefqInunutH() * ((tf->getnSH() * tf->getrSt()) * wt->getvvtH() + (tf->getnSH() * S) * wt->getxvvtH() + wt->getIwh1twwtH() + (tf->getnSH() * tf->getnSt()) * wt->getIwh1wwtH() + (tf->getnSH() * tf->getbSt()) * wt->getIwh2wwtH()) + wt->getnSqIH() * (tf->getrSt() * wt->getvvt() + S * wt->getxvvt() + tf->getnS() * wt->getIwh1twwt() + tf->getnSt() * wt->getIwh1wwt() + tf->getbS() * wt->getIwh2twwt() + tf->getbSt() * wt->getIwh2wwt()) * wt->getwh1coefqI() * tf->getqIt() + wt->getdpSH() * tf->getnSpSt().T() * (tf->getrSt() * wt->getIwh1() + S * wt->getIxwh1() + tf->getnS() * wt->getIwh1twh1() + tf->getnSt() * wt->getIwh1wh1() + tf->getbS() * wt->getIwh1wh2t() + tf->getbSt() * wt->getIwh1wh2()) + (qItH * wt->getbSqIH() * tf->getrSt()) * wt->getwh2coefqIH() * wt->getvvtH() + (qItH * wt->getbSqIH() * S) * wt->getwh2coefqIH() * wt->getxvvtH() + (qItH * wt->getbSqIH() * tf->getnS()) * wt->getwh2coefqIH() * wt->getIwh1twwtH() + (qItH * wt->getbSqIH() * tf->getnSt()) * wt->getwh2coefqIH() * wt->getIwh1wwtH() + (qItH * wt->getbSqIH() * tf->getbS()) * wt->getwh2coefqIH() * wt->getIwh2twwtH() + (qItH * wt->getbSqIH() * tf->getbSt()) * wt->getwh2coefqIH() * wt->getIwh2wwtH() + wt->getwh2coefqInunutH() * ((tf->getbSH() * tf->getrSt()) * wt->getvvtH() + (tf->getbSH() * S) * wt->getxvvtH() + (tf->getbSH() * tf->getnSt()) * wt->getIwh1wwtH() + wt->getIwh2twwtH() + (tf->getbSH() * tf->getbSt()) * wt->getIwh2wwtH()) + wt->getbSqIH() * (tf->getrSt() * wt->getvvt() + S * wt->getxvvt() + tf->getnS() * wt->getIwh1twwt() + tf->getnSt() * wt->getIwh1wwt() + tf->getbS() * wt->getIwh2twwt() + tf->getbSt() * wt->getIwh2wwt()) * wt->getwh2coefqI() * tf->getqIt() + wt->getdpSH() * tf->getbSpSt().T() * (tf->getrSt() * wt->getIwh2() + S * wt->getIxwh2() + tf->getnS() * wt->getIwh1twh2() + tf->getnSt() * wt->getIwh1wh2() + tf->getbS() * wt->getIwh2twh2() + tf->getbSt() * wt->getIwh2wh2()));
      TqItqIqIt += I0 * wt->getTtilqItqIqIt();
      TqItqIqIt = rho * TqItqIqIt;

      /* summarizing RHS */
      Vec16 hIZ = (TqI - VeqI - VgqI).T() - TqItqIqIt;
      hIZ(6) -= epstD * tf->getepstilt();
      hIZ(15) -= k0D * tf->getk0t();
      hIZ -= MI * tf->getJIGt() * qGt;

      /* global description */
      h = tf->getJIG().T() * hIZ;

      qG_Old = qG;
      qGt_Old = qGt;
    }
  }

  void FiniteElement1s33RCM::computedhdz(const Vec16& qG, const Vec16& qGt) {
    Vec16 h0 = h;

    Vec16 qG_tmp = qG;
    Vec16 qGt_tmp = qGt;

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

  double FiniteElement1s33RCM::computeKineticEnergy(const Vec16& qG, const Vec16& qGt) {
    if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp)
      wt->computeint(qG, qGt);
    Vec3 S = tf->getepstilt() * tf->gettS() + (1. + tf->getepstil()) * tf->gettSt();

    return 0.5 * rho * (A * (tf->getrSt().T() * (tf->getrSt() * l0 + 2. * tf->getnS() * wt->getIwh1t() + 2. * tf->getnSt() * wt->getIwh1() + 2. * tf->getbS() * wt->getIwh2t() + 2. * tf->getbSt() * wt->getIwh2()) + S.T() * (S * l0h3 / 12. + 2. * tf->getnS() * wt->getIxwh1t() + 2. * tf->getnSt() * wt->getIxwh1() + 2. * tf->getbS() * wt->getIxwh2t() + 2. * tf->getbSt() * wt->getIxwh2()) + tf->getnSH() * (2. * tf->getnSt() * wt->getIwh1twh1() + 2. * tf->getbSt() * wt->getIwh1twh2()) + wt->getIwh1twh1t() + tf->getnStH() * (tf->getnSt() * wt->getIwh1wh1() + 2. * tf->getbS() * wt->getIwh1wh2t() + 2. * tf->getbSt() * wt->getIwh1wh2()) + tf->getbSH() * (2. * tf->getbSt() * wt->getIwh2twh2()) + wt->getIwh2twh2t() + tf->getbStH() * (tf->getbSt() * wt->getIwh2wh2())) + I0 * wt->getTtil());
  }

  double FiniteElement1s33RCM::computeGravitationalEnergy(const Vec16& qG) {
    if (nrm2(qG - qG_Old) > tol_comp)
      wt->computewhcoefPos(qG);

    double Iwh1 = wt->intv(wt->getwh1coef());
    double Iwh2 = wt->intv(wt->getwh2coef());

    return -rho * A * g.T() * (l0 * tf->getrS() + Iwh1 * tf->getnS() + Iwh2 * tf->getbS());
  }

  double FiniteElement1s33RCM::computeElasticEnergy(const Vec16& qG) {
    if (nrm2(qG - qG_Old) > tol_comp)
      wt->computewhcoefPos(qG);

    double Iwh1xwh1x = wt->intvw(wt->getwh1coef(), wt->getwh1coef());
    double Iwh2xwh2x = wt->intvw(wt->getwh2coef(), wt->getwh2coef());
    double Iwh1xxwh1xx = wt->intvxxvxx(wt->getwh1coef(), k10);
    double Iwh2xxwh2xx = wt->intvxxvxx(wt->getwh2coef(), k20);
    double eps = tf->getepstil() + (Iwh1xwh1x + Iwh2xwh2x) / (2. * l0);

    return 0.5 * (E * A * eps * eps * l0 + E * I1 * Iwh1xxwh1xx + E * I2 * Iwh2xxwh2xx + G * I0 * l0 * tf->getk0() * tf->getk0());
  }

  Mat16x6 FiniteElement1s33RCM::computeJXqG(const Vec16& qG, double x) {
    Mat16x6 JXqG;

    if (nrm2(qG - qG_Old) > tol_comp)
      wt->computewhcoefPosD(qG);

    RowVec4 wwt(NONINIT);
    wwt(3) = x * x;
    wwt(2) = wwt(3) * x;
    wwt(1) = wwt(2) * x;
    wwt(0) = wwt(1) * x;

    RowVec16 wh1qI = wwt * wt->getwh1coefqI();
    RowVec16 wh2qI = wwt * wt->getwh2coefqI();

    Vec2 wh1 = wt->computew(wt->getwh1coef(), x);
    Vec2 wh2 = wt->computew(wt->getwh2coef(), x);

    /* JT */
    JXqG.set(Index(0, 15), Index(0, 2), (drS + (tf->gettS() * depstil + (1. + tf->getepstil()) * wt->gettSqI()) * x + tf->getnS() * wh1qI + wh1(0) * wt->getnSqI() + tf->getbS() * wh2qI + wh2(0) * wt->getbSqI()).T());

    Vec2 w1 = wt->computew(wt->getw1coef(), x);
    Vec2 w2 = wt->computew(wt->getw2coef(), x);

    Vec3 p = tf->getpS();
    p(0) += sin((tf->getpS())(1)) * w2(1) + tf->getk0() * x;
    p(1) += w1(1);
    p(2) += w2(1);

    Vec3 t = ag->computet(p);
    Vec3 n = ag->computen(p);
    Vec3 b = ag->computeb(p);

    SqrMat3 tp = ag->computetq(p);
    SqrMat3 np = ag->computenq(p);
    SqrMat3 bp = ag->computebq(p);

    RowVec4 dwxdwt(NONINIT);
    dwxdwt(3) = 2 * x;
    dwxdwt(2) = wwt(3) * 3.;
    dwxdwt(1) = wwt(2) * 4.;
    dwxdwt(0) = wwt(1) * 5.;

    Mat3x16 Z(NONINIT);
    Z.set(Index(0, 0), Index(0, 15), sin((tf->getpS())(1)) * dwxdwt * wt->getw2coefqI() + x * dk0);
    Z(0, 4) += cos((tf->getpS())(1)) * w2(1);
    Z.set(Index(1, 1), Index(0, 15), dwxdwt * wt->getw1coefqI());
    Z.set(Index(2, 2), Index(0, 15), dwxdwt * wt->getw2coefqI());

    Mat3x16 ptqIt = wt->getdpS() + Z;
    MatV16 ttqIt = tp * ptqIt;
    MatV16 ntqIt = np * ptqIt;
    MatV16 btqIt = bp * ptqIt;

    /* JR */
    JXqG.set(Index(0, 15), Index(3, 3), (t(1) * ttqIt(Index(2, 2), Index(0, 15)) + n(1) * ntqIt(Index(2, 2), Index(0, 15)) + b(1) * btqIt(Index(2, 2), Index(0, 15))).T());
    JXqG.set(Index(0, 15), Index(4, 4), (t(2) * ttqIt(Index(0, 0), Index(0, 15)) + n(2) * ntqIt(Index(0, 0), Index(0, 15)) + b(2) * btqIt(Index(0, 0), Index(0, 15))).T());
    JXqG.set(Index(0, 15), Index(5, 5), (t(0) * ttqIt(Index(1, 1), Index(0, 15)) + n(0) * ntqIt(Index(1, 1), Index(0, 15)) + b(0) * btqIt(Index(1, 1), Index(0, 15))).T());

    JXqG = tf->getJIG().T() * JXqG; /* transformation global coordinates */

    return JXqG;
  }

  const Vec12& FiniteElement1s33RCM::computeState(const Vec16& qG, const Vec16& qGt, double x) {
    if (nrm2(qG - qG_Old) < tol_comp && nrm2(qGt - qGt_Old) < tol_comp && fabs(x - x_Old) < MBSim::epsroot())
      return X;
    else {
      if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp)
        wt->computewhcoefVel(qG, qGt);

      Vec2 wh1 = wt->computew(wt->getwh1coef(), x); // position and velocity
      Vec2 wh2 = wt->computew(wt->getwh2coef(), x); // position and velocity
      Vec2 wh1t = wt->computew(wt->getwh1tcoef(), x); // only velocity
      Vec2 wh2t = wt->computew(wt->getwh2tcoef(), x); // only velocity

      Vec2 w1 = wt->computew(wt->getw1coef(), x); // only position
      Vec2 w2 = wt->computew(wt->getw2coef(), x); // position and velocity
      Vec2 w1t = wt->computew(wt->getw1tcoef(), x); // only velocity
      Vec2 w2t = wt->computew(wt->getw2tcoef(), x); // only velocity

      X.set(Index(0, 2), tf->getrS() + (1. + tf->getepstil()) * x * tf->gettS() + wh1(0) * tf->getnS() + wh2(0) * tf->getbS()); /* pos */
      X.set(Index(3, 5), tf->getpS());
      X(3) += sin((tf->getpS())(1)) * w2(1) + tf->getk0() * x;
      X(4) += w1(1);
      X(5) += w2(1);/* angles */
      X.set(Index(6, 8), tf->getrSt() + (tf->getepstilt() * tf->gettS() + (1. + tf->getepstil()) * tf->gettSt()) * x + wh1t(0) * tf->getnS() + wh1(0) * tf->getnSt() + wh2t(0) * tf->getbS() + wh2(0) * tf->getbSt()); /* vel */
      X.set(Index(9, 11), tf->getpSt());
      X(9) += cos((tf->getpS())(1)) * (tf->getpSt())(1) * w2(1) + sin((tf->getpS())(1)) * w2t(1) + tf->getk0t() * x;
      X(10) += w1t(1);
      X(11) += w2t(1); /* time differentiated angles */

      return X;
    }
  }

  Vec16 FiniteElement1s33RCM::computeData(const Vec16& qG, const Vec16& qGt) {
    if (nrm2(qG - qG_Old) > tol_comp || nrm2(qGt - qGt_Old) > tol_comp)
      tf->computezI(qG, qGt);

    Vec16 Data(NONINIT);
    Data.set(Index(0, 2), tf->getrS()); /* rS */
    Data.set(Index(3, 5), qG(Range13x15()) - qG(Range3x5())); /* dp */
    Data(6) = tf->getk0(); /* k0 */
    Data(7) = tf->getepstil(); /* epstil */
    Data.set(Index(8, 10), tf->getrSt()); /* rSt */
    Data.set(Index(11, 13), qGt(Range13x15()) - qGt(Range3x5())); /* dpt */
    Data(14) = tf->getk0t(); /* k0t */
    Data(15) = tf->getepstilt(); /* epstilt */

    return Data;
  }

  void FiniteElement1s33RCM::computedrS() {
    drS(0, 0) = 1.;
    drS(1, 1) = 1.;
    drS(2, 2) = 1.;
    drSH = drS.T();
  }

  void FiniteElement1s33RCM::computedepstil() {
    depstil(6) = 1.;
  }

  void FiniteElement1s33RCM::computedk0() {
    dk0(15) = 1.;
  }


}
