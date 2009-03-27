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
#include <config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "finite_element_1s_21_rcm.h"
#include <fstream>

using namespace std;

namespace MBSim {

  // Funktionen, die im Mathematica-Output vorkommen: Anfang
  inline double Sec(const double& alpha) {
    return 1.0/cos(alpha);
  }

  inline double Power(double base, int exponent) {
    return pow(base,exponent);
  }
  // Funktionen, die im Mathematica-Output vorkommen: Ende

  //-------------------------------------------------------------------------
  FiniteElement1s21RCM::FiniteElement1s21RCM(double sl0, double sArho, double sEA, double sEI, Vec sg)
    :l0(sl0), Arho(sArho), EA(sEA), EI(sEI), wss0(0.0), depsilon(0.0), g(sg),
    M(8), h(8), implicit(false), Dhq(8), Dhqp(8),
    Damp(8,fmatvec::INIT,0.0)
  {
    M.init(0.0);
    h.init(0.0);

    // zur Performance-Steigerung nur einmal im Konstruktor rechnen
    l0h2 = l0*l0;
    l0h3 = l0h2*l0;
    l0h4 = l0h3*l0;
    l0h5 = l0h4*l0;
    l0h7 = l0h4*l0h3;
    l0h8 = l0h4*l0h4;

  }


  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------
  void FiniteElement1s21RCM::setCurleRadius(double R)
  {
    if (R == 0.0) {cout << "CurleRadius muss ungleich 0 sein!\n"; throw(1);}
    wss0 = 1/R;
  }
  void FiniteElement1s21RCM::setMaterialDamping(double depsilons)//, const double& dws)
  {
    depsilon  = depsilons;
    Damp(3,3) = -depsilon;
  }
  void FiniteElement1s21RCM::setLehrDamping(double D)
  {
    // Longitudinaleifenfrequenz
    double weps = sqrt(12.*EA/(Arho*l0h2));
    // Biegeeigenfrequenz
    //    double wbL  = sqrt(1260.*128.*EI/(197.*Arho*l0h4));

    depsilon  = 2.0 * D * weps * (     Arho*l0h3/12.  );
    Damp(3,3) = -depsilon;
    //    dw       = 0.0 * D * wbL  * (197.*Arho*l0h3/1260.);

  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------
  void FiniteElement1s21RCM::computeEquationsOfMotion(const Vec& qElement, const Vec& qpElement)
  {
    Vec    qLokal(8,fmatvec::INIT,0.0), qpLokal(8,fmatvec::INIT,0.0);

    SymMat MLokal(8,fmatvec::INIT,0.0);
    SqrMat Jeg   (8,fmatvec::INIT,0.0), Jegp   (8,fmatvec::INIT,0.0);
    Vec    hLokal(8,fmatvec::INIT,0.0), hdLokal(8,fmatvec::INIT,0.0); // Daempungsanteil

    //--- globale Koordinaten, Geschwingigkeiten
    /*    double &x1   = qElement(0);     double &y1  = qElement(1);    
	  double &phi1 = qElement(2);    
	  double &a1   = qElement(3);     double &a2  = qElement(4);
	  double &x2   = qElement(5);     double &y2  = qElement(6);
	  double &phi2 = qElement(7);
    //
    double &x1p   = qpElement(0);     double &y1p  = qpElement(1);    
    double &phi1p = qpElement(2);    
    double &a1p   = qpElement(3);     double &a2p  = qpElement(4);
    double &x2p   = qpElement(5);     double &y2p  = qpElement(6);
    double &phi2p = qpElement(7);
    */

    //--- lokale  Koordinaten, Geschwingigkeiten
    //    double &xS     = qLokal(0);    // unused  
    //    double &yS     = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);
    //
    //    double &xSp    = qpLokal(0);     double &ySp   = qpLokal(1);
    double &phiSp  = qpLokal(2);     double &epsp  = qpLokal(3);
    double &aLp    = qpLokal(4);     double &bLp   = qpLokal(5);
    double &aRp    = qpLokal(6);     double &bRp   = qpLokal(7);

    // Gravitation
    double gx = g(0);
    double gy = g(1);

    //lokale Koordinate-----------------------------------------------------------
    BuildqLokal(qElement,qLokal);

    //JacobiMatrizen--------------------------------------------------------------
    BuildJacobi(qElement,qpElement,Jeg,Jegp);

    // Lokale Geschwindigkeit-----------------------------------------------------
    qpLokal << Jeg * qpElement;

    //MassenMatrix----------------------------------------------------------------
    MLokal(0,0) = Arho*l0;
    //MLokal(0,1) = 0;
    MLokal(0,2) = (Arho*l0*((104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*cos(bL - phiS) + (8*aL + 104*aR - 3*bL*l0 + 15*bR*l0)*cos(bR + phiS) - 60*(1 + eps)*l0*(sin(bL - phiS) + sin(bR + phiS))))/480.;
    MLokal(0,3) = (Arho*l0h2*(-cos(bL - phiS) + cos(bR + phiS)))/8.;
    MLokal(0,4) = (Arho*l0*(-13*sin(bL - phiS) + sin(bR + phiS)))/60.;
    MLokal(0,5) = -(Arho*l0*((104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*cos(bL - phiS) + 3*l0*(-5*(3 + 4*eps)*sin(bL - phiS) + sin(bR + phiS))))/480.;
    MLokal(0,6) = -(Arho*l0*(sin(bL - phiS) - 13*sin(bR + phiS)))/60.;
    MLokal(0,7) = (Arho*l0*((8*aL + 104*aR - 3*bL*l0 + 15*bR*l0)*cos(bR + phiS) + 3*l0*(sin(bL - phiS) - 5*(3 + 4*eps)*sin(bR + phiS))))/480.;
    //MLokal(1,0) = MLokal(0,1);
    MLokal(1,1) = Arho*l0;
    MLokal(1,2) = (Arho*l0*(-60*(1 + eps)*l0*cos(bL - phiS) + 60*(1 + eps)*l0*cos(bR + phiS) + (-104*aL - 8*aR + 3*(-5*bL + bR)*l0)*sin(bL - phiS) + (8*aL + 104*aR - 3*(bL - 5*bR)*l0)*sin(bR + phiS)))/480.;
    MLokal(1,3) = (Arho*l0h2*(sin(bL - phiS) + sin(bR + phiS)))/8.;
    MLokal(1,4) = -(Arho*l0*(13*cos(bL - phiS) + cos(bR + phiS)))/60.;
    MLokal(1,5) = (Arho*l0*(3*l0*(5*(3 + 4*eps)*cos(bL - phiS) + cos(bR + phiS)) + (104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*sin(bL - phiS)))/480.;
    MLokal(1,6) = -(Arho*l0*(cos(bL - phiS) + 13*cos(bR + phiS)))/60.;
    MLokal(1,7) = (Arho*l0*(3*l0*(cos(bL - phiS) + 5*(3 + 4*eps)*cos(bR + phiS)) + (8*aL + 104*aR - 3*(bL - 5*bR)*l0)*sin(bR + phiS)))/480.;
    //MLokal(2,0) = MLokal(0,2);
    //MLokal(2,1) = MLokal(1,2);
    MLokal(2,2) = (Arho*l0*(3152*Power(aL,2) + 3152*Power(aR,2) - 46*aR*bL*l0 + 398*aR*bR*l0 + (55*Power(bL,2) - 42*bL*bR + 55*Power(bR,2) + 1680*Power(1 + eps,2))*l0h2 + aL*(544*aR + 398*bL*l0 - 46*bR*l0)))/20160.;
    MLokal(2,3) = -(Arho*l0h2*(9*aL - 9*aR + (bL - bR)*l0))/120.;
    MLokal(2,4) = (3*Arho*(1 + eps)*l0h2)/40.;
    MLokal(2,5) = -(Arho*l0*(64*(196*Power(aL,2) + 17*aL*aR + Power(aR,2)) + 8*(205*aL*bL + 28*aR*bL - 51*aL*bR - 6*aR*bR)*l0 + (211*Power(bL,2) - 84*bL*bR + 9*Power(bR,2) + 672*(1 + eps)*(4 + 5*eps))*l0h2))/80640.;
    MLokal(2,6) = (-3*Arho*(1 + eps)*l0h2)/40.;
    MLokal(2,7) = (Arho*l0*(64*(Power(aL,2) + 17*aL*aR + 196*Power(aR,2)) - 8*(6*aL*bL + 51*aR*bL - 28*aL*bR - 205*aR*bR)*l0 + (9*Power(bL,2) - 84*bL*bR + 211*Power(bR,2) + 672*(1 + eps)*(4 + 5*eps))*l0h2))/80640.;
    //MLokal(3,0) = MLokal(0,3);
    //MLokal(3,1) = MLokal(1,3);
    //MLokal(3,2) = MLokal(2,3);
    MLokal(3,3) = (Arho*l0h3)/12.;
    //MLokal(3,4) = 0;
    MLokal(3,5) = (Arho*l0h2*(152*aL + 8*aR + 13*bL*l0 - 3*bR*l0))/1920.;
    //MLokal(3,6) = 0;
    MLokal(3,7) = (Arho*l0h2*(8*aL + 152*aR - 3*bL*l0 + 13*bR*l0))/1920.;
    //MLokal(4,0) = MLokal(0,4);
    //MLokal(4,1) = MLokal(1,4);
    //MLokal(4,2) = MLokal(2,4);
    ////MLokal(4,3) = MLokal(3,4);
    MLokal(4,4) = (197*Arho*l0)/1260.;
    MLokal(4,5) = -(Arho*(1397 + 1596*eps)*l0h2)/20160.;
    MLokal(4,6) = (17*Arho*l0)/1260.;
    MLokal(4,7) = -(Arho*(107 + 84*eps)*l0h2)/20160.;
    //MLokal(5,0) = MLokal(0,5);
    //MLokal(5,1) = MLokal(1,5);
    //MLokal(5,2) = MLokal(2,5);
    //MLokal(5,3) = MLokal(3,5);
    //MLokal(5,4) = MLokal(4,5);
    MLokal(5,5) = (Arho*l0*(64*(196*Power(aL,2) + 17*aL*aR + Power(aR,2)) + 8*(205*aL*bL + 28*aR*bL - 51*aL*bR - 6*aR*bR)*l0 + (211*Power(bL,2) - 84*bL*bR + 9*Power(bR,2) + 4*(622 + 21*eps*(67 + 40*eps)))*l0h2))/80640.;
    MLokal(5,6) = -(Arho*(107 + 84*eps)*l0h2)/20160.;
    MLokal(5,7) = (Arho*(2 + 3*eps)*l0h3)/960.;
    //MLokal(6,0) = MLokal(0,6);
    //MLokal(6,1) = MLokal(1,6);
    //MLokal(6,2) = MLokal(2,6);
    ////MLokal(6,3) = MLokal(3,6);
    //MLokal(6,4) = MLokal(4,6);
    //MLokal(6,5) = MLokal(5,6);
    MLokal(6,6) = (197*Arho*l0)/1260.;
    MLokal(6,7) = -(Arho*(1397 + 1596*eps)*l0h2)/20160.;
    //MLokal(7,0) = MLokal(0,7);
    //MLokal(7,1) = MLokal(1,7);
    //MLokal(7,2) = MLokal(2,7);
    //MLokal(7,3) = MLokal(3,7);
    //MLokal(7,4) = MLokal(4,7);
    //MLokal(7,5) = MLokal(5,7);
    //MLokal(7,6) = MLokal(6,7);
    MLokal(7,7) = (Arho*l0*(64*(Power(aL,2) + 17*aL*aR + 196*Power(aR,2)) - 8*(6*aL*bL + 51*aR*bL - 28*aL*bR - 205*aR*bR)*l0 + (9*Power(bL,2) - 84*bL*bR + 211*Power(bR,2) + 4*(622 + 21*eps*(67 + 40*eps)))*l0h2))/80640.;

    // //h-Vektor--------------------------------------------------------------------
    //     hLokal(0) = -(Arho*l0*(2*(bLp - phiSp)*(-104*aLp - 8*aRp + 3*l0*(5*bLp + bRp + 10*bLp*eps - 10*(1 + eps)*phiSp))*cos(bL - phiS) - 2*(bRp + phiSp)*(-8*aLp - 104*aRp + 3*l0*(bLp + 5*(bRp + 2*bRp*eps + 2*(1 + eps)*phiSp)))*cos(bR + phiS) + (3*l0*(40*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*(bLp - phiSp)*sin(bL - phiS) - (bRp + phiSp)*(8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-40*epsp + (bL - 5*bR)*(bRp + phiSp)))*sin(bR + phiS)))/480.;
    //     hLokal(1) = -(Arho*g*l0) - (Arho*l0*((3*l0*(40*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*(bLp - phiSp)*cos(bL - phiS) + (bRp + phiSp)*(8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-40*epsp + (bL - 5*bR)*(bRp + phiSp)))*cos(bR + phiS) + 2*(bLp - phiSp)*(104*aLp + 8*aRp - 3*(5*bLp + bRp + 10*bLp*eps)*l0 + 30*(1 + eps)*l0*phiSp)*sin(bL - phiS) - 2*(bRp + phiSp)*(-8*aLp - 104*aRp + 3*l0*(bLp + 5*(bRp + 2*bRp*eps + 2*(1 + eps)*phiSp)))*sin(bR + phiS)))/480.;
    //     hLokal(2) = (Arho*l0*(64*aR*aRp*bLp - 12544*aR*aRp*bRp + 112*aRp*bL*bLp*l0 + 112*aR*Power(bLp,2)*l0 - 24*aRp*bLp*bR*l0 + 204*aRp*bL*bRp*l0 + 180*aR*bLp*bRp*l0 - 820*aRp*bR*bRp*l0 - 820*aR*Power(bRp,2)*l0 + 211*bL*Power(bLp,2)*l0h2 - 42*Power(bLp,2)*bR*l0h2 - 51*bL*bLp*bRp*l0h2 + 51*bLp*bR*bRp*l0h2 + 42*bL*Power(bRp,2)*l0h2 - 211*bR*Power(bRp,2)*l0h2 + 3360*bLp*epsp*l0h2 - 3360*bRp*epsp*l0h2 + 3360*bLp*eps*epsp*l0h2 - 3360*bRp*eps*epsp*l0h2 - 4*(aR*(3152*aRp - 23*bLp*l0 + 199*bRp*l0) + l0*(aRp*(-23*bL + 199*bR) + (55*bL*bLp - 21*bLp*bR - 21*bL*bRp + 55*bR*bRp + 1680*(1 + eps)*epsp)*l0))*phiSp + 4*aL*(16*aLp*(196*bLp - bRp - 197*phiSp) + 136*aRp*(bLp - bRp - 2*phiSp) + l0*(205*Power(bLp,2) - 45*bLp*bRp - 28*Power(bRp,2) - 199*bLp*phiSp + 23*bRp*phiSp)) + 4*aLp*(136*aR*(bLp - bRp - 2*phiSp) + l0*(bL*(205*bLp + 6*bRp - 199*phiSp) + bR*(-51*bLp - 28*bRp + 23*phiSp))) + 84*g*(60*(1 + eps)*l0*cos(bL - phiS) - 60*(1 + eps)*l0*cos(bR + phiS) + (104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*sin(bL - phiS) - (8*aL + 104*aR - 3*bL*l0 + 15*bR*l0)*sin(bR + phiS))))/40320.;
    //     hLokal(3) = -(EA*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2)) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*l0 + 3*(140 + 9*Power(bL,2) - 6*bL*bR + 9*Power(bR,2))*l0h2)*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2))/(176400.*l0h3) - (aLp*Arho*l0h2*(19*bLp + bRp - 18*phiSp))/120. - (Arho*aRp*l0h2*(bLp + 19*bRp + 18*phiSp))/120. + (Arho*l0h3*(67*Power(bLp,2) + 6*bLp*bRp + 67*Power(bRp,2) + 80*Power(bLp,2)*eps + 80*Power(bRp,2)*eps - 16*(bLp - bRp)*(9 + 10*eps)*phiSp + 160*(1 + eps)*Power(phiSp,2)))/1920. - (Arho*l0h3*(13*Power(bLp,2) - 2*bLp*(3*bRp + 8*phiSp) + bRp*(13*bRp + 16*phiSp)))/1920. - (Arho*g*l0h2*sin(bL - phiS))/8. - (Arho*g*l0h2*sin(bR + phiS))/8.;
    //     hLokal(4) = -(282240*EI*(160*aL - 32*aR - 43*bL*l0 + 11*bR*l0) + 8*EA*(1 + eps)*(272*aL - 16*aR - 24*bL*l0 + 9*bR*l0)*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2))/(352800.*l0h3) + (Arho*epsp*l0h2*(19*bLp + bRp - 18*phiSp))/240. + (aL*Arho*l0*(196*Power(bLp,2) + Power(bRp,2) - 392*bLp*phiSp + 2*bRp*phiSp + 197*Power(phiSp,2)))/1260. + (Arho*l0h2*(205*bL*Power(bLp,2) - 51*Power(bLp,2)*bR - 6*bL*Power(bRp,2) + 28*bR*Power(bRp,2) + 1596*bLp*epsp + 84*bRp*epsp - 2*(205*bL*bLp - 51*bLp*bR + 6*bL*bRp - 28*bR*bRp + 756*epsp)*phiSp + (199*bL - 23*bR)*Power(phiSp,2)))/20160. + (17*aR*Arho*l0*(Power(bLp,2) + Power(bRp,2) - 2*bLp*phiSp + 2*phiSp*(bRp + phiSp)))/2520. + (Arho*g*l0*(13*cos(bL - phiS) + cos(bR + phiS)))/60.;  //(13*Arho*l0*(bLp - phiSp)*xSp*cos(bL - phiS))/60. + (13*Arho*l0*(-bLp + phiSp)*xSp*cos(bL - phiS))/60. + 
    //     hLokal(5) = -(-835584*Power(aL,3)*EA*Power(1 + eps,2) + 313344*Power(aR,3)*EA*Power(1 + eps,2) + 2304*Power(aR,2)*(213*bL - 92*bR)*EA*Power(1 + eps,2)*l0 + 3072*Power(aL,2)*EA*Power(1 + eps,2)*(134*aR + 201*bL*l0 - 69*bR*l0) - 4*aL*(24272640*EI + 218112*Power(aR,2)*EA*Power(1 + eps,2) + 5184*aR*(8*bL - 9*bR)*EA*Power(1 + eps,2)*l0 + l0h2*(288*EA*(1 + eps)*(560*eps + 3*(36*Power(bL,2) - 25*bL*bR + 15*Power(bR,2))*(1 + eps)) - 35*Arho*l0*(6272*aLp*(bLp - phiSp) + 272*aRp*(bLp - phiSp) + l0*(205*Power(bLp,2) - 102*bLp*bRp + 6*Power(bRp,2) + 114*bRp*phiSp - 199*Power(phiSp,2))))) + 4*aR*(6209280*EI + l0h2*(432*EA*(1 + eps)*(140*eps + (3*bL - 5*bR)*(9*bL - 5*bR)*(1 + eps)) + 35*Arho*l0*(272*aLp*(bLp - phiSp) + 32*aRp*(bLp - phiSp) + l0*(28*Power(bLp,2) - 12*bLp*bRp + 51*Power(bRp,2) + 114*bRp*phiSp + 23*Power(phiSp,2))))) + l0*(23328*Power(bL,3)*EA*Power(1 + eps,2)*l0h2 - 23328*Power(bL,2)*bR*EA*Power(1 + eps,2)*l0h2 + bL*(32175360*EI + l0h2*(2592*EA*(1 + eps)*(140*eps + 11*Power(bR,2)*(1 + eps)) + 35*Arho*l0*(1640*aLp*(bLp - phiSp) + 224*aRp*(bLp - phiSp) + l0*(211*Power(bLp,2) - 84*bLp*bRp - 9*Power(bRp,2) + 66*bRp*phiSp - 220*Power(phiSp,2))))) - 6*(1296*Power(bR,3)*EA*Power(1 + eps,2)*l0h2 + 35*bR*(40320*EI + l0h2*(576*EA*eps*(1 + eps) + Arho*l0*(68*aLp*(bLp - phiSp) + 8*aRp*(bLp - phiSp) + l0*(7*Power(bLp,2) - 3*bLp*bRp - 7*Power(bRp,2) - 11*bRp*phiSp - 14*Power(phiSp,2))))) + 490*l0*(Arho*epsp*l0h3*(-3*bRp - bLp*(67 + 80*eps) + 64*phiSp + 80*eps*phiSp) + 960*EI*wss0))) + 5880*Arho*g*l0h3*(3*l0*(5*(3 + 4*eps)*cos(bL - phiS) + cos(bR + phiS)) + (104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*sin(bL - phiS)))/(2.8224e6*l0h2);
    //     hLokal(6) = (282240*EI*(32*aL - 160*aR - 11*bL*l0 + 43*bR*l0) + 8*EA*(1 + eps)*(16*aL - 272*aR - 9*bL*l0 + 24*bR*l0)*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2))/(352800.*l0h3) + (Arho*epsp*l0h2*(bLp + 19*bRp + 18*phiSp))/240. + (aR*Arho*l0*(Power(bLp,2) + 196*Power(bRp,2) - 2*bLp*phiSp + 392*bRp*phiSp + 197*Power(phiSp,2)))/1260. + (Arho*l0h2*(28*bL*Power(bLp,2) - 6*Power(bLp,2)*bR - 51*bL*Power(bRp,2) + 205*bR*Power(bRp,2) + 84*bLp*epsp + 1596*bRp*epsp + 2*(-28*bL*bLp + 6*bLp*bR - 51*bL*bRp + 205*bR*bRp + 756*epsp)*phiSp + (-23*bL + 199*bR)*Power(phiSp,2)))/20160. + (17*aL*Arho*l0*(Power(bLp,2) + Power(bRp,2) - 2*bLp*phiSp + 2*phiSp*(bRp + phiSp)))/2520. + (Arho*g*l0*(cos(bL - phiS) + 13*cos(bR + phiS)))/60.; //(Arho*l0*(bLp - phiSp)*xSp*cos(bL - phiS))/60. + (Arho*l0*(-bLp + phiSp)*xSp*cos(bL - phiS))/60. + 
    //     hLokal(7) = -(313344*Power(aL,3)*EA*Power(1 + eps,2) - 835584*Power(aR,3)*EA*Power(1 + eps,2) + 32175360*bR*EI*l0 - 9216*Power(aR,2)*(23*bL - 67*bR)*EA*Power(1 + eps,2)*l0 + 23328*Power(bR,3)*EA*l0h3 + 362880*bR*EA*eps*l0h3 + 46656*Power(bR,3)*EA*eps*l0h3 + 362880*bR*EA*Power(eps,2)*l0h3 + 23328*Power(bR,3)*EA*Power(eps,2)*l0h3 - 7776*Power(bL,3)*EA*Power(1 + eps,2)*l0h3 + 28512*Power(bL,2)*bR*EA*Power(1 + eps,2)*l0h3 + 7840*aLp*Arho*bR*bRp*l0h4 + 57400*Arho*aRp*bR*bRp*l0h4 - 315*Arho*Power(bLp,2)*bR*l0h5 - 2940*Arho*bLp*bR*bRp*l0h5 + 7385*Arho*bR*Power(bRp,2)*l0h5 + 8820*Arho*bLp*epsp*l0h5 + 196980*Arho*bRp*epsp*l0h5 + 235200*Arho*bRp*eps*epsp*l0h5 - 768*Power(aL,2)*EA*Power(1 + eps,2)*(1136*aR + 276*bL*l0 - 639*bR*l0) + 7840*aLp*Arho*bR*l0h4*phiSp + 57400*Arho*aRp*bR*l0h4*phiSp - 2310*Arho*bLp*bR*l0h5*phiSp + 188160*Arho*epsp*l0h5*phiSp + 235200*Arho*eps*epsp*l0h5*phiSp - 7700*Arho*bR*l0h5*Power(phiSp,2) - 6*bL*l0*(1411200*EI + l0h2*(144*EA*(1 + eps)*(140*eps + 27*Power(bR,2)*(1 + eps)) + 35*Arho*l0*(8*aLp*(bRp + phiSp) + 68*aRp*(bRp + phiSp) + l0*(-7*Power(bLp,2) - 3*bLp*bRp + 7*Power(bRp,2) + 11*bLp*phiSp - 14*Power(phiSp,2))))) + 4*aL*(6209280*EI + 102912*Power(aR,2)*EA*Power(1 + eps,2) + 5184*aR*(9*bL - 8*bR)*EA*Power(1 + eps,2)*l0 + l0h2*(432*EA*(1 + eps)*(140*eps + (5*bL - 9*bR)*(5*bL - 3*bR)*(1 + eps)) + 35*Arho*l0*(32*aLp*(bRp + phiSp) + 272*aRp*(bRp + phiSp) + l0*(51*Power(bLp,2) + 28*Power(bRp,2) + 23*Power(phiSp,2) - 6*bLp*(2*bRp + 19*phiSp))))) - 4*aR*(24272640*EI + l0h2*(288*EA*(1 + eps)*(560*eps + 3*(15*Power(bL,2) - 25*bL*bR + 36*Power(bR,2))*(1 + eps)) - 35*Arho*l0*(272*aLp*(bRp + phiSp) + 6272*aRp*(bRp + phiSp) + l0*(6*Power(bLp,2) + 205*Power(bRp,2) - 199*Power(phiSp,2) - 6*bLp*(17*bRp + 19*phiSp))))) - 2822400*EI*l0h2*wss0 + 5880*Arho*g*l0h3*(3*l0*(cos(bL - phiS) + 5*(3 + 4*eps)*cos(bR + phiS)) + (8*aL + 104*aR - 3*(bL - 5*bR)*l0)*sin(bR + phiS)))/(2.8224e6*l0h2);
    //h-Vektor
    hLokal(0) = (Arho*l0*(480*gx + 2*(bLp - phiSp)*(104*aLp + 8*aRp - 3*(5*bLp + bRp + 10*bLp*eps)*l0 + 30*(1 + eps)*l0*phiSp)*cos(bL - phiS) + 2*(bRp + phiSp)*(-8*aLp - 104*aRp + 3*l0*(bLp + 5*(bRp + 2*bRp*eps + 2*(1 + eps)*phiSp)))*cos(bR + phiS) - (3*l0*(40*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*(bLp - phiSp)*sin(bL - phiS) + (bRp + phiSp)*(8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-40*epsp + (bL - 5*bR)*(bRp + phiSp)))*sin(bR + phiS)))/480.;
    hLokal(1) = Arho*gy*l0 - (Arho*l0*((3*l0*(40*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*(bLp - phiSp)*cos(bL - phiS) + (bRp + phiSp)*(8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-40*epsp + (bL - 5*bR)*(bRp + phiSp)))*cos(bR + phiS) + 2*(bLp - phiSp)*(104*aLp + 8*aRp - 3*(5*bLp + bRp + 10*bLp*eps)*l0 + 30*(1 + eps)*l0*phiSp)*sin(bL - phiS) - 2*(bRp + phiSp)*(-8*aLp - 104*aRp + 3*l0*(bLp + 5*(bRp + 2*bRp*eps + 2*(1 + eps)*phiSp)))*sin(bR + phiS)))/480.;
    hLokal(2) = (Arho*l0*(64*aR*aRp*bLp - 12544*aR*aRp*bRp + 112*aRp*bL*bLp*l0 + 112*aR*Power(bLp,2)*l0 - 24*aRp*bLp*bR*l0 + 204*aRp*bL*bRp*l0 + 180*aR*bLp*bRp*l0 - 820*aRp*bR*bRp*l0 - 820*aR*Power(bRp,2)*l0 + 211*bL*Power(bLp,2)*l0h2 - 42*Power(bLp,2)*bR*l0h2 - 51*bL*bLp*bRp*l0h2 + 51*bLp*bR*bRp*l0h2 + 42*bL*Power(bRp,2)*l0h2 - 211*bR*Power(bRp,2)*l0h2 + 3360*bLp*epsp*l0h2 - 3360*bRp*epsp*l0h2 + 3360*bLp*eps*epsp*l0h2 - 3360*bRp*eps*epsp*l0h2 - 4*(aR*(3152*aRp - 23*bLp*l0 + 199*bRp*l0) + l0*(aRp*(-23*bL + 199*bR) + (55*bL*bLp - 21*bLp*bR - 21*bL*bRp + 55*bR*bRp + 1680*(1 + eps)*epsp)*l0))*phiSp + 4*aL*(16*aLp*(196*bLp - bRp - 197*phiSp) + 136*aRp*(bLp - bRp - 2*phiSp) + l0*(205*Power(bLp,2) - 45*bLp*bRp - 28*Power(bRp,2) - 199*bLp*phiSp + 23*bRp*phiSp)) + 4*aLp*(136*aR*(bLp - bRp - 2*phiSp) + l0*(bL*(205*bLp + 6*bRp - 199*phiSp) + bR*(-51*bLp - 28*bRp + 23*phiSp))) + 84*((8*(13*aL + aR)*gx + 3*(5*bL*gx - bR*gx - 20*(1 + eps)*gy)*l0)*cos(bL - phiS) + (8*aL*gx + 104*aR*gx + 3*(-(bL*gx) + 5*bR*gx + 20*(1 + eps)*gy)*l0)*cos(bR + phiS) - (60*(1 + eps)*gx*l0 + gy*(104*aL + 8*aR + 15*bL*l0 - 3*bR*l0))*sin(bL - phiS) + (8*(aL + 13*aR)*gy - 3*(20*(1 + eps)*gx + (bL - 5*bR)*gy)*l0)*sin(bR + phiS))))/40320.;
    hLokal(3) = -(EA*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2)) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*l0 + 3*(140 + 9*Power(bL,2) - 6*bL*bR + 9*Power(bR,2))*l0h2)*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2))/(176400.*l0h3) - (aLp*Arho*l0h2*(19*bLp + bRp - 18*phiSp))/120. - (Arho*aRp*l0h2*(bLp + 19*bRp + 18*phiSp))/120. + (Arho*l0h3*(67*Power(bLp,2) + 6*bLp*bRp + 67*Power(bRp,2) + 80*Power(bLp,2)*eps + 80*Power(bRp,2)*eps - 16*(bLp - bRp)*(9 + 10*eps)*phiSp + 160*(1 + eps)*Power(phiSp,2)))/1920. - (Arho*l0h3*(13*Power(bLp,2) - 2*bLp*(3*bRp + 8*phiSp) + bRp*(13*bRp + 16*phiSp)))/1920. - (Arho*gx*l0h2*cos(bL - phiS))/8. + (Arho*gx*l0h2*cos(bR + phiS))/8. + (Arho*gy*l0h2*sin(bL - phiS))/8. + (Arho*gy*l0h2*sin(bR + phiS))/8.;
    hLokal(4) = -(282240*EI*(160*aL - 32*aR - 43*bL*l0 + 11*bR*l0) + 8*EA*(1 + eps)*(272*aL - 16*aR - 24*bL*l0 + 9*bR*l0)*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2))/(352800.*l0h3) + (Arho*epsp*l0h2*(19*bLp + bRp - 18*phiSp))/240. + (aL*Arho*l0*(196*Power(bLp,2) + Power(bRp,2) - 392*bLp*phiSp + 2*bRp*phiSp + 197*Power(phiSp,2)))/1260. + (Arho*l0h2*(205*bL*Power(bLp,2) - 51*Power(bLp,2)*bR - 6*bL*Power(bRp,2) + 28*bR*Power(bRp,2) + 1596*bLp*epsp + 84*bRp*epsp - 2*(205*bL*bLp - 51*bLp*bR + 6*bL*bRp - 28*bR*bRp + 756*epsp)*phiSp + (199*bL - 23*bR)*Power(phiSp,2)))/20160. + (17*aR*Arho*l0*(Power(bLp,2) + Power(bRp,2) - 2*bLp*phiSp + 2*phiSp*(bRp + phiSp)))/2520. - (13*Arho*gy*l0*cos(bL - phiS))/60.  - (Arho*gy*l0*cos(bR + phiS))/60. - (13*Arho*gx*l0*sin(bL - phiS))/60. + (Arho*gx*l0*sin(bR + phiS))/60.;
    hLokal(5) = -(-835584*Power(aL,3)*EA*Power(1 + eps,2) + 313344*Power(aR,3)*EA*Power(1 + eps,2) + 2304*Power(aR,2)*(213*bL - 92*bR)*EA*Power(1 + eps,2)*l0 + 3072*Power(aL,2)*EA*Power(1 + eps,2)*(134*aR + 201*bL*l0 - 69*bR*l0) - 4*aL*(24272640*EI + 218112*Power(aR,2)*EA*Power(1 + eps,2) + 5184*aR*(8*bL - 9*bR)*EA*Power(1 + eps,2)*l0 + l0h2*(288*EA*(1 + eps)*(560*eps + 3*(36*Power(bL,2) - 25*bL*bR + 15*Power(bR,2))*(1 + eps)) - 35*Arho*l0*(6272*aLp*(bLp - phiSp) + 272*aRp*(bLp - phiSp) + l0*(205*Power(bLp,2) - 102*bLp*bRp + 6*Power(bRp,2) + 114*bRp*phiSp - 199*Power(phiSp,2))))) + 4*aR*(6209280*EI + l0h2*(432*EA*(1 + eps)*(140*eps + (3*bL - 5*bR)*(9*bL - 5*bR)*(1 + eps)) + 35*Arho*l0*(272*aLp*(bLp - phiSp) + 32*aRp*(bLp - phiSp) + l0*(28*Power(bLp,2) - 12*bLp*bRp + 51*Power(bRp,2) + 114*bRp*phiSp + 23*Power(phiSp,2))))) + l0*(23328*Power(bL,3)*EA*Power(1 + eps,2)*l0h2 - 23328*Power(bL,2)*bR*EA*Power(1 + eps,2)*l0h2 + bL*(32175360*EI + l0h2*(2592*EA*(1 + eps)*(140*eps + 11*Power(bR,2)*(1 + eps)) + 35*Arho*l0*(1640*aLp*(bLp - phiSp) + 224*aRp*(bLp - phiSp) + l0*(211*Power(bLp,2) - 84*bLp*bRp - 9*Power(bRp,2) + 66*bRp*phiSp - 220*Power(phiSp,2))))) - 6*(1296*Power(bR,3)*EA*Power(1 + eps,2)*l0h2 + 35*bR*(40320*EI + l0h2*(576*EA*eps*(1 + eps) + Arho*l0*(68*aLp*(bLp - phiSp) + 8*aRp*(bLp - phiSp) + l0*(7*Power(bLp,2) - 3*bLp*bRp - 7*Power(bRp,2) - 11*bRp*phiSp - 14*Power(phiSp,2))))) + 490*l0*(Arho*epsp*l0h3*(-3*bRp - bLp*(67 + 80*eps) + 64*phiSp + 80*eps*phiSp) + 960*EI*wss0))) - 5880*Arho*l0h3*((-104*aL*gx - 8*aR*gx + 3*((-5*bL + bR)*gx + 5*(3 + 4*eps)*gy)*l0)*cos(bL - phiS) + 3*gy*l0*cos(bR + phiS) + (8*(13*aL + aR)*gy + 3*(5*(3 + 4*eps)*gx + (5*bL - bR)*gy)*l0)*sin(bL - phiS) - 3*gx*l0*sin(bR + phiS)))/(2.8224e6*l0h2);
    hLokal(6) = (282240*EI*(32*aL - 160*aR - 11*bL*l0 + 43*bR*l0) + 8*EA*(1 + eps)*(16*aL - 272*aR - 9*bL*l0 + 24*bR*l0)*(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2))/(352800.*l0h3) + (Arho*epsp*l0h2*(bLp + 19*bRp + 18*phiSp))/240. + (aR*Arho*l0*(Power(bLp,2) + 196*Power(bRp,2) - 2*bLp*phiSp + 392*bRp*phiSp + 197*Power(phiSp,2)))/1260. + (Arho*l0h2*(28*bL*Power(bLp,2) - 6*Power(bLp,2)*bR - 51*bL*Power(bRp,2) + 205*bR*Power(bRp,2) + 84*bLp*epsp + 1596*bRp*epsp + 2*(-28*bL*bLp + 6*bLp*bR - 51*bL*bRp + 205*bR*bRp + 756*epsp)*phiSp + (-23*bL + 199*bR)*Power(phiSp,2)))/20160. + (17*aL*Arho*l0*(Power(bLp,2) + Power(bRp,2) - 2*bLp*phiSp + 2*phiSp*(bRp + phiSp)))/2520. - (Arho*gy*l0*cos(bL - phiS))/60. - (13*Arho*gy*l0*cos(bR + phiS))/60. - (Arho*gx*l0*sin(bL - phiS))/60. + (13*Arho*gx*l0*sin(bR + phiS))/60.;
    hLokal(7) = -(313344*Power(aL,3)*EA*Power(1 + eps,2) - 835584*Power(aR,3)*EA*Power(1 + eps,2) + 32175360*bR*EI*l0 - 9216*Power(aR,2)*(23*bL - 67*bR)*EA*Power(1 + eps,2)*l0 + 23328*Power(bR,3)*EA*l0h3 + 362880*bR*EA*eps*l0h3 + 46656*Power(bR,3)*EA*eps*l0h3 + 362880*bR*EA*Power(eps,2)*l0h3 + 23328*Power(bR,3)*EA*Power(eps,2)*l0h3 - 7776*Power(bL,3)*EA*Power(1 + eps,2)*l0h3 + 28512*Power(bL,2)*bR*EA*Power(1 + eps,2)*l0h3 + 7840*aLp*Arho*bR*bRp*l0h4 + 57400*Arho*aRp*bR*bRp*l0h4 - 315*Arho*Power(bLp,2)*bR*l0h5 - 2940*Arho*bLp*bR*bRp*l0h5 + 7385*Arho*bR*Power(bRp,2)*l0h5 + 8820*Arho*bLp*epsp*l0h5 + 196980*Arho*bRp*epsp*l0h5 + 235200*Arho*bRp*eps*epsp*l0h5 - 768*Power(aL,2)*EA*Power(1 + eps,2)*(1136*aR + 276*bL*l0 - 639*bR*l0) + 7840*aLp*Arho*bR*l0h4*phiSp + 57400*Arho*aRp*bR*l0h4*phiSp - 2310*Arho*bLp*bR*l0h5*phiSp + 188160*Arho*epsp*l0h5*phiSp + 235200*Arho*eps*epsp*l0h5*phiSp - 7700*Arho*bR*l0h5*Power(phiSp,2) - 6*bL*l0*(1411200*EI + l0h2*(144*EA*(1 + eps)*(140*eps + 27*Power(bR,2)*(1 + eps)) + 35*Arho*l0*(8*aLp*(bRp + phiSp) + 68*aRp*(bRp + phiSp) + l0*(-7*Power(bLp,2) - 3*bLp*bRp + 7*Power(bRp,2) + 11*bLp*phiSp - 14*Power(phiSp,2))))) + 4*aL*(6209280*EI + 102912*Power(aR,2)*EA*Power(1 + eps,2) + 5184*aR*(9*bL - 8*bR)*EA*Power(1 + eps,2)*l0 + l0h2*(432*EA*(1 + eps)*(140*eps + (5*bL - 9*bR)*(5*bL - 3*bR)*(1 + eps)) + 35*Arho*l0*(32*aLp*(bRp + phiSp) + 272*aRp*(bRp + phiSp) + l0*(51*Power(bLp,2) + 28*Power(bRp,2) + 23*Power(phiSp,2) - 6*bLp*(2*bRp + 19*phiSp))))) - 4*aR*(24272640*EI + l0h2*(288*EA*(1 + eps)*(560*eps + 3*(15*Power(bL,2) - 25*bL*bR + 36*Power(bR,2))*(1 + eps)) - 35*Arho*l0*(272*aLp*(bRp + phiSp) + 6272*aRp*(bRp + phiSp) + l0*(6*Power(bLp,2) + 205*Power(bRp,2) - 199*Power(phiSp,2) - 6*bLp*(17*bRp + 19*phiSp))))) - 2822400*EI*l0h2*wss0 - 5880*Arho*l0h3*(8*(aL + 13*aR)*gx*cos(bR + phiS) + 3*l0*(gy*cos(bL - phiS) + (-(bL*gx) + 5*bR*gx + 15*gy + 20*eps*gy)*cos(bR + phiS) + gx*sin(bL - phiS)) + (8*(aL + 13*aR)*gy - 3*(5*(3 + 4*eps)*gx + (bL - 5*bR)*gy)*l0)*sin(bR + phiS)))/(2.8224e6*l0h2);


    // Daempfung------------------------------------------------------------------
    //    hdLokal.init(0.0); beim Initialisieren
    hdLokal(3) = - depsilon * epsp; // eps

    //     cout <<endl<< trans(qElement) <<endl<< trans(qLokal) << endl;
    //     cout << MLokal << endl;

    // ab ins globale System
    M << JTMJ(MLokal,Jeg);// trans(Jeg) * MLokal * Jeg );

    Vec hZwischen(8,fmatvec::INIT,0.0);
    hZwischen << hLokal + hdLokal  -   MLokal * Jegp * qpElement;
    h << trans(Jeg) * hZwischen;

    // für die Impliziten Integratoren
    if(implicit)
    {
      Mat Dhz(16,8,fmatvec::INIT,0.0);
      Dhz   = hFullJacobi(qElement,qpElement,qLokal,qpLokal,Jeg,Jegp,MLokal,hZwischen);
      Dhq  << static_cast<SqrMat>(Dhz(0,0, 7,7));
      Dhqp << static_cast<SqrMat>(Dhz(8,0,15,7));
      Dhqp += trans(Jeg)*Damp*Jeg;
    }
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------

  // Bestimmung der lokalen Koordinaten aus qElement
  void FiniteElement1s21RCM::BuildqLokal(const Vec& qElement,Vec& qLokal)
  {
    //--- globale Koordinaten
	const double &x1   = qElement(0);    const double &y1  = qElement(1);    
	const double &phi1 = qElement(2);    
	const double &a1   = qElement(3);    const double &a2  = qElement(4);
	const double &x2   = qElement(5);    const double &y2  = qElement(6);
	const double &phi2 = qElement(7);

    // Rechenaufwand senken durch "Zentralisierung"
    double dphih, sphih;
    dphih = (phi1 - phi2)/2.0;
    sphih = (phi1 + phi2)/2.0;
    //    one_p_cos_dphi = (1 + cos(phi1 - phi2));

    //lokale Koordinate
    qLokal(0) = (Sec(dphih)*(36*(x1 + x2)*cos(dphih) + 36*(y1 - y2)*sin(dphih) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*sin(sphih)))/72.;
    qLokal(1) = ((64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(sphih)*Sec(dphih) + 36*(y1 + y2 + (-x1 + x2)*tan(dphih)))/72.;
    qLokal(2) = (2*(-8*a1 + 8*a2 + l0*2.0*sphih) + 11*Sec(dphih)*((-y1 + y2)*cos(sphih) + (x1 - x2)*sin(sphih)))/(4.*l0);
    qLokal(3) = -(Sec(dphih)*(36*l0*cos(dphih) + 36*(x1 - x2)*cos(sphih) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*sin(dphih) + 36*(y1 - y2)*sin(sphih)))/(36.*l0);
    qLokal(4) = (64*a1 + 64*a2 + 5*l0*(-2.*dphih) + 36*Sec(dphih)*((-y1 + y2)*cos(sphih) + (x1 - x2)*sin(sphih)))/72.;
    qLokal(5) = (2*(-8*a1 + 8*a2 + l0*(-2.*dphih)) + 11*Sec(dphih)*((-y1 + y2)*cos(sphih) + (x1 - x2)*sin(sphih)))/(4.*l0);
    qLokal(6) = (64*a1 + 64*a2 + 5*l0*(-2.*dphih)  + 36*Sec(dphih)*((y1 - y2)*cos(sphih) + (-x1 + x2)*sin(sphih)))/72.;
    qLokal(7) = (2*(8*a1 - 8*a2 +  l0*(-2.*dphih)) + 11*Sec(dphih)*((y1 - y2)*cos(sphih) + (-x1 + x2)*sin(sphih)))/(4.*l0);

  }

  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------

  // Bestimmung der Jacobimatrix und ihres zeitl. Diff aus qElement und qpElement
  void FiniteElement1s21RCM::BuildJacobi(const Vec& qElement, SqrMat& Jeg)
  {
    //--- globale Koordinaten
    const double &x1   = qElement(0);     const double &y1  = qElement(1);    
    const double &phi1 = qElement(2);    
    const double &a1   = qElement(3);     const double &a2  = qElement(4);
    const double &x2   = qElement(5);     const double &y2  = qElement(6);
    const double &phi2 = qElement(7);

    // Rechenaufwand senken durch "Zentralisierung"
    double one_p_cos_dphi;
    one_p_cos_dphi = (1 + cos(phi1 - phi2));

    //JacobiMatrix
    Jeg(0,0) = 0.5;
    Jeg(0,1) = tan((phi1 - phi2)/2.)/2.;
    Jeg(0,2) = (Power(Sec((phi1 - phi2)/2.),2)*(36*(y1 - y2) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi2) + 5*l0*(sin(phi1) + sin(phi2))))/144.;
    Jeg(0,3) = (-8*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/9.;
    Jeg(0,4) = (-8*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/9.;
    Jeg(0,5) = 0.5;
    Jeg(0,6) = -tan((phi1 - phi2)/2.)/2.;
    Jeg(0,7) = -(36*(y1 - y2) + (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi1) + 5*l0*(sin(phi1) + sin(phi2)))/(72.*one_p_cos_dphi);
    Jeg(1,0) = -tan((phi1 - phi2)/2.)/2.;
    Jeg(1,1) = 0.5;
    Jeg(1,2) = (36*(-x1 + x2) - 64*(a1 + a2)*sin(phi2) - 5*l0*(cos(phi1) + cos(phi2) + (-phi1 + phi2)*sin(phi2)))/(72.*one_p_cos_dphi);
    Jeg(1,3) = (8*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/9.;
    Jeg(1,4) = (8*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/9.;
    Jeg(1,5) = tan((phi1 - phi2)/2.)/2.;
    Jeg(1,6) = 0.5;
    Jeg(1,7) = (36*(x1 - x2) - 64*(a1 + a2)*sin(phi1) + 5*l0*(cos(phi1) + cos(phi2) + (phi1 - phi2)*sin(phi1)))/(72.*one_p_cos_dphi);
    Jeg(2,0) = (11*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/(4.*l0);
    Jeg(2,1) = (-11*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/(4.*l0);
    Jeg(2,2) = (2*l0 + 2*l0*cos(phi1 - phi2) + 11*(x1 - x2)*cos(phi2) + 11*(y1 - y2)*sin(phi2))/(4.*l0*one_p_cos_dphi);
    Jeg(2,3) = -4/l0;
    Jeg(2,4) = 4/l0;
    Jeg(2,5) = (-11*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/(4.*l0);
    Jeg(2,6) = (11*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/(4.*l0);
    Jeg(2,7) = (4*l0 + (22*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1)))/one_p_cos_dphi)/(8.*l0);
    Jeg(3,0) = -((cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/l0);
    Jeg(3,1) = -((Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/l0);
    Jeg(3,2) = (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2 + 36*(-y1 + y2)*cos(phi2) - 5*l0*sin(phi1 - phi2) + 36*(x1 - x2)*sin(phi2))/(36.*l0*one_p_cos_dphi);
    Jeg(3,3) = (16*tan((phi1 - phi2)/2.))/(9.*l0);
    Jeg(3,4) = (16*tan((phi1 - phi2)/2.))/(9.*l0);
    Jeg(3,5) = (cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/l0;
    Jeg(3,6) = (Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/l0;
    Jeg(3,7) = (-64*a1 - 64*a2 + 5*l0*phi1 - 5*l0*phi2 + 36*(-y1 + y2)*cos(phi1) + 36*(x1 - x2)*sin(phi1) + 5*l0*sin(phi1 - phi2))/(36.*l0*one_p_cos_dphi);
    Jeg(4,0) = (Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/2.;
    Jeg(4,1) = -(cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/2.;
    Jeg(4,2) = -(5*l0 + 5*l0*cos(phi1 - phi2) + 36*(-x1 + x2)*cos(phi2) + 36*(-y1 + y2)*sin(phi2))/(72.*one_p_cos_dphi);
    Jeg(4,3) = 0.8888888888888888;
    Jeg(4,4) = 0.8888888888888888;
    Jeg(4,5) = -(Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/2.;
    Jeg(4,6) = (cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/2.;
    Jeg(4,7) = (10*l0 + (72*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1)))/one_p_cos_dphi)/144.;
    Jeg(5,0) = (11*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/(4.*l0);
    Jeg(5,1) = (-11*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/(4.*l0);
    Jeg(5,2) = -(2*l0 + 2*l0*cos(phi1 - phi2) + 11*(-x1 + x2)*cos(phi2) + 11*(-y1 + y2)*sin(phi2))/(4.*l0*one_p_cos_dphi);
    Jeg(5,3) = -4/l0;
    Jeg(5,4) = 4/l0;
    Jeg(5,5) = (-11*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/(4.*l0);
    Jeg(5,6) = (11*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/(4.*l0);
    Jeg(5,7) = (4*l0 + (22*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1)))/one_p_cos_dphi)/(8.*l0);
    Jeg(6,0) = -(Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/2.;
    Jeg(6,1) = (cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/2.;
    Jeg(6,2) = -(5*l0 + 5*l0*cos(phi1 - phi2) + 36*(x1 - x2)*cos(phi2) + 36*(y1 - y2)*sin(phi2))/(72.*one_p_cos_dphi);
    Jeg(6,3) = 0.8888888888888888;
    Jeg(6,4) = 0.8888888888888888;
    Jeg(6,5) = (Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/2.;
    Jeg(6,6) = -(cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/2.;
    Jeg(6,7) = (10*l0 - (72*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1)))/one_p_cos_dphi)/144.;
    Jeg(7,0) = (-11*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/(4.*l0);
    Jeg(7,1) = (11*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/(4.*l0);
    Jeg(7,2) = -(2*l0 + 2*l0*cos(phi1 - phi2) + 11*(x1 - x2)*cos(phi2) + 11*(y1 - y2)*sin(phi2))/(4.*l0*one_p_cos_dphi);
    Jeg(7,3) = 4/l0;
    Jeg(7,4) = -4/l0;
    Jeg(7,5) = (11*Sec((phi1 - phi2)/2.)*sin((phi1 + phi2)/2.))/(4.*l0);
    Jeg(7,6) = (-11*cos((phi1 + phi2)/2.)*Sec((phi1 - phi2)/2.))/(4.*l0);
    Jeg(7,7) = (4*l0 - (22*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1)))/one_p_cos_dphi)/(8.*l0);
  }

  //------------------------------------------------------------------------------
  // Bestimmung der Jacobimatrix und ihres zeitl. Diff aus qElement und qpElement
  void FiniteElement1s21RCM::BuildJacobi(const Vec& qElement, const Vec& qpElement, SqrMat& Jeg, SqrMat& Jegp)
  {
    //--- globale Koordinaten, Geschwingigkeiten
    const double &x1   = qElement(0);     const double &y1  = qElement(1);    
    const double &phi1 = qElement(2);    
    const double &a1   = qElement(3);     const double &a2  = qElement(4);
    const double &x2   = qElement(5);     const double &y2  = qElement(6);
    const double &phi2 = qElement(7);
    
    const double &x1p   = qpElement(0);     const double &y1p  = qpElement(1);    
    const double &phi1p = qpElement(2);    
    const double &a1p   = qpElement(3);     const double &a2p  = qpElement(4);
    const double &x2p   = qpElement(5);     const double &y2p  = qpElement(6);
    const double &phi2p = qpElement(7);

    // Rechenaufwand senken durch "Zentralisierung"
    double one_p_cos_dphi;
    one_p_cos_dphi = (1 + cos(phi1 - phi2));

    BuildJacobi(qElement,Jeg);

    //Jegp(0,0) = 0;
    Jegp(0,1) = (phi1p - phi2p)/(2.*one_p_cos_dphi);
    Jegp(0,2) = (Power(Sec((phi1 - phi2)/2.),2)*(36*(y1p - y2p) - 64*(a1p + a2p)*cos(phi2) + 64*(a1 + a2)*phi2p*sin(phi2) + 5*l0*(phi1p*(cos(phi1) + cos(phi2)) + (-phi1 + phi2)*phi2p*sin(phi2)) + (phi1p - phi2p)*(36*(y1 - y2) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi2) + 5*l0*(sin(phi1) + sin(phi2)))*tan((phi1 - phi2)/2.)))/144.;
    Jegp(0,3) = (-8*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(9.*one_p_cos_dphi);
    Jegp(0,4) = (-8*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(9.*one_p_cos_dphi);
    //Jegp(0,5) = 0;
    Jegp(0,6) = (-phi1p + phi2p)/(2.*one_p_cos_dphi);
    Jegp(0,7) = -(one_p_cos_dphi*(36*(y1p - y2p) + (64*(a1p + a2p) + 5*l0*phi2p)*cos(phi1) - 64*(a1 + a2)*phi1p*sin(phi1) + 5*l0*(phi2p*cos(phi2) + phi1p*(phi1 - phi2)*sin(phi1))) + (phi1p - phi2p)*sin(phi1 - phi2)*(36*(y1 - y2) + (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi1) + 5*l0*(sin(phi1) + sin(phi2))))/(72.*Power(1 + cos(phi1 - phi2),2));
    Jegp(1,0) = (-phi1p + phi2p)/(2.*one_p_cos_dphi);
    //Jegp(1,1) = 0;
    Jegp(1,2) = (-((phi1p - phi2p)*sin(phi1 - phi2)*(36*(x1 - x2) + 64*(a1 + a2)*sin(phi2) + 5*l0*(cos(phi1) + cos(phi2) + (-phi1 + phi2)*sin(phi2)))) + one_p_cos_dphi*(36*(-x1p + x2p) - 64*(a1 + a2)*phi2p*cos(phi2) - 64*(a1p + a2p)*sin(phi2) + 5*l0*((phi1 - phi2)*phi2p*cos(phi2) + phi1p*(sin(phi1) + sin(phi2)))))/(72.*Power(1 + cos(phi1 - phi2),2));
    Jegp(1,3) = (-8*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(9.*one_p_cos_dphi);
    Jegp(1,4) = (-8*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(9.*one_p_cos_dphi);
    Jegp(1,5) = (phi1p - phi2p)/(2.*one_p_cos_dphi);
    //Jegp(1,6) = 0;
    Jegp(1,7) = ((phi1p - phi2p)*(36*(x1 - x2) - 64*(a1 + a2)*sin(phi1) + 5*l0*(cos(phi1) + cos(phi2) + (phi1 - phi2)*sin(phi1)))*sin(phi1 - phi2) + one_p_cos_dphi*(36*(x1p - x2p) - 64*(a1 + a2)*phi1p*cos(phi1) - 64*(a1p + a2p)*sin(phi1) + 5*l0*(phi1p*(phi1 - phi2)*cos(phi1) - phi2p*(sin(phi1) + sin(phi2)))))/(72.*Power(1 + cos(phi1 - phi2),2));
    Jegp(2,0) = (11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(2,1) = (11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(2,2) = (11*(cos(phi2)*((x1p - x2p + phi2p*y1 - phi2p*y2)*one_p_cos_dphi + (phi1p - phi2p)*(x1 - x2)*sin(phi1 - phi2)) + (-((phi2p*(x1 - x2) - y1p + y2p)*one_p_cos_dphi) + (phi1p - phi2p)*(y1 - y2)*sin(phi1 - phi2))*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    //Jegp(2,3) = 0;
    //Jegp(2,4) = 0;
    Jegp(2,5) = (-11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(2,6) = (-11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(2,7) = (11*(one_p_cos_dphi*((x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(-x1 + x2) + y1p - y2p)*sin(phi1)) + (phi1p - phi2p)*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1))*sin(phi1 - phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    Jegp(3,0) = (phi2p*sin(phi1) + phi1p*sin(phi2))/(l0 + l0*cos(phi1 - phi2));
    Jegp(3,1) = -((phi2p*cos(phi1) + phi1p*cos(phi2))/(l0 + l0*cos(phi1 - phi2)));
    Jegp(3,2) = ((phi1p - phi2p)*sin(phi1 - phi2)*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2 + 36*(-y1 + y2)*cos(phi2) - 5*l0*sin(phi1 - phi2) + 36*(x1 - x2)*sin(phi2)) + one_p_cos_dphi*(64*a1p + 64*a2p - 5*l0*phi1p + 5*l0*phi2p + 5*l0*(-phi1p + phi2p)*cos(phi1 - phi2) + 36*(phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(phi2) + 36*(x1p - x2p + phi2p*y1 - phi2p*y2)*sin(phi2)))/(36.*l0*Power(1 + cos(phi1 - phi2),2));
    Jegp(3,3) = (8*(phi1p - phi2p)*Power(Sec((phi1 - phi2)/2.),2))/(9.*l0);
    Jegp(3,4) = (8*(phi1p - phi2p)*Power(Sec((phi1 - phi2)/2.),2))/(9.*l0);
    Jegp(3,5) = -((phi2p*sin(phi1) + phi1p*sin(phi2))/(l0 + l0*cos(phi1 - phi2)));
    Jegp(3,6) = (phi2p*cos(phi1) + phi1p*cos(phi2))/(l0 + l0*cos(phi1 - phi2));
    Jegp(3,7) = (one_p_cos_dphi*(-64*a1p - 64*a2p + 5*l0*phi1p - 5*l0*phi2p + 36*(phi1p*(x1 - x2) - y1p + y2p)*cos(phi1) + 5*l0*(phi1p - phi2p)*cos(phi1 - phi2) + 36*(x1p - x2p + phi1p*y1 - phi1p*y2)*sin(phi1)) + (phi1p - phi2p)*sin(phi1 - phi2)*(-64*a1 - 64*a2 + 5*l0*phi1 - 5*l0*phi2 + 36*(-y1 + y2)*cos(phi1) + 36*(x1 - x2)*sin(phi1) + 5*l0*sin(phi1 - phi2)))/(36.*l0*Power(1 + cos(phi1 - phi2),2));
    Jegp(4,0) = (phi2p*cos(phi1) + phi1p*cos(phi2))/(2.*one_p_cos_dphi);
    Jegp(4,1) = (phi2p*sin(phi1) + phi1p*sin(phi2))/(2.*one_p_cos_dphi);
    Jegp(4,2) = (cos(phi2)*((x1p - x2p + phi2p*y1 - phi2p*y2)*one_p_cos_dphi + (phi1p - phi2p)*(x1 - x2)*sin(phi1 - phi2)) + (-((phi2p*(x1 - x2) - y1p + y2p)*one_p_cos_dphi) + (phi1p - phi2p)*(y1 - y2)*sin(phi1 - phi2))*sin(phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    //Jegp(4,3) = 0;
    //Jegp(4,4) = 0;
    Jegp(4,5) = -(phi2p*cos(phi1) + phi1p*cos(phi2))/(2.*one_p_cos_dphi);
    Jegp(4,6) = -(phi2p*sin(phi1) + phi1p*sin(phi2))/(2.*one_p_cos_dphi);
    Jegp(4,7) = (one_p_cos_dphi*((x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(-x1 + x2) + y1p - y2p)*sin(phi1)) + (phi1p - phi2p)*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1))*sin(phi1 - phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    Jegp(5,0) = (11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(5,1) = (11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(5,2) = (11*(cos(phi2)*((x1p - x2p + phi2p*y1 - phi2p*y2)*one_p_cos_dphi + (phi1p - phi2p)*(x1 - x2)*sin(phi1 - phi2)) + (-((phi2p*(x1 - x2) - y1p + y2p)*one_p_cos_dphi) + (phi1p - phi2p)*(y1 - y2)*sin(phi1 - phi2))*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    //Jegp(5,3) = 0;
    //Jegp(5,4) = 0;
    Jegp(5,5) = (-11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(5,6) = (-11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(5,7) = (11*(one_p_cos_dphi*((x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(-x1 + x2) + y1p - y2p)*sin(phi1)) + (phi1p - phi2p)*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1))*sin(phi1 - phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    Jegp(6,0) = -(phi2p*cos(phi1) + phi1p*cos(phi2))/(2.*one_p_cos_dphi);
    Jegp(6,1) = -(phi2p*sin(phi1) + phi1p*sin(phi2))/(2.*one_p_cos_dphi);
    Jegp(6,2) = (cos(phi2)*(-((x1p - x2p + phi2p*y1 - phi2p*y2)*one_p_cos_dphi) + (-phi1p + phi2p)*(x1 - x2)*sin(phi1 - phi2)) + ((phi2p*(x1 - x2) - y1p + y2p)*one_p_cos_dphi + (-phi1p + phi2p)*(y1 - y2)*sin(phi1 - phi2))*sin(phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    //Jegp(6,3) = 0;
    //Jegp(6,4) = 0;
    Jegp(6,5) = (phi2p*cos(phi1) + phi1p*cos(phi2))/(2.*one_p_cos_dphi);
    Jegp(6,6) = (phi2p*sin(phi1) + phi1p*sin(phi2))/(2.*one_p_cos_dphi);
    Jegp(6,7) = (one_p_cos_dphi*((-x1p + x2p - phi1p*y1 + phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - y1p + y2p)*sin(phi1)) - (phi1p - phi2p)*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1))*sin(phi1 - phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    Jegp(7,0) = (-11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(7,1) = (-11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(7,2) = (-11*(cos(phi2)*((x1p - x2p + phi2p*y1 - phi2p*y2)*one_p_cos_dphi + (phi1p - phi2p)*(x1 - x2)*sin(phi1 - phi2)) + (-((phi2p*(x1 - x2) - y1p + y2p)*one_p_cos_dphi) + (phi1p - phi2p)*(y1 - y2)*sin(phi1 - phi2))*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    //Jegp(7,3) = 0;
    //Jegp(7,4) = 0;
    Jegp(7,5) = (11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(7,6) = (11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(4.*l0*one_p_cos_dphi);
    Jegp(7,7) = (11*(one_p_cos_dphi*((-x1p + x2p - phi1p*y1 + phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - y1p + y2p)*sin(phi1)) - (phi1p - phi2p)*((x1 - x2)*cos(phi1) + (y1 - y2)*sin(phi1))*sin(phi1 - phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));

  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------------------------------------------------------------------------------------

  // Balkenort ermitteln aus lokalen Lagen 
//  Vec FiniteElement1s21RCM::LocateLokalBalken(const Vec& qLokal, Vec& qpLokal, double& s)
//  {
//    return LocateLokalBalken( qLokal, qpLokal, s, true );
//  }
  // Balkenort ermitteln aus lokalen Lagen 
  Vec FiniteElement1s21RCM::LocateLokalBalken(const Vec& qLokal, const Vec& qpLokal, const double& s, bool calcAll)
  {
    Vec X(6); // x,y und phi | und | xp,yp und phip !!!

    //--- lokale  Koordinaten, Geschwingigkeiten
    //     double &xS     = qLokal(0);      double &yS    = qLokal(1);
    //     double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    //     double &aL     = qLokal(4);      double &bL    = qLokal(5);
    //     double &aR     = qLokal(6);      double &bR    = qLokal(7);

    // Aenderung 30.11.2005 - keine static Definitionen mehr
    const double &xS     = qLokal(0);      const double &yS    = qLokal(1);
    const double &phiS   = qLokal(2);      const double &eps   = qLokal(3);
    const double &aL     = qLokal(4);      const double &bL    = qLokal(5);
    const double &aR     = qLokal(6);      const double &bR    = qLokal(7);

    const double &xSp    = qpLokal(0);     const double &ySp   = qpLokal(1);
    const double &phiSp  = qpLokal(2);     const double &epsp  = qpLokal(3);
    const double &aLp    = qpLokal(4);     const double &bLp   = qpLokal(5);
    const double &aRp    = qpLokal(6);     const double &bRp   = qpLokal(7);

    if (s < 0) // linker Teilbereich
    {
      X(0) = xS + (1 + eps)*s*cos(bL - phiS) + (s*(bL*l0*(2*l0 - 5*s)*Power(l0 + 2*s,2) + s*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*sin(bL - phiS))/(2.*l0h4);
      X(1) = yS + (s*(bL*l0*(2*l0 - 5*s)*Power(l0 + 2*s,2) + s*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*cos(bL - phiS))/(2.*l0h4) - (1 + eps)*s*sin(bL - phiS);
      X(2) = phiS + (s*(l0h2*(-8*(aL + aR) + 3*(bL + bR)*l0) - 6*l0*(-8*aL + 8*aR + 3*(bL - bR)*l0)*s + 8*(16*aL - 8*aR - 5*bL*l0 + 3*bR*l0)*Power(s,2)))/l0h4;

      if(calcAll) {
	X(3) = xSp + (s*((2*epsp*l0h4 + bL*l0*(bLp - phiSp)*(2*l0 - 5*s)*Power(l0 + 2*s,2) + (bLp - phiSp)*s*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*cos(bL - phiS) + (2*l0h4*(-(bLp*eps) + phiSp + eps*phiSp) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s - 4*l0*(-8*aLp + 8*aRp + 3*(bLp - bRp)*l0)*Power(s,2) + 4*(16*aLp - 8*aRp - 5*bLp*l0 + 3*bRp*l0)*Power(s,3))*sin(bL - phiS)))/(2.*l0h4);
	X(4) = ySp + (s*(2*l0h4*(-(bLp*eps) + phiSp + eps*phiSp) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s - 4*l0*(-8*aLp + 8*aRp + 3*(bLp - bRp)*l0)*Power(s,2) + 4*(16*aLp - 8*aRp - 5*bLp*l0 + 3*bRp*l0)*Power(s,3))*cos(bL - phiS) + s*(-2*epsp*l0h4 - bL*l0*(bLp - phiSp)*(2*l0 - 5*s)*Power(l0 + 2*s,2) + (bLp - phiSp)*s*((8*aR - 3*bR*l0)*Power(l0 + 2*s,2) + 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*sin(bL - phiS))/(2.*l0h4);
	X(5) = phiSp + (3*(bLp + bRp)*l0h3*s + 64*(2*aLp - aRp)*Power(s,3) - 2*l0h2*s*(4*aLp + 4*aRp + 9*bLp*s - 9*bRp*s) + 8*l0*Power(s,2)*(6*aLp - 6*aRp - 5*bLp*s + 3*bRp*s))/l0h4;
      }
    }
    else       // rechter Teilbereich
    {
      X(0) = xS + (1 + eps)*s*cos(bR + phiS) + (s*(bR*l0*Power(l0 - 2*s,2)*(2*l0 + 5*s) + s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*sin(bR + phiS))/(2.*l0h4);
      X(1) = yS - (s*(bR*l0*Power(l0 - 2*s,2)*(2*l0 + 5*s) + s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*cos(bR + phiS))/(2.*l0h4) + (1 + eps)*s*sin(bR + phiS);
      X(2) = phiS + (s*(l0h2*(-8*(aL + aR) + 3*(bL + bR)*l0) - 6*l0*(-8*aL + 8*aR + 3*(bL - bR)*l0)*s - 8*(8*aL - 16*aR - 3*bL*l0 + 5*bR*l0)*Power(s,2)))/l0h4;

      if(calcAll) {
	X(3) = xSp + (s*((2*epsp*l0h4 + bR*l0*(bRp + phiSp)*Power(l0 - 2*s,2)*(2*l0 + 5*s) + (bRp + phiSp)*s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*cos(bR + phiS) - (2*l0h4*(phiSp + eps*(bRp + phiSp)) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s + 4*l0*(8*aLp - 8*aRp + 3*(-bLp + bRp)*l0)*Power(s,2) - 4*(8*aLp - 16*aRp - 3*bLp*l0 + 5*bRp*l0)*Power(s,3))*sin(bR + phiS)))/(2.*l0h4);
	X(4) = ySp + (s*(2*l0h4*(phiSp + eps*(bRp + phiSp)) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s + 4*l0*(8*aLp - 8*aRp + 3*(-bLp + bRp)*l0)*Power(s,2) - 4*(8*aLp - 16*aRp - 3*bLp*l0 + 5*bRp*l0)*Power(s,3))*cos(bR + phiS) + s*(2*epsp*l0h4 + bR*l0*(bRp + phiSp)*Power(l0 - 2*s,2)*(2*l0 + 5*s) + (bRp + phiSp)*s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*sin(bR + phiS))/(2.*l0h4);
	X(5) = phiSp + (3*(bLp + bRp)*l0h3*s - 64*(aLp - 2*aRp)*Power(s,3) - 2*l0h2*s*(4*aLp + 4*aRp + 9*bLp*s - 9*bRp*s) + 8*l0*Power(s,2)*(6*aLp - 6*aRp + 3*bLp*s - 5*bRp*s))/l0h4;
      }
    }

    return X;
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  // Balkenort ermitteln aus globalen Lagen 
  Vec FiniteElement1s21RCM::LocateBalken(const Vec& qElement, const double& s)
  {
    Vec  qLokal     (8,fmatvec::INIT,0.0);
    Vec qpLokalDummy(8,fmatvec::INIT,0.0);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);

    return (LocateLokalBalken(qLokal,qpLokalDummy,s,false))(0,2);// Liefert nur einen Teil(Lagen) des Zustands (Geschwindigkeiten werden eh NICHT berechnet!!! gleich 0)
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  // Balkenzustand ermitteln aus globalen Lagen, Geschwindigkeit
  Vec FiniteElement1s21RCM::StateBalken(const Vec& qElement, const Vec& qpElement, const double& s)
  {
    Vec  qLokal(8,fmatvec::INIT,0.0);
    Vec qpLokal(8,fmatvec::INIT,0.0);
    SqrMat  Jeg(8,fmatvec::INIT,0.0);
    SqrMat Jegp(8,fmatvec::INIT,0.0);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);
    //JacobiMatrizen--------------------------------------------------------------
    BuildJacobi(qElement,qpElement,Jeg,Jegp);
    qpLokal = Jeg*qpElement;

    return LocateLokalBalken(qLokal,qpLokal,s,true);
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  // JacobiMatrix der Lage des Balkenquerschnittes bei s (dx/dq,dy/dq,dphi/dq)^T
  Mat FiniteElement1s21RCM::JGeneralizedInternal(const Vec& qElement,const double& s) {
    // Rueckgagewert J: JacobiMatrix
    Mat J(8,3);

    Vec  qLokal(8);
    //    Vec qpElementDummy(8,fmatvec::INIT,0.0);
    //lokale Koordinate
    BuildqLokal(qElement,qLokal);
//    double &xS     = qLokal(0);      double &yS    = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

    if( s < 0 ) // Linker Teilbereich
    {
      // Translation
      J(0,0) = 1;
      J(0,1) = 0;
      J(1,0) = 0;
      J(1,1) = 1;
      J(2,0) = (s*(-(bL*l0*(2*l0 - 5*s)*Power(l0 + 2*s,2)) + s*((8*aR - 3*bR*l0)*Power(l0 + 2*s,2) + 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*cos(bL - phiS))/(2.*l0h4) + (1 + eps)*s*sin(bL - phiS);
      J(2,1) = (1 + eps)*s*cos(bL - phiS) + (s*(bL*l0*(2*l0 - 5*s)*Power(l0 + 2*s,2) + s*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*sin(bL - phiS))/(2.*l0h4);
      J(3,0) = s*cos(bL - phiS);
      J(3,1) = -(s*sin(bL - phiS));
      J(4,0) = (-4*Power(s,2)*(l0h2 - 4*l0*s - 8*Power(s,2))*sin(bL - phiS))/l0h4;
      J(4,1) = (-4*Power(s,2)*(l0h2 - 4*l0*s - 8*Power(s,2))*cos(bL - phiS))/l0h4;
      J(5,0) = (s*(((bL*l0*(2*l0 - 5*s)*Power(l0 + 2*s,2) + s*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*cos(bL - phiS))/l0h4 - 2*(1 + eps)*sin(bL - phiS) + ((2*l0 - 5*s)*Power(l0 + 2*s,2)*sin(bL - phiS))/l0h3))/2.;
      J(5,1) = (s*(-2*(1 + eps)*cos(bL - phiS) + ((2*l0 - 5*s)*Power(l0 + 2*s,2)*cos(bL - phiS))/l0h3 + ((-(bL*l0*(2*l0 - 5*s)*Power(l0 + 2*s,2)) + s*((8*aR - 3*bR*l0)*Power(l0 + 2*s,2) + 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))))*sin(bL - phiS))/l0h4))/2.;
      J(6,0) = (-4*Power(s,2)*Power(l0 + 2*s,2)*sin(bL - phiS))/l0h4;
      J(6,1) = (-4*Power(s,2)*Power(l0 + 2*s,2)*cos(bL - phiS))/l0h4;
      J(7,0) = (3*Power(s,2)*Power(l0 + 2*s,2)*sin(bL - phiS))/(2.*l0h3);
      J(7,1) = (3*Power(s,2)*Power(l0 + 2*s,2)*cos(bL - phiS))/(2.*l0h3);
      // Rotation
      J(0,2) = 0;
      J(1,2) = 0;
      J(2,2) = 1;
      J(3,2) = 0;
      J(4,2) = (-8*(l0 - 8*s)*s*(l0 + 2*s))/l0h4;
      J(5,2) = (s*(3*l0h2 - 18*l0*s - 40*Power(s,2)))/l0h3;
      J(6,2) = (-8*s*(l0 + 2*s)*(l0 + 4*s))/l0h4;
      J(7,2) = (3*s*(l0 + 2*s)*(l0 + 4*s))/l0h3;
    }
    else
    {
      // Translation
      J(0,0) = 1;
      J(0,1) = 0;
      J(1,0) = 0;
      J(1,1) = 1;
      J(2,0) = (s*(bR*l0*Power(l0 - 2*s,2)*(2*l0 + 5*s) + s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*cos(bR + phiS))/(2.*l0h4) - (1 + eps)*s*sin(bR + phiS);
      J(2,1) = (1 + eps)*s*cos(bR + phiS) + (s*(bR*l0*Power(l0 - 2*s,2)*(2*l0 + 5*s) + s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*sin(bR + phiS))/(2.*l0h4);
      J(3,0) = s*cos(bR + phiS);
      J(3,1) = s*sin(bR + phiS);
      J(4,0) = (4*Power(l0 - 2*s,2)*Power(s,2)*sin(bR + phiS))/l0h4;
      J(4,1) = (-4*Power(l0 - 2*s,2)*Power(s,2)*cos(bR + phiS))/l0h4;
      J(5,0) = (-3*Power(l0 - 2*s,2)*Power(s,2)*sin(bR + phiS))/(2.*l0h3);
      J(5,1) = (3*Power(l0 - 2*s,2)*Power(s,2)*cos(bR + phiS))/(2.*l0h3);
      J(6,0) = (4*Power(s,2)*(l0h2 + 4*l0*s - 8*Power(s,2))*sin(bR + phiS))/l0h4;
      J(6,1) = (-4*Power(s,2)*(l0h2 + 4*l0*s - 8*Power(s,2))*cos(bR + phiS))/l0h4;
      J(7,0) = (s*((bR*l0*Power(l0 - 2*s,2)*(2*l0 + 5*s) + s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*cos(bR + phiS) - l0*(2*eps*l0h3 + s*(3*l0h2 + 12*l0*s - 20*Power(s,2)))*sin(bR + phiS)))/(2.*l0h4);
      J(7,1) = (s*(l0*(2*eps*l0h3 + s*(3*l0h2 + 12*l0*s - 20*Power(s,2)))*cos(bR + phiS) + (bR*l0*Power(l0 - 2*s,2)*(2*l0 + 5*s) + s*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))))*sin(bR + phiS)))/(2.*l0h4);
      // Rotation
      J(0,2) = 0;
      J(1,2) = 0;
      J(2,2) = 1;
      J(3,2) = 0;
      J(4,2) = (-8*(l0 - 4*s)*(l0 - 2*s)*s)/l0h4;
      J(5,2) = (3*(l0 - 4*s)*(l0 - 2*s)*s)/l0h3;
      J(6,2) = (-8*(l0 - 2*s)*s*(l0 + 8*s))/l0h4;
      J(7,2) = (s*(3*l0h2 + 18*l0*s - 40*Power(s,2)))/l0h3;
    }
    return J;
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  // JacobiMatrix der Lage des Balkenquerschnittes bei s (dx/dq,dy/dq,dphi/dq)^T in internen Koordinaten
  Mat FiniteElement1s21RCM::JGeneralized(const Vec& qElement,const double& s) {
    SqrMat Jeg(8); // vermutlich schneller (mal wieder) als gleich die Transformierte zu implementieren
    BuildJacobi(qElement,Jeg);
    return trans(Jeg)*JGeneralizedInternal(qElement,s);
  }
  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  // zeitliche Ableitung der JacobiMatrix der Lage des Balkenquerschnittes bei s (dx/dq,dy/dq,dphi/dq)^T
  Mat FiniteElement1s21RCM::JpGeneralized(const Vec& qElement, const Vec& qpElement,const double& s, const double& sp) {
    // Rueckgagewert J: JacobiMatrix
    Mat Jp(8,3,NONINIT);

    Vec  qLokal(8,fmatvec::NONINIT);
    Vec qpLokal(8,fmatvec::NONINIT);
    SqrMat  Jeg(8,fmatvec::INIT,0.0);
    SqrMat Jegp(8,fmatvec::INIT,0.0);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);
    //JacobiMatrizen--------------------------------------------------------------
    BuildJacobi(qElement,qpElement,Jeg,Jegp);
    qpLokal = Jeg*qpElement;

//    double &xS     = qLokal(0);      double &yS    = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);
//    double &xSp    = qpLokal(0);     double &ySp   = qpLokal(1);
    double &phiSp  = qpLokal(2);     double &epsp  = qpLokal(3);
    double &aLp    = qpLokal(4);     double &bLp   = qpLokal(5);
    double &aRp    = qpLokal(6);     double &bRp   = qpLokal(7);

    if( s < 0 ) {// Linker Teilbereich
      //JTLp
      Jp(0,0) = 0;
      Jp(0,1) = 0;
      Jp(1,0) = 0;
      Jp(1,1) = 0;
      Jp(2,0) = ((s*(2*l0h4*(bLp*eps - (1 + eps)*phiSp) - l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s + 4*l0*(-8*aLp + 8*aRp + 3*(bLp - bRp)*l0)*Power(s,2) + 4*(-16*aLp + 8*aRp + 5*bLp*l0 - 3*bRp*l0)*Power(s,3)) - 2*(l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sp)*cos(bL - phiS) + (2*epsp*l0h4*s + bL*l0*(bLp - phiSp)*(2*l0 - 5*s)*s*Power(l0 + 2*s,2) + (bLp - phiSp)*Power(s,2)*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))) + 2*(1 + eps)*l0h4*sp)*sin(bL - phiS))/(2.*l0h4);
      Jp(2,1) = ((2*epsp*l0h4*s + bL*l0*(bLp - phiSp)*(2*l0 - 5*s)*s*Power(l0 + 2*s,2) + (bLp - phiSp)*Power(s,2)*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))) + 2*(1 + eps)*l0h4*sp)*cos(bL - phiS) + (s*(2*l0h4*(-(bLp*eps) + phiSp + eps*phiSp) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s - 4*l0*(-8*aLp + 8*aRp + 3*(bLp - bRp)*l0)*Power(s,2) + 4*(16*aLp - 8*aRp - 5*bLp*l0 + 3*bRp*l0)*Power(s,3)) + 2*(l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sp)*sin(bL - phiS))/(2.*l0h4);
      Jp(3,0) = sp*cos(bL - phiS) + (-bLp + phiSp)*s*sin(bL - phiS);
      Jp(3,1) = (-bLp + phiSp)*s*cos(bL - phiS) - sp*sin(bL - phiS);
      Jp(4,0) = (4*s*((bLp - phiSp)*s*(-l0h2 + 4*l0*s + 8*Power(s,2))*cos(bL - phiS) - 2*(l0 - 8*s)*(l0 + 2*s)*sp*sin(bL - phiS)))/l0h4;
      Jp(4,1) = (4*s*(-2*(l0 - 8*s)*(l0 + 2*s)*sp*cos(bL - phiS) + (bLp - phiSp)*s*(l0h2 - 4*l0*s - 8*Power(s,2))*sin(bL - phiS)))/l0h4;
      Jp(5,0) = ((s*(-2*l0h4*(bLp*(-1 + eps) - eps*phiSp) + l0h2*(-8*aLp - 8*aRp + 3*l0*(2*bLp + bRp - phiSp))*s - 4*l0*(-8*aLp + 8*aRp + 6*bLp*l0 - 3*bRp*l0 - 3*l0*phiSp)*Power(s,2) + 4*(16*aLp - 8*aRp + l0*(-10*bLp + 3*bRp + 5*phiSp))*Power(s,3)) + 2*(l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sp)*cos(bL - phiS) - (2*epsp*l0h4*s + bL*l0*(bLp - phiSp)*(2*l0 - 5*s)*s*Power(l0 + 2*s,2) + (bLp - phiSp)*Power(s,2)*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))) + 2*l0*(eps*l0h3 + s*(-3*l0h2 + 18*l0*s + 40*Power(s,2)))*sp)*sin(bL - phiS))/(2.*l0h4);
      Jp(5,1) = (-((2*epsp*l0h4*s + bL*l0*(bLp - phiSp)*(2*l0 - 5*s)*s*Power(l0 + 2*s,2) + (bLp - phiSp)*Power(s,2)*(-((8*aR - 3*bR*l0)*Power(l0 + 2*s,2)) - 8*aL*(l0h2 - 4*l0*s - 8*Power(s,2))) + 2*l0*(eps*l0h3 + s*(-3*l0h2 + 18*l0*s + 40*Power(s,2)))*sp)*cos(bL - phiS)) + (s*(2*l0h4*(bLp*(-1 + eps) - eps*phiSp) - l0h2*(-8*aLp - 8*aRp + 3*l0*(2*bLp + bRp - phiSp))*s + 4*l0*(-8*aLp + 8*aRp + 6*bLp*l0 - 3*bRp*l0 - 3*l0*phiSp)*Power(s,2) - 4*(16*aLp - 8*aRp + l0*(-10*bLp + 3*bRp + 5*phiSp))*Power(s,3)) - 2*(l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sp)*sin(bL - phiS))/(2.*l0h4);
      Jp(6,0) = (-4*s*(l0 + 2*s)*((bLp - phiSp)*s*(l0 + 2*s)*cos(bL - phiS) + 2*(l0 + 4*s)*sp*sin(bL - phiS)))/l0h4;
      Jp(6,1) = (4*s*(l0 + 2*s)*(-2*(l0 + 4*s)*sp*cos(bL - phiS) + (bLp - phiSp)*s*(l0 + 2*s)*sin(bL - phiS)))/l0h4;
      Jp(7,0) = (3*s*(l0 + 2*s)*((bLp - phiSp)*s*(l0 + 2*s)*cos(bL - phiS) + 2*(l0 + 4*s)*sp*sin(bL - phiS)))/(2.*l0h3);
      Jp(7,1) = (3*s*(l0 + 2*s)*(2*(l0 + 4*s)*sp*cos(bL - phiS) - (bLp - phiSp)*s*(l0 + 2*s)*sin(bL - phiS)))/(2.*l0h3);
      //JRLp
      Jp(0,2) = 0;
      Jp(1,2) = 0;
      Jp(2,2) = 0;
      Jp(3,2) = 0;
      Jp(4,2) = (-8*(l0h2 - 12*l0*s - 48*Power(s,2))*sp)/l0h4;
      Jp(5,2) = (3*(l0h2 - 12*l0*s - 40*Power(s,2))*sp)/l0h3;
      Jp(6,2) = (-8*(l0h2 + 12*l0*s + 24*Power(s,2))*sp)/l0h4;
      Jp(7,2) = (3*(l0h2 + 12*l0*s + 24*Power(s,2))*sp)/l0h3;
    } else {
      //JTRp
      Jp(0,0) = 0;
      Jp(0,1) = 0;
      Jp(1,0) = 0;
      Jp(1,1) = 0;
      Jp(2,0) = -((s*(2*l0h4*(phiSp + eps*(bRp + phiSp)) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s + 4*l0*(8*aLp - 8*aRp + 3*(-bLp + bRp)*l0)*Power(s,2) - 4*(8*aLp - 16*aRp - 3*bLp*l0 + 5*bRp*l0)*Power(s,3)) - 2*(l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sp)*cos(bR + phiS) + (2*epsp*l0h4*s + bR*l0*(bRp + phiSp)*Power(l0 - 2*s,2)*s*(2*l0 + 5*s) + (bRp + phiSp)*Power(s,2)*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))) + 2*(1 + eps)*l0h4*sp)*sin(bR + phiS))/(2.*l0h4);
      Jp(2,1) = ((2*epsp*l0h4*s + bR*l0*(bRp + phiSp)*Power(l0 - 2*s,2)*s*(2*l0 + 5*s) + (bRp + phiSp)*Power(s,2)*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))) + 2*(1 + eps)*l0h4*sp)*cos(bR + phiS) - (s*(2*l0h4*(phiSp + eps*(bRp + phiSp)) + l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0)*s + 4*l0*(8*aLp - 8*aRp + 3*(-bLp + bRp)*l0)*Power(s,2) - 4*(8*aLp - 16*aRp - 3*bLp*l0 + 5*bRp*l0)*Power(s,3)) - 2*(l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sp)*sin(bR + phiS))/(2.*l0h4);
      Jp(3,0) = sp*cos(bR + phiS) - (bRp + phiSp)*s*sin(bR + phiS);
      Jp(3,1) = (bRp + phiSp)*s*cos(bR + phiS) + sp*sin(bR + phiS);
      Jp(4,0) = (4*(l0 - 2*s)*s*((bRp + phiSp)*(l0 - 2*s)*s*cos(bR + phiS) + 2*(l0 - 4*s)*sp*sin(bR + phiS)))/l0h4;
      Jp(4,1) = (4*(l0 - 2*s)*s*(-2*(l0 - 4*s)*sp*cos(bR + phiS) + (bRp + phiSp)*(l0 - 2*s)*s*sin(bR + phiS)))/l0h4;
      Jp(5,0) = (-3*(l0 - 2*s)*s*((bRp + phiSp)*(l0 - 2*s)*s*cos(bR + phiS) + 2*(l0 - 4*s)*sp*sin(bR + phiS)))/(2.*l0h3);
      Jp(5,1) = (3*(l0 - 2*s)*s*(2*(l0 - 4*s)*sp*cos(bR + phiS) + (bRp + phiSp)*s*(-l0 + 2*s)*sin(bR + phiS)))/(2.*l0h3);
      Jp(6,0) = (4*s*((bRp + phiSp)*s*(l0h2 + 4*l0*s - 8*Power(s,2))*cos(bR + phiS) + 2*(l0 - 2*s)*(l0 + 8*s)*sp*sin(bR + phiS)))/l0h4;
      Jp(6,1) = (4*s*(-2*(l0 - 2*s)*(l0 + 8*s)*sp*cos(bR + phiS) + (bRp + phiSp)*s*(l0h2 + 4*l0*s - 8*Power(s,2))*sin(bR + phiS)))/l0h4;
      Jp(7,0) = ((s*(-2*l0h4*(bRp*(-1 + eps) + eps*phiSp) - l0h2*(-8*aLp - 8*aRp + 3*l0*(bLp + 2*bRp + phiSp))*s + 4*l0*(-8*aLp + 8*aRp + 3*l0*(bLp - 2*bRp - phiSp))*Power(s,2) + 4*(8*aLp - 16*aRp + l0*(-3*bLp + 10*bRp + 5*phiSp))*Power(s,3)) + 2*(l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sp)*cos(bR + phiS) - (2*epsp*l0h4*s + bR*l0*(bRp + phiSp)*Power(l0 - 2*s,2)*s*(2*l0 + 5*s) + (bRp + phiSp)*Power(s,2)*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))) + 2*l0*(eps*l0h3 + s*(3*l0h2 + 18*l0*s - 40*Power(s,2)))*sp)*sin(bR + phiS))/(2.*l0h4);
      Jp(7,1) = ((2*epsp*l0h4*s + bR*l0*(bRp + phiSp)*Power(l0 - 2*s,2)*s*(2*l0 + 5*s) + (bRp + phiSp)*Power(s,2)*(8*aL*Power(l0 - 2*s,2) - 3*bL*l0*Power(l0 - 2*s,2) + 8*aR*(l0h2 + 4*l0*s - 8*Power(s,2))) + 2*l0*(eps*l0h3 + s*(3*l0h2 + 18*l0*s - 40*Power(s,2)))*sp)*cos(bR + phiS) + (s*(-2*l0h4*(bRp*(-1 + eps) + eps*phiSp) - l0h2*(-8*aLp - 8*aRp + 3*l0*(bLp + 2*bRp + phiSp))*s + 4*l0*(-8*aLp + 8*aRp + 3*l0*(bLp - 2*bRp - phiSp))*Power(s,2) + 4*(8*aLp - 16*aRp + l0*(-3*bLp + 10*bRp + 5*phiSp))*Power(s,3)) + 2*(l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sp)*sin(bR + phiS))/(2.*l0h4);
      //JRRp
      Jp(0,2) = 0;
      Jp(1,2) = 0;
      Jp(2,2) = 0;
      Jp(3,2) = 0;
      Jp(4,2) = (-8*(l0h2 - 12*l0*s + 24*Power(s,2))*sp)/l0h4;
      Jp(5,2) = (3*(l0h2 - 12*l0*s + 24*Power(s,2))*sp)/l0h3;
      Jp(6,2) = (-8*(l0h2 + 12*l0*s - 48*Power(s,2))*sp)/l0h4;
      Jp(7,2) = (3*(l0h2 + 12*l0*s - 40*Power(s,2))*sp)/l0h3;
    }
    return trans(Jeg)*Jp + trans(Jegp)*JGeneralizedInternal(qElement,s);
  }

  Vec FiniteElement1s21RCM::DrDs (Vec &qElement,const double &s) {
    Vec dr(2);
    Vec  qLokal(8,fmatvec::NONINIT);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);

//    double &xS     = qLokal(0);      double &yS    = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

    if (s < 0) {// linker Teilbereich
      //DrDsL
      dr(0) = ((1 + eps)*l0h4*cos(bL - phiS) + (l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sin(bL - phiS))/l0h4;
      dr(1) = ((l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*cos(bL - phiS) - (1 + eps)*l0h4*sin(bL - phiS))/l0h4;

    } else {
      //DrDsR
      dr(0) = ((1 + eps)*l0h4*cos(bR + phiS) + (l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sin(bR + phiS))/l0h4;
      dr(1) = (-((l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*cos(bR + phiS)) + (1 + eps)*l0h4*sin(bR + phiS))/l0h4;

    }
    return dr;
  }
  Vec FiniteElement1s21RCM::DrDsp(Vec &qElement,Vec &qpElement,const double &s,const double &sp) {
    Vec drp(2);
    Vec  qLokal(8,fmatvec::NONINIT);
    SqrMat  Jeg(8,fmatvec::INIT,0.0);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);
    BuildJacobi(qElement,Jeg);
    Vec qpLokal = Jeg*qpElement;

//    double &xS     = qLokal(0);      double &yS    = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

//    double &xSp    = qpLokal(0);     double &ySp   = qpLokal(1);
    double &phiSp  = qpLokal(2);     double &epsp  = qpLokal(3);
    double &aLp    = qpLokal(4);     double &bLp   = qpLokal(5);
    double &aRp    = qpLokal(6);     double &bRp   = qpLokal(7);

    if (s < 0) {// linker Teilbereich
      //DrDspL
      drp(0) = (epsp*l0h4*cos(bL - phiS) + (bLp - phiSp)*(l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*cos(bL - phiS) - (1 + eps)*l0h4*(bLp - phiSp)*sin(bL - phiS) + 2*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sp*sin(bL - phiS) + (l0 + 2*s)*(bLp*l0*(l0 - 4*s)*(l0 + 5*s) + 5*bL*l0*(l0 - 4*s)*sp - 4*bL*l0*(l0 + 5*s)*sp + (-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s))*sp + s*(-8*aLp*(l0 - 8*s) - (8*aRp - 3*bRp*l0)*(l0 + 4*s) + 64*aL*sp - 4*(8*aR - 3*bR*l0)*sp))*sin(bL - phiS))/l0h4;
      drp(1) = (-((1 + eps)*l0h4*(bLp - phiSp)*cos(bL - phiS)) + 2*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sp*cos(bL - phiS) + (l0 + 2*s)*(bLp*l0*(l0 - 4*s)*(l0 + 5*s) + 5*bL*l0*(l0 - 4*s)*sp - 4*bL*l0*(l0 + 5*s)*sp + (-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s))*sp + s*(-8*aLp*(l0 - 8*s) - (8*aRp - 3*bRp*l0)*(l0 + 4*s) + 64*aL*sp - 4*(8*aR - 3*bR*l0)*sp))*cos(bL - phiS) - epsp*l0h4*sin(bL - phiS) - (bLp - phiSp)*(l0 + 2*s)*(bL*l0*(l0 - 4*s)*(l0 + 5*s) + s*(-8*aL*(l0 - 8*s) - (8*aR - 3*bR*l0)*(l0 + 4*s)))*sin(bL - phiS))/l0h4;
    } else {
      //DrDspR
      drp(0) = (epsp*l0h4*cos(bR + phiS) + (bRp + phiSp)*(l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*cos(bR + phiS) - (1 + eps)*l0h4*(bRp + phiSp)*sin(bR + phiS) - 2*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sp*sin(bR + phiS) + (l0 - 2*s)*(bRp*l0*(l0 - 5*s)*(l0 + 4*s) + 4*bR*l0*(l0 - 5*s)*sp - 5*bR*l0*(l0 + 4*s)*sp + (8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s))*sp + s*(8*aLp*(l0 - 4*s) - 3*bLp*l0*(l0 - 4*s) + 8*aRp*(l0 + 8*s) - 32*aL*sp + 64*aR*sp + 12*bL*l0*sp))*sin(bR + phiS))/l0h4;
      drp(1) = ((1 + eps)*l0h4*(bRp + phiSp)*cos(bR + phiS) + 2*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sp*cos(bR + phiS) - (l0 - 2*s)*(bRp*l0*(l0 - 5*s)*(l0 + 4*s) + 4*bR*l0*(l0 - 5*s)*sp - 5*bR*l0*(l0 + 4*s)*sp + (8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s))*sp + s*(8*aLp*(l0 - 4*s) - 3*bLp*l0*(l0 - 4*s) + 8*aRp*(l0 + 8*s) - 32*aL*sp + 64*aR*sp + 12*bL*l0*sp))*cos(bR + phiS) + epsp*l0h4*sin(bR + phiS) + (bRp + phiSp)*(l0 - 2*s)*(bR*l0*(l0 - 5*s)*(l0 + 4*s) + s*(8*aL*(l0 - 4*s) - 3*bL*l0*(l0 - 4*s) + 8*aR*(l0 + 8*s)))*sin(bR + phiS))/l0h4;
    }
    return drp;
  }
  double FiniteElement1s21RCM::Kcurvature (Vec &qElement,const double &s) {
    double Kc;
    Vec  qLokal(8,fmatvec::NONINIT);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);

    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

    if (s < 0) // linker Teilbereich
      Kc = (l0h2*(-8*(aL + aR) + 3*(bL + bR)*l0) - 12*l0*(-8*aL + 8*aR + 3*(bL - bR)*l0)*s + 24*(16*aL - 8*aR - 5*bL*l0 + 3*bR*l0)*Power(s,2))/ l0h4;
    else
      Kc = (l0h2*(-8*(aL + aR) + 3*(bL + bR)*l0) - 12*l0*(-8*aL + 8*aR + 3*(bL - bR)*l0)*s - 24*(8*aL - 16*aR - 3*bL*l0 + 5*bR*l0)*Power(s,2))/ l0h4;
    return Kc;
  }
  double FiniteElement1s21RCM::Kpcurvature(Vec &qElement,Vec &qpElement,const double &s,const double &sp) {
    double Kcp;
    Vec  qLokal(8,fmatvec::NONINIT);
    SqrMat  Jeg(8,fmatvec::INIT,0.0);

    //lokale Koordinate
    BuildqLokal(qElement,qLokal);
    BuildJacobi(qElement,Jeg);
    Vec qpLokal = Jeg*qpElement;

    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);
    double &aLp    = qpLokal(4);     double &bLp   = qpLokal(5);
    double &aRp    = qpLokal(6);     double &bRp   = qpLokal(7);

    if (s < 0) // linker Teilbereich
      Kcp = (l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0) - 12*l0*(-8*aLp + 8*aRp + 3*(bLp - bRp)*l0)*s + 24*(16*aLp - 8*aRp - 5*bLp*l0 + 3*bRp*l0)*Power(s,2) - 12*l0*(-8*aL + 8*aR + 3*(bL - bR)*l0)*sp + 48*(16*aL - 8*aR - 5*bL*l0 + 3*bR*l0)*s*sp)/l0h4;
    else 
      Kcp = (l0h2*(-8*(aLp + aRp) + 3*(bLp + bRp)*l0) - 12*l0*(-8*aLp + 8*aRp + 3*(bLp - bRp)*l0)*s -  24*(8*aLp - 16*aRp - 3*bLp*l0 + 5*bRp*l0)*Power(s,2) - 12*l0*(-8*aL + 8*aR + 3*(bL - bR)*l0)*sp - 48*(8*aL - 16*aR - 3*bL*l0 + 5*bR*l0)*s*sp)/l0h4;
    return Kcp;
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  Mat FiniteElement1s21RCM::hFullJacobi(const Vec& qElement,const Vec& qpElement,const Vec& qLokal,const Vec& qpLokal,const SqrMat& Jeg,const SqrMat& Jegp,const SymMat& MLokal,const Vec& hZwischen)
  {
    Mat Dhz(16,8,fmatvec::INIT,0.0);

    //--- globale Koordinaten, Geschwingigkeiten
    const double &x1   = qElement(0);     const double &y1  = qElement(1);    
    const double &phi1 = qElement(2);    
    const double &a1   = qElement(3);     const double &a2  = qElement(4);
    const double &x2   = qElement(5);     const double &y2  = qElement(6);
    const double &phi2 = qElement(7);
    
    const double &x1p   = qpElement(0);     const double &y1p  = qpElement(1);    
    const double &phi1p = qpElement(2);    
    const double &a1p   = qpElement(3);     const double &a2p  = qpElement(4);
    const double &x2p   = qpElement(5);     const double &y2p  = qpElement(6);
    const double &phi2p = qpElement(7);

    //--- lokale  Koordinaten, Geschwingigkeiten
    const double &phiS   = qLokal(2);      const double &eps   = qLokal(3);
    const double &aL     = qLokal(4);      const double &bL    = qLokal(5);
    const double &aR     = qLokal(6);      const double &bR    = qLokal(7);
    
	const double &phiSp  = qpLokal(2);     const double &epsp  = qpLokal(3);
    const double &aLp    = qpLokal(4);     const double &bLp   = qpLokal(5);
    const double &aRp    = qpLokal(6);     const double &bRp   = qpLokal(7);

    //--- Zwischenergebnisse
    const double &hZ1    = hZwischen(0);   const double &hZ2   = hZwischen(1);
    const double &hZ3    = hZwischen(2);   const double &hZ4   = hZwischen(3);
    const double &hZ5    = hZwischen(4);   const double &hZ6   = hZwischen(5);
    const double &hZ7    = hZwischen(6);   const double &hZ8   = hZwischen(7);
    //
    Vec JpqpG(8,fmatvec::INIT,0.0);
    JpqpG = Jegp*qpElement;
    const double &JpqpG1     = JpqpG(0);        const double &JpqpG2     = JpqpG(1);    
    const double &JpqpG3     = JpqpG(2);        const double &JpqpG4     = JpqpG(3);    
    const double &JpqpG5     = JpqpG(4);        const double &JpqpG6     = JpqpG(5);    
    const double &JpqpG7     = JpqpG(6);        const double &JpqpG8     = JpqpG(7);    

    // Rechenaufwand senken durch "Zentralisierung"
    double one_p_cos_dphi;
    one_p_cos_dphi = (1 + cos(phi1 - phi2));

    // Gravitation
    double gx = g(0);
//    double gy = g(1);
cout << "\nIMPORTANT -- addapt to arbitrary gravitation Vec g(2)\n" << endl;

    // die Komponenten
    // Alb. nach q
    SqrMat dhqJ(8,fmatvec::INIT,0.0), dhLq(8,fmatvec::INIT,0.0), dhLqM(8,fmatvec::INIT,0.0), dhLqJp(8,fmatvec::INIT,0.0);
    // Alb. nach qp
    SqrMat dhLqp(8,fmatvec::INIT,0.0), dhLqpJp(8,fmatvec::INIT,0.0);

    // Aus der Herleitungsroutine

    //Abl. h nach qGlobal
    //dhqJ(0,0) = 0;
    //dhqJ(0,1) = 0;
    dhqJ(0,2) = (-4*hZ2*l0 + (22*(hZ3 + hZ6 - hZ8) + 4*(hZ5 - hZ7)*l0)*cos(phi2) + 8*hZ4*sin(phi2))/(8.*l0*one_p_cos_dphi);
    //dhqJ(0,3) = 0;
    //dhqJ(0,4) = 0;
    //dhqJ(0,5) = 0;
    //dhqJ(0,6) = 0;
    dhqJ(0,7) = (4*hZ2*l0 + (22*(hZ3 + hZ6 - hZ8) + 4*(hZ5 - hZ7)*l0)*cos(phi1) + 8*hZ4*sin(phi1))/(8.*l0*one_p_cos_dphi);
    //dhqJ(1,0) = 0;
    //dhqJ(1,1) = 0;
    dhqJ(1,2) = (4*hZ1*l0 - 8*hZ4*cos(phi2) + (22*(hZ3 + hZ6 - hZ8) + 4*(hZ5 - hZ7)*l0)*sin(phi2))/(8.*l0*one_p_cos_dphi);
    //dhqJ(1,3) = 0;
    //dhqJ(1,4) = 0;
    //dhqJ(1,5) = 0;
    //dhqJ(1,6) = 0;
    dhqJ(1,7) = (-4*hZ1*l0 - 8*hZ4*cos(phi1) + (22*(hZ3 + hZ6 - hZ8) + 4*(hZ5 - hZ7)*l0)*sin(phi1))/(8.*l0*one_p_cos_dphi);
    dhqJ(2,0) = (-2*hZ2*l0 + (11*(hZ3 + hZ6 - hZ8) + 2*(hZ5 - hZ7)*l0)*cos(phi2) + 4*hZ4*sin(phi2))/(4.*l0*one_p_cos_dphi);
    dhqJ(2,1) = (2*hZ1*l0 - 4*hZ4*cos(phi2) + (11*(hZ3 + hZ6 - hZ8) + 2*(hZ5 - hZ7)*l0)*sin(phi2))/(4.*l0*one_p_cos_dphi);
    dhqJ(2,2) = (-((sin(phi1 - phi2)*(l0*(-36*hZ3 + 36*(hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0) - 2*(64*a1*hZ4 + 64*a2*hZ4 + l0*(5*hZ4*(-phi1 + phi2) + 18*(-(hZ2*x1) + hZ2*x2 + hZ1*y1 - hZ1*y2))) + 5*l0h2*(hZ2*cos(phi1) - hZ1*sin(phi1)) + cos(phi2)*(l0*(64*a1*hZ1 + 64*a2*hZ1 + 5*l0*(hZ2 - hZ1*phi1 + hZ1*phi2) - 36*(hZ5 - hZ7)*(x1 - x2)) - 18*(11*(hZ3 + hZ6 - hZ8)*(x1 - x2) + 4*hZ4*(-y1 + y2)) + l0*(-36*hZ3 + 36*(hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0)*cos(phi1) + 10*hZ4*l0*sin(phi1)) + (l0*(64*a1*hZ2 + 64*a2*hZ2 - 5*l0*(hZ1 + hZ2*phi1 - hZ2*phi2) - 36*(hZ5 - hZ7)*(y1 - y2)) + 18*(4*hZ4*(-x1 + x2) + 11*(hZ3 + hZ6 - hZ8)*(-y1 + y2)) - 10*hZ4*l0*cos(phi1) + l0*(-36*hZ3 + 36*(hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0)*sin(phi1))*sin(phi2)))/l0) + one_p_cos_dphi*(-10*hZ4*one_p_cos_dphi + 36*(-hZ3 + hZ6 + hZ8)*sin(phi1 - phi2) + 5*l0*(hZ1*(cos(phi1) + cos(phi2)) + (hZ5 + hZ7)*sin(phi1 - phi2) + hZ2*(sin(phi1) + sin(phi2)))))/(72.*Power(1 + cos(phi1 - phi2),2));
    dhqJ(2,3) = (-8*(-2*hZ4 + hZ1*l0*cos(phi2) + hZ2*l0*sin(phi2)))/(9.*l0*one_p_cos_dphi);
    dhqJ(2,4) = (-8*(-2*hZ4 + hZ1*l0*cos(phi2) + hZ2*l0*sin(phi2)))/(9.*l0*one_p_cos_dphi);
    dhqJ(2,5) = (2*hZ2*l0 + (-11*(hZ3 + hZ6 - hZ8) - 2*(hZ5 - hZ7)*l0)*cos(phi2) - 4*hZ4*sin(phi2))/(4.*l0*one_p_cos_dphi);
    dhqJ(2,6) = (-2*hZ1*l0 + 4*hZ4*cos(phi2) + (-11*(hZ3 + hZ6 - hZ8) - 2*(hZ5 - hZ7)*l0)*sin(phi2))/(4.*l0*one_p_cos_dphi);
    dhqJ(2,7) = (40*hZ4*l0 + (l0*(5*hZ1*l0 - 2*hZ2*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2)) + 72*(hZ5 - hZ7)*(y1 - y2)) + 36*(4*hZ4*(x1 - x2) + 11*(hZ3 + hZ6 - hZ8)*(y1 - y2)))*cos(phi1) - 5*hZ1*l0h2*cos(phi1 - 2*phi2) + (l0*(5*hZ2*l0 + 2*hZ1*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2)) - 72*(hZ5 - hZ7)*(x1 - x2)) - 36*(11*(hZ3 + hZ6 - hZ8)*(x1 - x2) + 4*hZ4*(-y1 + y2)))*sin(phi1) + 5*hZ2*l0h2*cos(2*phi2)*sin(phi1) + cos(phi2)*(l0*(-5*hZ1*l0 - 2*hZ2*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2) + 72*(hZ5 - hZ7)*(y1 - y2)) + 36*(4*hZ4*(x1 - x2) + 11*(hZ3 + hZ6 - hZ8)*(y1 - y2)) + 40*hZ4*l0*cos(phi1) - 256*(a1 + a2)*hZ4*sin(phi1) + l0*(5*hZ1*l0*cos(2*phi1) + 4*(5*hZ4*(phi1 - phi2) + 18*(hZ2*x1 - hZ2*x2 - hZ1*y1 + hZ1*y2))*sin(phi1) + 5*hZ2*l0*sin(2*phi1))) + (l0*(-5*hZ2*l0 + 2*hZ1*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2)) - 72*(hZ5 - hZ7)*(x1 - x2)) - 36*(11*(hZ3 + hZ6 - hZ8)*(x1 - x2) + 4*hZ4*(-y1 + y2)) + 4*(64*a1*hZ4 + 64*a2*hZ4 + l0*(5*hZ4*(-phi1 + phi2) + 18*(-(hZ2*x1) + hZ2*x2 + hZ1*y1 - hZ1*y2)))*cos(phi1) + 5*l0*(-(hZ2*l0*cos(2*phi1)) + 8*hZ4*sin(phi1) + hZ1*l0*sin(2*phi1)))*sin(phi2) - 5*hZ2*l0h2*cos(phi1)*sin(2*phi2))/(144.*l0*Power(1 + cos(phi1 - phi2),2));
    //dhqJ(3,0) = 0;
    //dhqJ(3,1) = 0;
    dhqJ(3,2) = (-8*(-2*hZ4 + hZ1*l0*cos(phi2) + hZ2*l0*sin(phi2)))/(9.*l0*one_p_cos_dphi);
    //dhqJ(3,3) = 0;
    //dhqJ(3,4) = 0;
    //dhqJ(3,5) = 0;
    //dhqJ(3,6) = 0;
    dhqJ(3,7) = (-8*(2*hZ4 + hZ1*l0*cos(phi1) + hZ2*l0*sin(phi1)))/(9.*l0*one_p_cos_dphi);
    //dhqJ(4,0) = 0;
    //dhqJ(4,1) = 0;
    dhqJ(4,2) = (-8*(-2*hZ4 + hZ1*l0*cos(phi2) + hZ2*l0*sin(phi2)))/(9.*l0*one_p_cos_dphi);
    //dhqJ(4,3) = 0;
    //dhqJ(4,4) = 0;
    //dhqJ(4,5) = 0;
    //dhqJ(4,6) = 0;
    dhqJ(4,7) = (-8*(2*hZ4 + hZ1*l0*cos(phi1) + hZ2*l0*sin(phi1)))/(9.*l0*one_p_cos_dphi);
    //dhqJ(5,0) = 0;
    //dhqJ(5,1) = 0;
    dhqJ(5,2) = (4*hZ2*l0 + (-22*(hZ3 + hZ6 - hZ8) - 4*(hZ5 - hZ7)*l0)*cos(phi2) - 8*hZ4*sin(phi2))/(8.*l0*one_p_cos_dphi);
    //dhqJ(5,3) = 0;
    //dhqJ(5,4) = 0;
    //dhqJ(5,5) = 0;
    //dhqJ(5,6) = 0;
    dhqJ(5,7) = -(2*hZ2*l0 + (11*(hZ3 + hZ6 - hZ8) + 2*(hZ5 - hZ7)*l0)*cos(phi1) + 4*hZ4*sin(phi1))/(4.*l0*one_p_cos_dphi);
    //dhqJ(6,0) = 0;
    //dhqJ(6,1) = 0;
    dhqJ(6,2) = (-4*hZ1*l0 + 8*hZ4*cos(phi2) + (-22*(hZ3 + hZ6 - hZ8) - 4*(hZ5 - hZ7)*l0)*sin(phi2))/(8.*l0*one_p_cos_dphi);
    //dhqJ(6,3) = 0;
    //dhqJ(6,4) = 0;
    //dhqJ(6,5) = 0;
    //dhqJ(6,6) = 0;
    dhqJ(6,7) = (4*hZ1*l0 + 8*hZ4*cos(phi1) + (-22*(hZ3 + hZ6 - hZ8) - 4*(hZ5 - hZ7)*l0)*sin(phi1))/(8.*l0*one_p_cos_dphi);
    dhqJ(7,0) = (2*hZ2*l0 + (11*(hZ3 + hZ6 - hZ8) + 2*(hZ5 - hZ7)*l0)*cos(phi1) + 4*hZ4*sin(phi1))/(4.*l0*one_p_cos_dphi);
    dhqJ(7,1) = (-2*hZ1*l0 - 4*hZ4*cos(phi1) + (11*(hZ3 + hZ6 - hZ8) + 2*(hZ5 - hZ7)*l0)*sin(phi1))/(4.*l0*one_p_cos_dphi);
    dhqJ(7,2) = (one_p_cos_dphi*(10*hZ4*l0 - (l0*(hZ2*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2) - 36*(hZ5 - hZ7)*(y1 - y2)) + 18*(4*hZ4*(-x1 + x2) + 11*(hZ3 + hZ6 - hZ8)*(-y1 + y2)))*cos(phi1) + 10*hZ4*l0*cos(phi1 - phi2) + (l0*(hZ1*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2) - 36*(hZ5 - hZ7)*(x1 - x2)) - 18*(11*(hZ3 + hZ6 - hZ8)*(x1 - x2) + 4*hZ4*(-y1 + y2)))*sin(phi1) - l0*(36*(hZ3 + hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0)*sin(phi1 - phi2)) + sin(phi1 - phi2)*(l0*(36*(hZ3 + hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0) - 2*(64*a1*hZ4 + 64*a2*hZ4 + l0*(5*hZ4*(-phi1 + phi2) + 18*(-(hZ2*x1) + hZ2*x2 + hZ1*y1 - hZ1*y2))) + (l0*(-64*a1*hZ1 - 64*a2*hZ1 + 5*l0*(hZ2 + hZ1*phi1 - hZ1*phi2) + 36*(hZ5 - hZ7)*(x1 - x2)) + 18*(11*(hZ3 + hZ6 - hZ8)*(x1 - x2) + 4*hZ4*(-y1 + y2)))*cos(phi1) + l0*(36*(hZ3 + hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0)*cos(phi1 - phi2) - (l0*(64*a1*hZ2 + 64*a2*hZ2 + 5*l0*(hZ1 - hZ2*phi1 + hZ2*phi2) - 36*(hZ5 - hZ7)*(y1 - y2)) + 18*(4*hZ4*(-x1 + x2) + 11*(hZ3 + hZ6 - hZ8)*(-y1 + y2)))*sin(phi1) + 5*l0*cos(phi2)*(hZ2*l0 + 2*hZ4*sin(phi1)) - 5*l0*(hZ1*l0 + 2*hZ4*cos(phi1))*sin(phi2)))/(72.*l0*Power(1 + cos(phi1 - phi2),2));
    dhqJ(7,3) = (-8*(2*hZ4 + hZ1*l0*cos(phi1) + hZ2*l0*sin(phi1)))/(9.*l0*one_p_cos_dphi);
    dhqJ(7,4) = (-8*(2*hZ4 + hZ1*l0*cos(phi1) + hZ2*l0*sin(phi1)))/(9.*l0*one_p_cos_dphi);
    dhqJ(7,5) = (-2*hZ2*l0 + (-11*(hZ3 + hZ6 - hZ8) - 2*(hZ5 - hZ7)*l0)*cos(phi1) - 4*hZ4*sin(phi1))/(4.*l0*one_p_cos_dphi);
    dhqJ(7,6) = (2*hZ1*l0 + 4*hZ4*cos(phi1) + (-11*(hZ3 + hZ6 - hZ8) - 2*(hZ5 - hZ7)*l0)*sin(phi1))/(4.*l0*one_p_cos_dphi);
    dhqJ(7,7) = (-((sin(phi1 - phi2)*(l0*(36*(hZ3 + hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0) - 2*(64*a1*hZ4 + 64*a2*hZ4 + l0*(5*hZ4*(-phi1 + phi2) + 18*(-(hZ2*x1) + hZ2*x2 + hZ1*y1 - hZ1*y2))) + (l0*(-64*a1*hZ1 - 64*a2*hZ1 + 5*l0*(hZ2 + hZ1*phi1 - hZ1*phi2) + 36*(hZ5 - hZ7)*(x1 - x2)) + 18*(11*(hZ3 + hZ6 - hZ8)*(x1 - x2) + 4*hZ4*(-y1 + y2)))*cos(phi1) + l0*(36*(hZ3 + hZ6 + hZ8) + 5*(hZ5 + hZ7)*l0)*cos(phi1 - phi2) - (l0*(64*a1*hZ2 + 64*a2*hZ2 + 5*l0*(hZ1 - hZ2*phi1 + hZ2*phi2) - 36*(hZ5 - hZ7)*(y1 - y2)) + 18*(4*hZ4*(-x1 + x2) + 11*(hZ3 + hZ6 - hZ8)*(-y1 + y2)))*sin(phi1) + 5*l0*cos(phi2)*(hZ2*l0 + 2*hZ4*sin(phi1)) - 5*l0*(hZ1*l0 + 2*hZ4*cos(phi1))*sin(phi2)))/l0) + one_p_cos_dphi*(-10*hZ4*one_p_cos_dphi + 36*(hZ3 + hZ6 + hZ8)*sin(phi1 - phi2) - 5*l0*(hZ1*(cos(phi1) + cos(phi2)) - (hZ5 + hZ7)*sin(phi1 - phi2) + hZ2*(sin(phi1) + sin(phi2)))))/(72.*Power(1 + cos(phi1 - phi2),2));

    //dhLq(0,0) = 0;
    //dhLq(0,1) = 0;
    dhLq(0,2) = (Arho*l0*((bLp - phiSp)*(104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp) + 3*l0*(5*bL*bLp - bLp*bR + 40*epsp - 5*bL*phiSp + bR*phiSp))*cos(bL - phiS) + (bRp + phiSp)*(8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(bL*(bRp + phiSp) - 5*(8*epsp + bR*(bRp + phiSp))))*cos(bR + phiS) - 2*((bLp - phiSp)*(-104*aLp - 8*aRp + 3*l0*(bRp + 5*bLp*(1 + 2*eps) - 10*(1 + eps)*phiSp))*sin(bL - phiS) + (bRp + phiSp)*(-8*aLp - 104*aRp + 3*l0*(bLp + 5*(bRp + 2*bRp*eps + 2*(1 + eps)*phiSp)))*sin(bR + phiS))))/480.;
    dhLq(0,3) = -(Arho*l0h2*(Power(bLp - phiSp,2)*cos(bL - phiS) - Power(bRp + phiSp,2)*cos(bR + phiS)))/8.;
    dhLq(0,4) = (Arho*l0*(-13*Power(bLp - phiSp,2)*sin(bL - phiS) + Power(bRp + phiSp,2)*sin(bR + phiS)))/60.;
    dhLq(0,5) = -(Arho*l0*((bLp - phiSp)*(104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp) + 3*l0*(5*bL*bLp - bLp*bR + 40*epsp - 5*bL*phiSp + bR*phiSp))*cos(bL - phiS) + (bLp - phiSp)*(208*aLp + 16*aRp - 3*l0*(2*bRp + 5*bLp*(1 + 4*eps) - 5*(3 + 4*eps)*phiSp))*sin(bL - phiS) + 3*l0*Power(bRp + phiSp,2)*sin(bR + phiS)))/480.;
    dhLq(0,6) = (Arho*l0*(-(Power(bLp - phiSp,2)*sin(bL - phiS)) + 13*Power(bRp + phiSp,2)*sin(bR + phiS)))/60.;
    dhLq(0,7) = (Arho*l0*((bRp + phiSp)*(8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(bL*(bRp + phiSp) - 5*(8*epsp + bR*(bRp + phiSp))))*cos(bR + phiS) + 3*l0*Power(bLp - phiSp,2)*sin(bL - phiS) - (bRp + phiSp)*(-16*aLp - 208*aRp + 3*l0*(2*bLp + 5*(bRp + 4*bRp*eps + 3*phiSp + 4*eps*phiSp)))*sin(bR + phiS)))/480.;
    //dhLq(1,0) = 0;
    //dhLq(1,1) = 0;
    dhLq(1,2) = (Arho*l0*(2*(bLp - phiSp)*(104*aLp + 8*aRp - 3*l0*(bRp + 5*bLp*(1 + 2*eps) - 10*(1 + eps)*phiSp))*cos(bL - phiS) + 2*(bRp + phiSp)*(-8*aLp - 104*aRp + 3*l0*(bLp + 5*(bRp + 2*bRp*eps + 2*(1 + eps)*phiSp)))*cos(bR + phiS) - 104*aL*Power(bLp,2)*sin(bL - phiS) - 8*aR*Power(bLp,2)*sin(bL - phiS) - 15*bL*Power(bLp,2)*l0*sin(bL - phiS) + 3*Power(bLp,2)*bR*l0*sin(bL - phiS) - 120*bLp*epsp*l0*sin(bL - phiS) + 208*aL*bLp*phiSp*sin(bL - phiS) + 16*aR*bLp*phiSp*sin(bL - phiS) + 30*bL*bLp*l0*phiSp*sin(bL - phiS) - 6*bLp*bR*l0*phiSp*sin(bL - phiS) + 120*epsp*l0*phiSp*sin(bL - phiS) - 104*aL*Power(phiSp,2)*sin(bL - phiS) - 8*aR*Power(phiSp,2)*sin(bL - phiS) - 15*bL*l0*Power(phiSp,2)*sin(bL - phiS) + 3*bR*l0*Power(phiSp,2)*sin(bL - phiS) + 8*aL*Power(bRp,2)*sin(bR + phiS) + 104*aR*Power(bRp,2)*sin(bR + phiS) - 3*bL*Power(bRp,2)*l0*sin(bR + phiS) + 15*bR*Power(bRp,2)*l0*sin(bR + phiS) + 120*bRp*epsp*l0*sin(bR + phiS) + 16*aL*bRp*phiSp*sin(bR + phiS) + 208*aR*bRp*phiSp*sin(bR + phiS) - 6*bL*bRp*l0*phiSp*sin(bR + phiS) + 30*bR*bRp*l0*phiSp*sin(bR + phiS) + 120*epsp*l0*phiSp*sin(bR + phiS) + 8*aL*Power(phiSp,2)*sin(bR + phiS) + 104*aR*Power(phiSp,2)*sin(bR + phiS) - 3*bL*l0*Power(phiSp,2)*sin(bR + phiS) + 15*bR*l0*Power(phiSp,2)*sin(bR + phiS)))/480.;
    dhLq(1,3) = (Arho*l0h2*(Power(bLp - phiSp,2)*sin(bL - phiS) + Power(bRp + phiSp,2)*sin(bR + phiS)))/8.;
    dhLq(1,4) = -(Arho*l0*(13*Power(bLp - phiSp,2)*cos(bL - phiS) + Power(bRp + phiSp,2)*cos(bR + phiS)))/60.;
    dhLq(1,5) = (Arho*l0*((bLp - phiSp)*(-208*aLp - 16*aRp + 3*l0*(2*bRp + 5*bLp*(1 + 4*eps) - 5*(3 + 4*eps)*phiSp))*cos(bL - phiS) + 3*l0*Power(bRp + phiSp,2)*cos(bR + phiS) + (bLp - phiSp)*(104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp) + 3*l0*(5*bL*bLp - bLp*bR + 40*epsp - 5*bL*phiSp + bR*phiSp))*sin(bL - phiS)))/480.;
    dhLq(1,6) = -(Arho*l0*(Power(bLp - phiSp,2)*cos(bL - phiS) + 13*Power(bRp + phiSp,2)*cos(bR + phiS)))/60.;
    dhLq(1,7) = (Arho*l0*(3*l0*Power(bLp - phiSp,2)*cos(bL - phiS) + (bRp + phiSp)*((-16*aLp - 208*aRp + 3*l0*(2*bLp + 5*(bRp + 4*bRp*eps + 3*phiSp + 4*eps*phiSp)))*cos(bR + phiS) + (8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(bL*(bRp + phiSp) - 5*(8*epsp + bR*(bRp + phiSp))))*sin(bR + phiS))))/480.;
    //dhLq(2,0) = 0;
    //dhLq(2,1) = 0;
    dhLq(2,2) = -(Arho*gx*l0*((104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*cos(bL - phiS) + (8*aL + 104*aR - 3*bL*l0 + 15*bR*l0)*cos(bR + phiS) - 60*(1 + eps)*l0*(sin(bL - phiS) + sin(bR + phiS))))/480.;
    dhLq(2,3) = (Arho*l0h2*(2*epsp*l0*(bLp - bRp - 2*phiSp) + 3*gx*cos(bL - phiS) - 3*gx*cos(bR + phiS)))/24.;
    dhLq(2,4) = (Arho*l0*(3136*aLp*bLp + 136*aRp*bLp - 16*aLp*bRp - 136*aRp*bRp + 205*Power(bLp,2)*l0 - 45*bLp*bRp*l0 - 28*Power(bRp,2)*l0 - 3152*aLp*phiSp - 272*aRp*phiSp - 199*bLp*l0*phiSp + 23*bRp*l0*phiSp + 2184*gx*sin(bL - phiS) - 168*gx*sin(bR + phiS)))/10080.;
    dhLq(2,5) = (Arho*l0*(84*gx*(104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*cos(bL - phiS) + l0*(820*aLp*bLp + 112*aRp*bLp + 24*aLp*bRp + 204*aRp*bRp + 211*Power(bLp,2)*l0 - 51*bLp*bRp*l0 + 42*Power(bRp,2)*l0 - 796*aLp*phiSp + 92*aRp*phiSp - 220*bLp*l0*phiSp + 84*bRp*l0*phiSp - 1260*(3 + 4*eps)*gx*sin(bL - phiS) + 252*gx*sin(bR + phiS))))/40320.;
    dhLq(2,6) = (Arho*l0*(136*aLp*bLp + 16*aRp*bLp - 136*aLp*bRp - 3136*aRp*bRp + 28*Power(bLp,2)*l0 + 45*bLp*bRp*l0 - 205*Power(bRp,2)*l0 - 272*aLp*phiSp - 3152*aRp*phiSp + 23*bLp*l0*phiSp - 199*bRp*l0*phiSp + 168*gx*sin(bL - phiS) - 2184*gx*sin(bR + phiS)))/10080.;
    dhLq(2,7) = -(Arho*l0*(84*gx*(8*aL + 104*aR - 3*(bL - 5*bR)*l0)*cos(bR + phiS) + l0*(204*aLp*bLp + 24*aRp*bLp + 112*aLp*bRp + 820*aRp*bRp + 42*Power(bLp,2)*l0 - 51*bLp*bRp*l0 + 211*Power(bRp,2)*l0 - 92*aLp*phiSp + 796*aRp*phiSp - 84*bLp*l0*phiSp + 220*bRp*l0*phiSp + 252*gx*sin(bL - phiS) - 1260*(3 + 4*eps)*gx*sin(bR + phiS))))/40320.;
    //dhLq(3,0) = 0;
    //dhLq(3,1) = 0;
    dhLq(3,2) = (Arho*gx*l0h2*(cos(bL - phiS) - cos(bR + phiS)))/8.;
    dhLq(3,3) = -(295936*Power(aL,4)*EA + 295936*Power(aR,4)*EA + 13056*Power(aR,3)*(3*bL - 8*bR)*EA*l0 + 48*Power(aR,2)*(9520 + 639*Power(bL,2) - 552*bL*bR + 804*Power(bR,2))*EA*l0h2 + 72*aR*(27*Power(bL,3) - 90*Power(bL,2)*bR + 15*bL*(28 + 5*Power(bR,2)) - 8*bR*(140 + 9*Power(bR,2)))*EA*l0h3 - 4352*Power(aL,3)*EA*(16*aR + 24*bL*l0 - 9*bR*l0) + 48*Power(aL,2)*EA*(12416*Power(aR,2) + 16*aR*(67*bL - 142*bR)*l0 + (9520 + 804*Power(bL,2) - 552*bL*bR + 639*Power(bR,2))*l0h2) - 8*aL*EA*(8704*Power(aR,3) + 96*Power(aR,2)*(142*bL - 67*bR)*l0 + 12*aR*(560 + 108*Power(bL,2) - 243*bL*bR + 108*Power(bR,2))*l0h2 + 9*(1120*bL + 72*Power(bL,3) - 420*bR - 75*Power(bL,2)*bR + 90*bL*Power(bR,2) - 27*Power(bR,3))*l0h3) - 3*l0h4*(-3*Power(140 + 9*Power(bL,2) - 6*bL*bR + 9*Power(bR,2),2)*EA + 2450*Arho*l0h2*(Power(bLp,2) + Power(bRp,2) - 2*bLp*phiSp + 2*bRp*phiSp + 2*Power(phiSp,2))))/(176400.*l0h3);
    dhLq(3,4) = (EA*(-272*aL + 16*aR + 24*bL*l0 - 9*bR*l0)*(544*Power(aL,2)*(1 + eps) + 544*Power(aR,2)*(1 + eps) + 12*aR*(3*bL - 8*bR)*(1 + eps)*l0 + 3*(9*Power(bL,2)*(1 + eps) - 6*bL*bR*(1 + eps) + 9*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2 - 4*aL*(1 + eps)*(16*aR + 24*bL*l0 - 9*bR*l0)))/(22050.*l0h3);
    dhLq(3,5) = (2*EA*(16*aL - 6*aR + 3*(-3*bL + bR)*l0)*(544*Power(aL,2)*(1 + eps) + 544*Power(aR,2)*(1 + eps) + 12*aR*(3*bL - 8*bR)*(1 + eps)*l0 + 3*(9*Power(bL,2)*(1 + eps) - 6*bL*bR*(1 + eps) + 9*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2 - 4*aL*(1 + eps)*(16*aR + 24*bL*l0 - 9*bR*l0)) - 3675*Arho*gx*l0h4*cos(bL - phiS))/(29400.*l0h2);
    dhLq(3,6) = (EA*(16*aL - 272*aR - 9*bL*l0 + 24*bR*l0)*(544*Power(aL,2)*(1 + eps) + 544*Power(aR,2)*(1 + eps) + 12*aR*(3*bL - 8*bR)*(1 + eps)*l0 + 3*(9*Power(bL,2)*(1 + eps) - 6*bL*bR*(1 + eps) + 9*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2 - 4*aL*(1 + eps)*(16*aR + 24*bL*l0 - 9*bR*l0)))/(22050.*l0h3);
    dhLq(3,7) = -(2*EA*(6*aL - 16*aR - 3*bL*l0 + 9*bR*l0)*(544*Power(aL,2)*(1 + eps) + 544*Power(aR,2)*(1 + eps) + 12*aR*(3*bL - 8*bR)*(1 + eps)*l0 + 3*(9*Power(bL,2)*(1 + eps) - 6*bL*bR*(1 + eps) + 9*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2 - 4*aL*(1 + eps)*(16*aR + 24*bL*l0 - 9*bR*l0)) + 3675*Arho*gx*l0h4*cos(bR + phiS))/(29400.*l0h2);
    //dhLq(4,0) = 0;
    //dhLq(4,1) = 0;
    dhLq(4,2) = -(Arho*gx*l0*(-13*sin(bL - phiS) + sin(bR + phiS)))/60.;
    dhLq(4,3) = (EA*(-272*aL + 16*aR + 24*bL*l0 - 9*bR*l0)*(544*Power(aL,2)*(1 + eps) + 544*Power(aR,2)*(1 + eps) + 12*aR*(3*bL - 8*bR)*(1 + eps)*l0 + 3*(9*Power(bL,2)*(1 + eps) - 6*bL*bR*(1 + eps) + 9*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2 - 4*aL*(1 + eps)*(16*aR + 24*bL*l0 - 9*bR*l0)))/(22050.*l0h3);
    dhLq(4,4) = -(5644800*EI + 443904*Power(aL,2)*EA*Power(1 + eps,2) + 148992*Power(aR,2)*EA*Power(1 + eps,2) + 192*aR*(67*bL - 142*bR)*EA*Power(1 + eps,2)*l0 + 9648*Power(bL,2)*EA*l0h2 - 6624*bL*bR*EA*l0h2 + 7668*Power(bR,2)*EA*l0h2 + 114240*EA*eps*l0h2 + 19296*Power(bL,2)*EA*eps*l0h2 - 13248*bL*bR*EA*eps*l0h2 + 15336*Power(bR,2)*EA*eps*l0h2 + 114240*EA*Power(eps,2)*l0h2 + 9648*Power(bL,2)*EA*Power(eps,2)*l0h2 - 6624*bL*bR*EA*Power(eps,2)*l0h2 + 7668*Power(bR,2)*EA*Power(eps,2)*l0h2 - 6860*Arho*Power(bLp,2)*l0h4 - 35*Arho*Power(bRp,2)*l0h4 - 3264*aL*EA*Power(1 + eps,2)*(16*aR + 24*bL*l0 - 9*bR*l0) + 13720*Arho*bLp*l0h4*phiSp - 70*Arho*bRp*l0h4*phiSp - 6895*Arho*l0h4*Power(phiSp,2))/(44100.*l0h3);
    dhLq(4,5) = (218112*Power(aR,2)*EA + 24272640*EI + 436224*Power(aR,2)*EA*eps + 218112*Power(aR,2)*EA*Power(eps,2) + 626688*Power(aL,2)*EA*Power(1 + eps,2) + 41472*aR*bL*EA*l0 - 46656*aR*bR*EA*l0 + 82944*aR*bL*EA*eps*l0 - 93312*aR*bR*EA*eps*l0 + 41472*aR*bL*EA*Power(eps,2)*l0 - 46656*aR*bR*EA*Power(eps,2)*l0 + 31104*Power(bL,2)*EA*l0h2 - 21600*bL*bR*EA*l0h2 + 12960*Power(bR,2)*EA*l0h2 + 161280*EA*eps*l0h2 + 62208*Power(bL,2)*EA*eps*l0h2 - 43200*bL*bR*EA*eps*l0h2 + 25920*Power(bR,2)*EA*eps*l0h2 + 161280*EA*Power(eps,2)*l0h2 + 31104*Power(bL,2)*EA*Power(eps,2)*l0h2 - 21600*bL*bR*EA*Power(eps,2)*l0h2 + 12960*Power(bR,2)*EA*Power(eps,2)*l0h2 + 7175*Arho*Power(bLp,2)*l0h4 - 210*Arho*Power(bRp,2)*l0h4 - 1536*aL*EA*Power(1 + eps,2)*(134*aR + 201*bL*l0 - 69*bR*l0) - 14350*Arho*bLp*l0h4*phiSp - 420*Arho*bRp*l0h4*phiSp + 6965*Arho*l0h4*Power(phiSp,2) - 152880*Arho*gx*l0h3*sin(bL - phiS))/(705600.*l0h2);
    dhLq(4,6) = (2257920*EI + 52224*Power(aL,2)*EA*Power(1 + eps,2) + 52224*Power(aR,2)*EA*Power(1 + eps,2) + 384*aR*(142*bL - 67*bR)*EA*Power(1 + eps,2)*l0 + 2592*Power(bL,2)*EA*l0h2 - 5832*bL*bR*EA*l0h2 + 2592*Power(bR,2)*EA*l0h2 + 13440*EA*eps*l0h2 + 5184*Power(bL,2)*EA*eps*l0h2 - 11664*bL*bR*EA*eps*l0h2 + 5184*Power(bR,2)*EA*eps*l0h2 + 13440*EA*Power(eps,2)*l0h2 + 2592*Power(bL,2)*EA*Power(eps,2)*l0h2 - 5832*bL*bR*EA*Power(eps,2)*l0h2 + 2592*Power(bR,2)*EA*Power(eps,2)*l0h2 + 595*Arho*Power(bLp,2)*l0h4 + 595*Arho*Power(bRp,2)*l0h4 - 384*aL*EA*Power(1 + eps,2)*(1552*aR + 67*bL*l0 - 142*bR*l0) - 1190*Arho*bLp*l0h4*phiSp + 1190*Arho*bRp*l0h4*phiSp + 1190*Arho*l0h4*Power(phiSp,2))/(88200.*l0h3);
    dhLq(4,7) = -(102912*Power(aR,2)*EA + 6209280*EI + 205824*Power(aR,2)*EA*eps + 102912*Power(aR,2)*EA*Power(eps,2) + 235008*Power(aL,2)*EA*Power(1 + eps,2) + 46656*aR*bL*EA*l0 - 41472*aR*bR*EA*l0 + 93312*aR*bL*EA*eps*l0 - 82944*aR*bR*EA*eps*l0 + 46656*aR*bL*EA*Power(eps,2)*l0 - 41472*aR*bR*EA*Power(eps,2)*l0 + 10800*Power(bL,2)*EA*l0h2 - 25920*bL*bR*EA*l0h2 + 11664*Power(bR,2)*EA*l0h2 + 60480*EA*eps*l0h2 + 21600*Power(bL,2)*EA*eps*l0h2 - 51840*bL*bR*EA*eps*l0h2 + 23328*Power(bR,2)*EA*eps*l0h2 + 60480*EA*Power(eps,2)*l0h2 + 10800*Power(bL,2)*EA*Power(eps,2)*l0h2 - 25920*bL*bR*EA*Power(eps,2)*l0h2 + 11664*Power(bR,2)*EA*Power(eps,2)*l0h2 + 1785*Arho*Power(bLp,2)*l0h4 - 980*Arho*Power(bRp,2)*l0h4 - 384*aL*EA*Power(1 + eps,2)*(1136*aR + 276*bL*l0 - 639*bR*l0) - 3570*Arho*bLp*l0h4*phiSp - 1960*Arho*bRp*l0h4*phiSp + 805*Arho*l0h4*Power(phiSp,2) + 11760*Arho*gx*l0h3*sin(bR + phiS))/(705600.*l0h2);
    //dhLq(5,0) = 0;
    //dhLq(5,1) = 0;
    dhLq(5,2) = (Arho*gx*l0*((104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*cos(bL - phiS) + 3*l0*(-5*(3 + 4*eps)*sin(bL - phiS) + sin(bR + phiS))))/480.;
    dhLq(5,3) = (2*(8704*Power(aL,3)*EA*(1 + eps) - 3264*Power(aR,3)*EA*(1 + eps) - 24*Power(aR,2)*(213*bL - 92*bR)*EA*(1 + eps)*l0 - 18*aR*EA*(27*Power(bL,2)*(1 + eps) - 60*bL*bR*(1 + eps) + 5*(14 + 28*eps + 5*Power(bR,2)*(1 + eps)))*l0h2 - 32*Power(aL,2)*EA*(1 + eps)*(134*aR + 201*bL*l0 - 69*bR*l0) + 4*aL*EA*(2272*Power(aR,2)*(1 + eps) + 54*aR*(8*bL - 9*bR)*(1 + eps)*l0 + 3*(108*Power(bL,2)*(1 + eps) - 75*bL*bR*(1 + eps) + 5*(9*Power(bR,2)*(1 + eps) + 56*(1 + 2*eps)))*l0h2) - l0h3*(243*Power(bL,3)*EA*(1 + eps) - 243*Power(bL,2)*bR*EA*(1 + eps) - 81*Power(bR,3)*EA*(1 + eps) - 630*bR*(EA + 2*EA*eps) + 27*bL*EA*(11*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps)) + 1225*Arho*epsp*l0h2*(bLp - phiSp))) - 3675*Arho*gx*l0h4*cos(bL - phiS))/(29400.*l0h2);
    dhLq(5,4) = (218112*Power(aR,2)*EA + 24272640*EI + 436224*Power(aR,2)*EA*eps + 218112*Power(aR,2)*EA*Power(eps,2) + 626688*Power(aL,2)*EA*Power(1 + eps,2) + 41472*aR*bL*EA*l0 - 46656*aR*bR*EA*l0 + 82944*aR*bL*EA*eps*l0 - 93312*aR*bR*EA*eps*l0 + 41472*aR*bL*EA*Power(eps,2)*l0 - 46656*aR*bR*EA*Power(eps,2)*l0 + 31104*Power(bL,2)*EA*l0h2 - 21600*bL*bR*EA*l0h2 + 12960*Power(bR,2)*EA*l0h2 + 161280*EA*eps*l0h2 + 62208*Power(bL,2)*EA*eps*l0h2 - 43200*bL*bR*EA*eps*l0h2 + 25920*Power(bR,2)*EA*eps*l0h2 + 161280*EA*Power(eps,2)*l0h2 + 31104*Power(bL,2)*EA*Power(eps,2)*l0h2 - 21600*bL*bR*EA*Power(eps,2)*l0h2 + 12960*Power(bR,2)*EA*Power(eps,2)*l0h2 - 219520*aLp*Arho*bLp*l0h3 - 9520*Arho*aRp*bLp*l0h3 - 7175*Arho*Power(bLp,2)*l0h4 + 3570*Arho*bLp*bRp*l0h4 - 210*Arho*Power(bRp,2)*l0h4 - 1536*aL*EA*Power(1 + eps,2)*(134*aR + 201*bL*l0 - 69*bR*l0) + 219520*aLp*Arho*l0h3*phiSp + 9520*Arho*aRp*l0h3*phiSp - 3990*Arho*bRp*l0h4*phiSp + 6965*Arho*l0h4*Power(phiSp,2) - 152880*Arho*gx*l0h3*sin(bL - phiS))/(705600.*l0h2);
    dhLq(5,5) = -(617472*Power(aL,2)*EA - 165888*aL*aR*EA + 490752*Power(aR,2)*EA + 32175360*EI + 1234944*Power(aL,2)*EA*eps - 331776*aL*aR*EA*eps + 981504*Power(aR,2)*EA*eps + 617472*Power(aL,2)*EA*Power(eps,2) - 165888*aL*aR*EA*Power(eps,2) + 490752*Power(aR,2)*EA*Power(eps,2) - 248832*aL*bL*EA*l0 + 93312*aR*bL*EA*l0 + 86400*aL*bR*EA*l0 - 103680*aR*bR*EA*l0 - 497664*aL*bL*EA*eps*l0 + 186624*aR*bL*EA*eps*l0 + 172800*aL*bR*EA*eps*l0 - 207360*aR*bR*EA*eps*l0 - 248832*aL*bL*EA*Power(eps,2)*l0 + 93312*aR*bL*EA*Power(eps,2)*l0 + 86400*aL*bR*EA*Power(eps,2)*l0 - 103680*aR*bR*EA*Power(eps,2)*l0 + 69984*Power(bL,2)*EA*l0h2 - 46656*bL*bR*EA*l0h2 + 28512*Power(bR,2)*EA*l0h2 + 362880*EA*eps*l0h2 + 139968*Power(bL,2)*EA*eps*l0h2 - 93312*bL*bR*EA*eps*l0h2 + 57024*Power(bR,2)*EA*eps*l0h2 + 362880*EA*Power(eps,2)*l0h2 + 69984*Power(bL,2)*EA*Power(eps,2)*l0h2 - 46656*bL*bR*EA*Power(eps,2)*l0h2 + 28512*Power(bR,2)*EA*Power(eps,2)*l0h2 + 57400*aLp*Arho*bLp*l0h3 + 7840*Arho*aRp*bLp*l0h3 + 7385*Arho*Power(bLp,2)*l0h4 - 2940*Arho*bLp*bRp*l0h4 - 315*Arho*Power(bRp,2)*l0h4 - 57400*aLp*Arho*l0h3*phiSp - 7840*Arho*aRp*l0h3*phiSp + 2310*Arho*bRp*l0h4*phiSp - 7700*Arho*l0h4*Power(phiSp,2) + 5880*Arho*gx*l0h2*(104*aL + 8*aR + 15*bL*l0 - 3*bR*l0)*cos(bL - phiS) - 176400*Arho*(1 + 2*eps)*gx*l0h3*sin(bL - phiS))/(2.8224e6*l0);
    dhLq(5,6) = -(235008*Power(aR,2)*EA + 6209280*EI + 470016*Power(aR,2)*EA*eps + 235008*Power(aR,2)*EA*Power(eps,2) + 102912*Power(aL,2)*EA*Power(1 + eps,2) + 245376*aR*bL*EA*l0 - 105984*aR*bR*EA*l0 + 490752*aR*bL*EA*eps*l0 - 211968*aR*bR*EA*eps*l0 + 245376*aR*bL*EA*Power(eps,2)*l0 - 105984*aR*bR*EA*Power(eps,2)*l0 + 11664*Power(bL,2)*EA*l0h2 - 25920*bL*bR*EA*l0h2 + 10800*Power(bR,2)*EA*l0h2 + 60480*EA*eps*l0h2 + 23328*Power(bL,2)*EA*eps*l0h2 - 51840*bL*bR*EA*eps*l0h2 + 21600*Power(bR,2)*EA*eps*l0h2 + 60480*EA*Power(eps,2)*l0h2 + 11664*Power(bL,2)*EA*Power(eps,2)*l0h2 - 25920*bL*bR*EA*Power(eps,2)*l0h2 + 10800*Power(bR,2)*EA*Power(eps,2)*l0h2 + 9520*aLp*Arho*bLp*l0h3 + 1120*Arho*aRp*bLp*l0h3 + 980*Arho*Power(bLp,2)*l0h4 - 420*Arho*bLp*bRp*l0h4 + 1785*Arho*Power(bRp,2)*l0h4 - 192*aL*EA*Power(1 + eps,2)*(2272*aR + 27*(8*bL - 9*bR)*l0) - 9520*aLp*Arho*l0h3*phiSp - 1120*Arho*aRp*l0h3*phiSp + 3990*Arho*bRp*l0h4*phiSp + 805*Arho*l0h4*Power(phiSp,2) + 11760*Arho*gx*l0h3*sin(bL - phiS))/(705600.*l0h2);
    dhLq(5,7) = (35328*Power(aL,2)*EA - 31104*aL*aR*EA + 35328*Power(aR,2)*EA + 1411200*EI + 70656*Power(aL,2)*EA*eps - 62208*aL*aR*EA*eps + 70656*Power(aR,2)*EA*eps + 35328*Power(aL,2)*EA*Power(eps,2) - 31104*aL*aR*EA*Power(eps,2) + 35328*Power(aR,2)*EA*Power(eps,2) - 14400*aL*bL*EA*l0 + 17280*aR*bL*EA*l0 + 17280*aL*bR*EA*l0 - 14400*aR*bR*EA*l0 - 28800*aL*bL*EA*eps*l0 + 34560*aR*bL*EA*eps*l0 + 34560*aL*bR*EA*eps*l0 - 28800*aR*bR*EA*eps*l0 - 14400*aL*bL*EA*Power(eps,2)*l0 + 17280*aR*bL*EA*Power(eps,2)*l0 + 17280*aL*bR*EA*Power(eps,2)*l0 - 14400*aR*bR*EA*Power(eps,2)*l0 + 3888*Power(bL,2)*EA*l0h2 - 9504*bL*bR*EA*l0h2 + 3888*Power(bR,2)*EA*l0h2 + 20160*EA*eps*l0h2 + 7776*Power(bL,2)*EA*eps*l0h2 - 19008*bL*bR*EA*eps*l0h2 + 7776*Power(bR,2)*EA*eps*l0h2 + 20160*EA*Power(eps,2)*l0h2 + 3888*Power(bL,2)*EA*Power(eps,2)*l0h2 - 9504*bL*bR*EA*Power(eps,2)*l0h2 + 3888*Power(bR,2)*EA*Power(eps,2)*l0h2 + 2380*aLp*Arho*bLp*l0h3 + 280*Arho*aRp*bLp*l0h3 + 245*Arho*Power(bLp,2)*l0h4 - 105*Arho*bLp*bRp*l0h4 - 245*Arho*Power(bRp,2)*l0h4 - 2380*aLp*Arho*l0h3*phiSp - 280*Arho*aRp*l0h3*phiSp - 385*Arho*bRp*l0h4*phiSp - 490*Arho*l0h4*Power(phiSp,2) + 2940*Arho*gx*l0h3*sin(bL - phiS) + 2940*Arho*gx*l0h3*sin(bR + phiS))/(470400.*l0);
    //dhLq(6,0) = 0;
    //dhLq(6,1) = 0;
    dhLq(6,2) = (Arho*gx*l0*(sin(bL - phiS) - 13*sin(bR + phiS)))/60.;
    dhLq(6,3) = (EA*(16*aL - 272*aR - 9*bL*l0 + 24*bR*l0)*(544*Power(aL,2)*(1 + eps) + 544*Power(aR,2)*(1 + eps) + 12*aR*(3*bL - 8*bR)*(1 + eps)*l0 + 3*(9*Power(bL,2)*(1 + eps) - 6*bL*bR*(1 + eps) + 9*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2 - 4*aL*(1 + eps)*(16*aR + 24*bL*l0 - 9*bR*l0)))/(22050.*l0h3);
    dhLq(6,4) = (2257920*EI + 52224*Power(aL,2)*EA*Power(1 + eps,2) + 52224*Power(aR,2)*EA*Power(1 + eps,2) + 384*aR*(142*bL - 67*bR)*EA*Power(1 + eps,2)*l0 + 2592*Power(bL,2)*EA*l0h2 - 5832*bL*bR*EA*l0h2 + 2592*Power(bR,2)*EA*l0h2 + 13440*EA*eps*l0h2 + 5184*Power(bL,2)*EA*eps*l0h2 - 11664*bL*bR*EA*eps*l0h2 + 5184*Power(bR,2)*EA*eps*l0h2 + 13440*EA*Power(eps,2)*l0h2 + 2592*Power(bL,2)*EA*Power(eps,2)*l0h2 - 5832*bL*bR*EA*Power(eps,2)*l0h2 + 2592*Power(bR,2)*EA*Power(eps,2)*l0h2 + 595*Arho*Power(bLp,2)*l0h4 + 595*Arho*Power(bRp,2)*l0h4 - 384*aL*EA*Power(1 + eps,2)*(1552*aR + 67*bL*l0 - 142*bR*l0) - 1190*Arho*bLp*l0h4*phiSp + 1190*Arho*bRp*l0h4*phiSp + 1190*Arho*l0h4*Power(phiSp,2))/(88200.*l0h3);
    dhLq(6,5) = -(235008*Power(aR,2)*EA + 6209280*EI + 470016*Power(aR,2)*EA*eps + 235008*Power(aR,2)*EA*Power(eps,2) + 102912*Power(aL,2)*EA*Power(1 + eps,2) + 245376*aR*bL*EA*l0 - 105984*aR*bR*EA*l0 + 490752*aR*bL*EA*eps*l0 - 211968*aR*bR*EA*eps*l0 + 245376*aR*bL*EA*Power(eps,2)*l0 - 105984*aR*bR*EA*Power(eps,2)*l0 + 11664*Power(bL,2)*EA*l0h2 - 25920*bL*bR*EA*l0h2 + 10800*Power(bR,2)*EA*l0h2 + 60480*EA*eps*l0h2 + 23328*Power(bL,2)*EA*eps*l0h2 - 51840*bL*bR*EA*eps*l0h2 + 21600*Power(bR,2)*EA*eps*l0h2 + 60480*EA*Power(eps,2)*l0h2 + 11664*Power(bL,2)*EA*Power(eps,2)*l0h2 - 25920*bL*bR*EA*Power(eps,2)*l0h2 + 10800*Power(bR,2)*EA*Power(eps,2)*l0h2 - 980*Arho*Power(bLp,2)*l0h4 + 1785*Arho*Power(bRp,2)*l0h4 - 192*aL*EA*Power(1 + eps,2)*(2272*aR + 27*(8*bL - 9*bR)*l0) + 1960*Arho*bLp*l0h4*phiSp + 3570*Arho*bRp*l0h4*phiSp + 805*Arho*l0h4*Power(phiSp,2) + 11760*Arho*gx*l0h3*sin(bL - phiS))/(705600.*l0h2);
    dhLq(6,6) = -(5644800*EI + 148992*Power(aL,2)*EA*Power(1 + eps,2) + 443904*Power(aR,2)*EA*Power(1 + eps,2) + 9792*aR*(3*bL - 8*bR)*EA*Power(1 + eps,2)*l0 + 7668*Power(bL,2)*EA*l0h2 - 6624*bL*bR*EA*l0h2 + 9648*Power(bR,2)*EA*l0h2 + 114240*EA*eps*l0h2 + 15336*Power(bL,2)*EA*eps*l0h2 - 13248*bL*bR*EA*eps*l0h2 + 19296*Power(bR,2)*EA*eps*l0h2 + 114240*EA*Power(eps,2)*l0h2 + 7668*Power(bL,2)*EA*Power(eps,2)*l0h2 - 6624*bL*bR*EA*Power(eps,2)*l0h2 + 9648*Power(bR,2)*EA*Power(eps,2)*l0h2 - 35*Arho*Power(bLp,2)*l0h4 - 6860*Arho*Power(bRp,2)*l0h4 - 192*aL*EA*Power(1 + eps,2)*(272*aR + 142*bL*l0 - 67*bR*l0) + 70*Arho*bLp*l0h4*phiSp - 13720*Arho*bRp*l0h4*phiSp - 6895*Arho*l0h4*Power(phiSp,2))/(44100.*l0h3);
    dhLq(6,7) = (626688*Power(aR,2)*EA + 24272640*EI + 1253376*Power(aR,2)*EA*eps + 626688*Power(aR,2)*EA*Power(eps,2) + 218112*Power(aL,2)*EA*Power(1 + eps,2) + 105984*aR*bL*EA*l0 - 308736*aR*bR*EA*l0 + 211968*aR*bL*EA*eps*l0 - 617472*aR*bR*EA*eps*l0 + 105984*aR*bL*EA*Power(eps,2)*l0 - 308736*aR*bR*EA*Power(eps,2)*l0 + 12960*Power(bL,2)*EA*l0h2 - 21600*bL*bR*EA*l0h2 + 31104*Power(bR,2)*EA*l0h2 + 161280*EA*eps*l0h2 + 25920*Power(bL,2)*EA*eps*l0h2 - 43200*bL*bR*EA*eps*l0h2 + 62208*Power(bR,2)*EA*eps*l0h2 + 161280*EA*Power(eps,2)*l0h2 + 12960*Power(bL,2)*EA*Power(eps,2)*l0h2 - 21600*bL*bR*EA*Power(eps,2)*l0h2 + 31104*Power(bR,2)*EA*Power(eps,2)*l0h2 - 210*Arho*Power(bLp,2)*l0h4 + 7175*Arho*Power(bRp,2)*l0h4 - 192*aL*EA*Power(1 + eps,2)*(1072*aR + 27*(9*bL - 8*bR)*l0) + 420*Arho*bLp*l0h4*phiSp + 14350*Arho*bRp*l0h4*phiSp + 6965*Arho*l0h4*Power(phiSp,2) - 152880*Arho*gx*l0h3*sin(bR + phiS))/(705600.*l0h2);
    //dhLq(7,0) = 0;
    //dhLq(7,1) = 0;
    dhLq(7,2) = -(Arho*gx*l0*((8*aL + 104*aR - 3*bL*l0 + 15*bR*l0)*cos(bR + phiS) + 3*l0*(sin(bL - phiS) - 5*(3 + 4*eps)*sin(bR + phiS))))/480.;
    dhLq(7,3) = -(2*(3264*Power(aL,3)*EA*(1 + eps) - 8704*Power(aR,3)*EA*(1 + eps) - 96*Power(aR,2)*(23*bL - 67*bR)*EA*(1 + eps)*l0 - 12*aR*EA*(45*Power(bL,2)*(1 + eps) - 75*bL*bR*(1 + eps) + 4*(27*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps)))*l0h2 - 8*Power(aL,2)*EA*(1 + eps)*(1136*aR + 276*bL*l0 - 639*bR*l0) + 2*aL*EA*(2144*Power(aR,2)*(1 + eps) + 108*aR*(9*bL - 8*bR)*(1 + eps)*l0 + 9*(25*Power(bL,2)*(1 + eps) - 60*bL*bR*(1 + eps) + 27*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps))*l0h2) + l0h3*(-81*Power(bL,3)*EA*(1 + eps) + 297*Power(bL,2)*bR*EA*(1 + eps) + 243*Power(bR,3)*EA*(1 + eps) + 1890*bR*(EA + 2*EA*eps) - 9*bL*EA*(27*Power(bR,2)*(1 + eps) + 70*(1 + 2*eps)) + 1225*Arho*epsp*l0h2*(bRp + phiSp))) + 3675*Arho*gx*l0h4*cos(bR + phiS))/(29400.*l0h2);
    dhLq(7,4) = -(102912*Power(aR,2)*EA + 6209280*EI + 205824*Power(aR,2)*EA*eps + 102912*Power(aR,2)*EA*Power(eps,2) + 235008*Power(aL,2)*EA*Power(1 + eps,2) + 46656*aR*bL*EA*l0 - 41472*aR*bR*EA*l0 + 93312*aR*bL*EA*eps*l0 - 82944*aR*bR*EA*eps*l0 + 46656*aR*bL*EA*Power(eps,2)*l0 - 41472*aR*bR*EA*Power(eps,2)*l0 + 10800*Power(bL,2)*EA*l0h2 - 25920*bL*bR*EA*l0h2 + 11664*Power(bR,2)*EA*l0h2 + 60480*EA*eps*l0h2 + 21600*Power(bL,2)*EA*eps*l0h2 - 51840*bL*bR*EA*eps*l0h2 + 23328*Power(bR,2)*EA*eps*l0h2 + 60480*EA*Power(eps,2)*l0h2 + 10800*Power(bL,2)*EA*Power(eps,2)*l0h2 - 25920*bL*bR*EA*Power(eps,2)*l0h2 + 11664*Power(bR,2)*EA*Power(eps,2)*l0h2 + 1120*aLp*Arho*bRp*l0h3 + 9520*Arho*aRp*bRp*l0h3 + 1785*Arho*Power(bLp,2)*l0h4 - 420*Arho*bLp*bRp*l0h4 + 980*Arho*Power(bRp,2)*l0h4 - 384*aL*EA*Power(1 + eps,2)*(1136*aR + 276*bL*l0 - 639*bR*l0) + 1120*aLp*Arho*l0h3*phiSp + 9520*Arho*aRp*l0h3*phiSp - 3990*Arho*bLp*l0h4*phiSp + 805*Arho*l0h4*Power(phiSp,2) + 11760*Arho*gx*l0h3*sin(bR + phiS))/(705600.*l0h2);
    dhLq(7,5) = (35328*Power(aL,2)*EA - 31104*aL*aR*EA + 35328*Power(aR,2)*EA + 1411200*EI + 70656*Power(aL,2)*EA*eps - 62208*aL*aR*EA*eps + 70656*Power(aR,2)*EA*eps + 35328*Power(aL,2)*EA*Power(eps,2) - 31104*aL*aR*EA*Power(eps,2) + 35328*Power(aR,2)*EA*Power(eps,2) - 14400*aL*bL*EA*l0 + 17280*aR*bL*EA*l0 + 17280*aL*bR*EA*l0 - 14400*aR*bR*EA*l0 - 28800*aL*bL*EA*eps*l0 + 34560*aR*bL*EA*eps*l0 + 34560*aL*bR*EA*eps*l0 - 28800*aR*bR*EA*eps*l0 - 14400*aL*bL*EA*Power(eps,2)*l0 + 17280*aR*bL*EA*Power(eps,2)*l0 + 17280*aL*bR*EA*Power(eps,2)*l0 - 14400*aR*bR*EA*Power(eps,2)*l0 + 3888*Power(bL,2)*EA*l0h2 - 9504*bL*bR*EA*l0h2 + 3888*Power(bR,2)*EA*l0h2 + 20160*EA*eps*l0h2 + 7776*Power(bL,2)*EA*eps*l0h2 - 19008*bL*bR*EA*eps*l0h2 + 7776*Power(bR,2)*EA*eps*l0h2 + 20160*EA*Power(eps,2)*l0h2 + 3888*Power(bL,2)*EA*Power(eps,2)*l0h2 - 9504*bL*bR*EA*Power(eps,2)*l0h2 + 3888*Power(bR,2)*EA*Power(eps,2)*l0h2 + 280*aLp*Arho*bRp*l0h3 + 2380*Arho*aRp*bRp*l0h3 - 245*Arho*Power(bLp,2)*l0h4 - 105*Arho*bLp*bRp*l0h4 + 245*Arho*Power(bRp,2)*l0h4 + 280*aLp*Arho*l0h3*phiSp + 2380*Arho*aRp*l0h3*phiSp + 385*Arho*bLp*l0h4*phiSp - 490*Arho*l0h4*Power(phiSp,2) + 2940*Arho*gx*l0h3*sin(bL - phiS) + 2940*Arho*gx*l0h3*sin(bR + phiS))/(470400.*l0);
    dhLq(7,6) = (626688*Power(aR,2)*EA + 24272640*EI + 1253376*Power(aR,2)*EA*eps + 626688*Power(aR,2)*EA*Power(eps,2) + 218112*Power(aL,2)*EA*Power(1 + eps,2) + 105984*aR*bL*EA*l0 - 308736*aR*bR*EA*l0 + 211968*aR*bL*EA*eps*l0 - 617472*aR*bR*EA*eps*l0 + 105984*aR*bL*EA*Power(eps,2)*l0 - 308736*aR*bR*EA*Power(eps,2)*l0 + 12960*Power(bL,2)*EA*l0h2 - 21600*bL*bR*EA*l0h2 + 31104*Power(bR,2)*EA*l0h2 + 161280*EA*eps*l0h2 + 25920*Power(bL,2)*EA*eps*l0h2 - 43200*bL*bR*EA*eps*l0h2 + 62208*Power(bR,2)*EA*eps*l0h2 + 161280*EA*Power(eps,2)*l0h2 + 12960*Power(bL,2)*EA*Power(eps,2)*l0h2 - 21600*bL*bR*EA*Power(eps,2)*l0h2 + 31104*Power(bR,2)*EA*Power(eps,2)*l0h2 - 9520*aLp*Arho*bRp*l0h3 - 219520*Arho*aRp*bRp*l0h3 - 210*Arho*Power(bLp,2)*l0h4 + 3570*Arho*bLp*bRp*l0h4 - 7175*Arho*Power(bRp,2)*l0h4 - 192*aL*EA*Power(1 + eps,2)*(1072*aR + 27*(9*bL - 8*bR)*l0) - 9520*aLp*Arho*l0h3*phiSp - 219520*Arho*aRp*l0h3*phiSp + 3990*Arho*bLp*l0h4*phiSp + 6965*Arho*l0h4*Power(phiSp,2) - 152880*Arho*gx*l0h3*sin(bR + phiS))/(705600.*l0h2);
    dhLq(7,7) = -(490752*Power(aL,2)*EA - 165888*aL*aR*EA + 617472*Power(aR,2)*EA + 32175360*EI + 981504*Power(aL,2)*EA*eps - 331776*aL*aR*EA*eps + 1234944*Power(aR,2)*EA*eps + 490752*Power(aL,2)*EA*Power(eps,2) - 165888*aL*aR*EA*Power(eps,2) + 617472*Power(aR,2)*EA*Power(eps,2) - 103680*aL*bL*EA*l0 + 86400*aR*bL*EA*l0 + 93312*aL*bR*EA*l0 - 248832*aR*bR*EA*l0 - 207360*aL*bL*EA*eps*l0 + 172800*aR*bL*EA*eps*l0 + 186624*aL*bR*EA*eps*l0 - 497664*aR*bR*EA*eps*l0 - 103680*aL*bL*EA*Power(eps,2)*l0 + 86400*aR*bL*EA*Power(eps,2)*l0 + 93312*aL*bR*EA*Power(eps,2)*l0 - 248832*aR*bR*EA*Power(eps,2)*l0 + 28512*Power(bL,2)*EA*l0h2 - 46656*bL*bR*EA*l0h2 + 69984*Power(bR,2)*EA*l0h2 + 362880*EA*eps*l0h2 + 57024*Power(bL,2)*EA*eps*l0h2 - 93312*bL*bR*EA*eps*l0h2 + 139968*Power(bR,2)*EA*eps*l0h2 + 362880*EA*Power(eps,2)*l0h2 + 28512*Power(bL,2)*EA*Power(eps,2)*l0h2 - 46656*bL*bR*EA*Power(eps,2)*l0h2 + 69984*Power(bR,2)*EA*Power(eps,2)*l0h2 + 7840*aLp*Arho*bRp*l0h3 + 57400*Arho*aRp*bRp*l0h3 - 315*Arho*Power(bLp,2)*l0h4 - 2940*Arho*bLp*bRp*l0h4 + 7385*Arho*Power(bRp,2)*l0h4 + 7840*aLp*Arho*l0h3*phiSp + 57400*Arho*aRp*l0h3*phiSp - 2310*Arho*bLp*l0h4*phiSp - 7700*Arho*l0h4*Power(phiSp,2) + 5880*Arho*gx*l0h2*(8*aL + 104*aR - 3*bL*l0 + 15*bR*l0)*cos(bR + phiS) - 176400*Arho*(1 + 2*eps)*gx*l0h3*sin(bR + phiS))/(2.8224e6*l0);

    //dhLqM(0,0) = 0;
    //dhLqM(0,1) = 0;
    dhLqM(0,2) = (Arho*l0*((104*JpqpG5 + 8*JpqpG7 + 3*(20*(1 + eps)*JpqpG3 - 5*(3 + 4*eps)*JpqpG6 - JpqpG8)*l0)*cos(bL - phiS) + (8*(JpqpG5 + 13*JpqpG7) - 3*(JpqpG6 + 15*JpqpG8 + 20*(JpqpG3 + eps*JpqpG3 + eps*JpqpG8))*l0)*cos(bR + phiS) + (8*(13*aL + aR)*(JpqpG3 - JpqpG6) + 3*(-20*JpqpG4 + (5*bL - bR)*(JpqpG3 - JpqpG6))*l0)*sin(bL - phiS) - (8*(aL + 13*aR)*(JpqpG3 + JpqpG8) - 3*(-20*JpqpG4 + (bL - 5*bR)*(JpqpG3 + JpqpG8))*l0)*sin(bR + phiS)))/480.;
    dhLqM(0,3) = -(Arho*l0h2*((JpqpG3 - JpqpG6)*sin(bL - phiS) + (JpqpG3 + JpqpG8)*sin(bR + phiS)))/8.;
    dhLqM(0,4) = (Arho*l0*(13*(JpqpG3 - JpqpG6)*cos(bL - phiS) + (JpqpG3 + JpqpG8)*cos(bR + phiS)))/60.;
    dhLqM(0,5) = (Arho*l0*((-104*JpqpG5 - 8*JpqpG7 + 3*(-5*(3 + 4*eps)*JpqpG3 + 10*JpqpG6 + 20*eps*JpqpG6 + JpqpG8)*l0)*cos(bL - phiS) - 3*(JpqpG3 + JpqpG8)*l0*cos(bR + phiS) + (-8*(13*aL + aR)*(JpqpG3 - JpqpG6) + 3*(-5*bL*JpqpG3 + bR*JpqpG3 + 20*JpqpG4 + 5*bL*JpqpG6 - bR*JpqpG6)*l0)*sin(bL - phiS)))/480.;
    dhLqM(0,6) = (Arho*l0*((JpqpG3 - JpqpG6)*cos(bL - phiS) + 13*(JpqpG3 + JpqpG8)*cos(bR + phiS)))/60.;
    dhLqM(0,7) = (Arho*l0*(3*(-JpqpG3 + JpqpG6)*l0*cos(bL - phiS) + (8*(JpqpG5 + 13*JpqpG7) - 3*(5*(3 + 4*eps)*JpqpG3 + JpqpG6 + 10*JpqpG8 + 20*eps*JpqpG8)*l0)*cos(bR + phiS) - (8*(aL + 13*aR)*(JpqpG3 + JpqpG8) - 3*(-20*JpqpG4 + (bL - 5*bR)*(JpqpG3 + JpqpG8))*l0)*sin(bR + phiS)))/480.;
    //dhLqM(1,0) = 0;
    //dhLqM(1,1) = 0;
    dhLqM(1,2) = (Arho*l0*((8*(13*aL + aR)*(JpqpG3 - JpqpG6) + 3*(-20*JpqpG4 + (5*bL - bR)*(JpqpG3 - JpqpG6))*l0)*cos(bL - phiS) + (8*(aL + 13*aR)*(JpqpG3 + JpqpG8) - 3*(-20*JpqpG4 + (bL - 5*bR)*(JpqpG3 + JpqpG8))*l0)*cos(bR + phiS) + (-104*JpqpG5 - 8*JpqpG7 + 3*(-20*(1 + eps)*JpqpG3 + 5*(3 + 4*eps)*JpqpG6 + JpqpG8)*l0)*sin(bL - phiS) + (8*JpqpG5 + 104*JpqpG7 - 3*(JpqpG6 + 15*JpqpG8 + 20*(JpqpG3 + eps*JpqpG3 + eps*JpqpG8))*l0)*sin(bR + phiS)))/480.;
    dhLqM(1,3) = (Arho*l0h2*((-JpqpG3 + JpqpG6)*cos(bL - phiS) + (JpqpG3 + JpqpG8)*cos(bR + phiS)))/8.;
    dhLqM(1,4) = (Arho*l0*(13*(-JpqpG3 + JpqpG6)*sin(bL - phiS) + (JpqpG3 + JpqpG8)*sin(bR + phiS)))/60.;
    dhLqM(1,5) = (Arho*l0*((-8*(13*aL + aR)*(JpqpG3 - JpqpG6) + 3*(-5*bL*JpqpG3 + bR*JpqpG3 + 20*JpqpG4 + 5*bL*JpqpG6 - bR*JpqpG6)*l0)*cos(bL - phiS) + (8*(13*JpqpG5 + JpqpG7) + 3*(5*(3*JpqpG3 + 4*eps*JpqpG3 - 2*JpqpG6 - 4*eps*JpqpG6) - JpqpG8)*l0)*sin(bL - phiS) - 3*(JpqpG3 + JpqpG8)*l0*sin(bR + phiS)))/480.;
    dhLqM(1,6) = (Arho*l0*((-JpqpG3 + JpqpG6)*sin(bL - phiS) + 13*(JpqpG3 + JpqpG8)*sin(bR + phiS)))/60.;
    dhLqM(1,7) = (Arho*l0*((8*(aL + 13*aR)*(JpqpG3 + JpqpG8) - 3*(-20*JpqpG4 + (bL - 5*bR)*(JpqpG3 + JpqpG8))*l0)*cos(bR + phiS) + 3*(JpqpG3 - JpqpG6)*l0*sin(bL - phiS) + (8*(JpqpG5 + 13*JpqpG7) - 3*(5*(3 + 4*eps)*JpqpG3 + JpqpG6 + 10*JpqpG8 + 20*eps*JpqpG8)*l0)*sin(bR + phiS)))/480.;
    //dhLqM(2,0) = 0;
    //dhLqM(2,1) = 0;
    dhLqM(2,2) = (Arho*l0*((60*(1 + eps)*JpqpG1*l0 + JpqpG2*(104*aL + 8*aR + 15*bL*l0 - 3*bR*l0))*cos(bL - phiS) + (8*(aL + 13*aR)*JpqpG2 - 3*(20*(1 + eps)*JpqpG1 + (bL - 5*bR)*JpqpG2)*l0)*cos(bR + phiS) + (8*(13*aL + aR)*JpqpG1 + 3*(5*bL*JpqpG1 - bR*JpqpG1 - 20*(1 + eps)*JpqpG2)*l0)*sin(bL - phiS) - (8*aL*JpqpG1 + 104*aR*JpqpG1 + 3*(-(bL*JpqpG1) + 5*bR*JpqpG1 + 20*(1 + eps)*JpqpG2)*l0)*sin(bR + phiS)))/480.;
    dhLqM(2,3) = (Arho*l0h2*(9*JpqpG5 - 9*JpqpG7 + (20*(1 + eps)*JpqpG3 - (9 + 10*eps)*(JpqpG6 - JpqpG8))*l0 + 15*JpqpG2*(-cos(bL - phiS) + cos(bR + phiS)) - 15*JpqpG1*(sin(bL - phiS) + sin(bR + phiS))))/120.;
    dhLqM(2,4) = (Arho*l0*(16*aL*(197*JpqpG3 - 196*JpqpG6 + JpqpG8) + 136*aR*(2*JpqpG3 - JpqpG6 + JpqpG8) + (-756*JpqpG4 + bL*(199*JpqpG3 - 205*JpqpG6 - 6*JpqpG8) + bR*(-23*JpqpG3 + 51*JpqpG6 + 28*JpqpG8))*l0 + 168*JpqpG1*(13*cos(bL - phiS) + cos(bR + phiS)) + 168*JpqpG2*(-13*sin(bL - phiS) + sin(bR + phiS))))/10080.;
    dhLqM(2,5) = (Arho*l0*(l0*(4*aL*(199*JpqpG3 - 205*JpqpG6 - 6*JpqpG8) - 4*aR*(23*JpqpG3 + 28*JpqpG6 + 51*JpqpG8) + (bL*(220*JpqpG3 - 211*JpqpG6 + 9*JpqpG8) - 42*(8*JpqpG4 + bR*(2*JpqpG3 - JpqpG6 + JpqpG8)))*l0) - 84*((8*(13*aL + aR)*JpqpG2 + 3*(5*(3 + 4*eps)*JpqpG1 + (5*bL - bR)*JpqpG2)*l0)*cos(bL - phiS) + 3*JpqpG1*l0*cos(bR + phiS) + (8*(13*aL + aR)*JpqpG1 + 3*(5*bL*JpqpG1 - bR*JpqpG1 - 5*(3 + 4*eps)*JpqpG2)*l0)*sin(bL - phiS) + 3*JpqpG2*l0*sin(bR + phiS))))/40320.;
    dhLqM(2,6) = (Arho*l0*(8*(17*aL*(2*JpqpG3 - JpqpG6 + JpqpG8) + 2*aR*(197*JpqpG3 - JpqpG6 + 196*JpqpG8)) + (756*JpqpG4 - bL*(23*JpqpG3 + 28*JpqpG6 + 51*JpqpG8) + bR*(199*JpqpG3 + 6*JpqpG6 + 205*JpqpG8))*l0 + 168*(JpqpG1*cos(bL - phiS) + 13*JpqpG1*cos(bR + phiS) - JpqpG2*sin(bL - phiS) + 13*JpqpG2*sin(bR + phiS))))/10080.;
    dhLqM(2,7) = (Arho*l0*(8*(aL*(-23*JpqpG3 + 51*JpqpG6 + 28*JpqpG8) + aR*(199*JpqpG3 + 6*JpqpG6 + 205*JpqpG8))*l0 + (672*JpqpG4 - 84*bL*(2*JpqpG3 - JpqpG6 + JpqpG8) + 2*bR*(220*JpqpG3 - 9*JpqpG6 + 211*JpqpG8))*l0h2 + 168*(-3*JpqpG1*l0*cos(bL - phiS) + (8*(aL + 13*aR)*JpqpG2 - 3*(5*(3 + 4*eps)*JpqpG1 + (bL - 5*bR)*JpqpG2)*l0)*cos(bR + phiS) - 8*(aL + 13*aR)*JpqpG1*sin(bR + phiS) + 3*l0*(JpqpG2*sin(bL - phiS) + ((bL - 5*bR)*JpqpG1 - 5*(3 + 4*eps)*JpqpG2)*sin(bR + phiS)))))/80640.;
    //dhLqM(3,0) = 0;
    //dhLqM(3,1) = 0;
    dhLqM(3,2) = -(Arho*l0h2*(JpqpG2*cos(bL - phiS) - JpqpG2*cos(bR + phiS) + JpqpG1*(sin(bL - phiS) + sin(bR + phiS))))/8.;
    //dhLqM(3,3) = 0;
    dhLqM(3,4) = (Arho*(-18*JpqpG3 + 19*JpqpG6 + JpqpG8)*l0h2)/240.;
    dhLqM(3,5) = (Arho*l0h2*((-16*JpqpG3 + 13*JpqpG6 - 3*JpqpG8)*l0 + 240*JpqpG2*cos(bL - phiS) + 240*JpqpG1*sin(bL - phiS)))/1920.;
    dhLqM(3,6) = (Arho*(18*JpqpG3 + JpqpG6 + 19*JpqpG8)*l0h2)/240.;
    dhLqM(3,7) = (Arho*l0h2*((16*JpqpG3 - 3*JpqpG6 + 13*JpqpG8)*l0 + 240*JpqpG2*cos(bR + phiS) - 240*JpqpG1*sin(bR + phiS)))/1920.;
    //dhLqM(4,0) = 0;
    //dhLqM(4,1) = 0;
    dhLqM(4,2) = (Arho*l0*(13*JpqpG1*cos(bL - phiS) + JpqpG1*cos(bR + phiS) + JpqpG2*(-13*sin(bL - phiS) + sin(bR + phiS))))/60.;
    dhLqM(4,3) = (Arho*(18*JpqpG3 - 19*JpqpG6 - JpqpG8)*l0h2)/240.;
    //dhLqM(4,4) = 0;
    dhLqM(4,5) = (-13*Arho*l0*(JpqpG1*cos(bL - phiS) - JpqpG2*sin(bL - phiS)))/60.;
    //dhLqM(4,6) = 0;
    dhLqM(4,7) = (Arho*l0*(JpqpG1*cos(bR + phiS) + JpqpG2*sin(bR + phiS)))/60.;
    //dhLqM(5,0) = 0;
    //dhLqM(5,1) = 0;
    dhLqM(5,2) = -(Arho*l0*((8*(13*aL + aR)*JpqpG2 + 3*(5*(3 + 4*eps)*JpqpG1 + (5*bL - bR)*JpqpG2)*l0)*cos(bL - phiS) + 3*JpqpG1*l0*cos(bR + phiS) + (8*(13*aL + aR)*JpqpG1 + 3*(5*bL*JpqpG1 - bR*JpqpG1 - 5*(3 + 4*eps)*JpqpG2)*l0)*sin(bL - phiS) + 3*JpqpG2*l0*sin(bR + phiS)))/480.;
    dhLqM(5,3) = (Arho*l0h2*(-76*JpqpG5 - 4*JpqpG7 + (-8*(9 + 10*eps)*JpqpG3 + 67*JpqpG6 + 80*eps*JpqpG6 + 3*JpqpG8)*l0 + 120*JpqpG2*cos(bL - phiS) + 120*JpqpG1*sin(bL - phiS)))/960.;
    dhLqM(5,4) = (Arho*l0*(-8*(392*aL + 17*aR)*(JpqpG3 - JpqpG6) - (-798*JpqpG4 + (205*bL - 51*bR)*(JpqpG3 - JpqpG6))*l0 - 2184*JpqpG1*cos(bL - phiS) + 2184*JpqpG2*sin(bL - phiS)))/10080.;
    dhLqM(5,5) = (Arho*l0*(l0*(-4*(205*aL + 28*aR)*(JpqpG3 - JpqpG6) - (-273*JpqpG4 + (211*bL - 42*bR)*(JpqpG3 - JpqpG6))*l0) + 84*(8*(13*aL + aR)*JpqpG2 + 3*(10*JpqpG1 + 20*eps*JpqpG1 + 5*bL*JpqpG2 - bR*JpqpG2)*l0)*cos(bL - phiS) + 84*(8*(13*aL + aR)*JpqpG1 + 3*(5*bL*JpqpG1 - bR*JpqpG1 - 10*(JpqpG2 + 2*eps*JpqpG2))*l0)*sin(bL - phiS)))/40320.;
    dhLqM(5,6) = (Arho*l0*(-4*(17*aL + 2*aR)*(JpqpG3 - JpqpG6) - (-21*JpqpG4 + (14*bL - 3*bR)*(JpqpG3 - JpqpG6))*l0 - 84*JpqpG1*cos(bL - phiS) + 84*JpqpG2*sin(bL - phiS)))/5040.;
    dhLqM(5,7) = (Arho*l0h2*(4*(17*aL + 2*aR)*(JpqpG3 - JpqpG6) + (-21*JpqpG4 + (14*bL - 3*bR)*(JpqpG3 - JpqpG6))*l0 + 84*JpqpG1*(cos(bL - phiS) - cos(bR + phiS)) - 84*JpqpG2*(sin(bL - phiS) + sin(bR + phiS))))/13440.;
    //dhLqM(6,0) = 0;
    //dhLqM(6,1) = 0;
    dhLqM(6,2) = (Arho*l0*(JpqpG1*cos(bL - phiS) + 13*JpqpG1*cos(bR + phiS) - JpqpG2*sin(bL - phiS) + 13*JpqpG2*sin(bR + phiS)))/60.;
    dhLqM(6,3) = -(Arho*(18*JpqpG3 + JpqpG6 + 19*JpqpG8)*l0h2)/240.;
    dhLqM(6,4) = 0;
    dhLqM(6,5) = (Arho*l0*(-(JpqpG1*cos(bL - phiS)) + JpqpG2*sin(bL - phiS)))/60.;
    //dhLqM(6,6) = 0;
    dhLqM(6,7) = (13*Arho*l0*(JpqpG1*cos(bR + phiS) + JpqpG2*sin(bR + phiS)))/60.;
    //dhLqM(7,0) = 0;
    //dhLqM(7,1) = 0;
    dhLqM(7,2) = (Arho*l0*(-3*JpqpG1*l0*cos(bL - phiS) + (8*(aL + 13*aR)*JpqpG2 - 3*(5*(3 + 4*eps)*JpqpG1 + (bL - 5*bR)*JpqpG2)*l0)*cos(bR + phiS) - 8*(aL + 13*aR)*JpqpG1*sin(bR + phiS) + 3*l0*(JpqpG2*sin(bL - phiS) + ((bL - 5*bR)*JpqpG1 - 5*(3 + 4*eps)*JpqpG2)*sin(bR + phiS))))/480.;
    dhLqM(7,3) = (Arho*l0h2*(-4*(JpqpG5 + 19*JpqpG7) + (72*JpqpG3 + 80*eps*JpqpG3 + 3*JpqpG6 + 67*JpqpG8 + 80*eps*JpqpG8)*l0 + 120*JpqpG2*cos(bR + phiS) - 120*JpqpG1*sin(bR + phiS)))/960.;
    dhLqM(7,4) = (Arho*l0*(8*aL*(JpqpG3 + JpqpG8) + 68*aR*(JpqpG3 + JpqpG8) + (-3*bL*(JpqpG3 + JpqpG8) + 7*(3*JpqpG4 + 2*bR*(JpqpG3 + JpqpG8)))*l0 + 84*JpqpG1*cos(bR + phiS) + 84*JpqpG2*sin(bR + phiS)))/5040.;
    dhLqM(7,5) = (Arho*l0h2*(-4*(2*aL + 17*aR)*(JpqpG3 + JpqpG8) + (-21*JpqpG4 + (3*bL - 14*bR)*(JpqpG3 + JpqpG8))*l0 + 84*JpqpG1*(cos(bL - phiS) - cos(bR + phiS)) - 84*JpqpG2*(sin(bL - phiS) + sin(bR + phiS))))/13440.;
    dhLqM(7,6) = (Arho*l0*(64*(17*aL + 392*aR)*(JpqpG3 + JpqpG8) - 8*(-798*JpqpG4 + (51*bL - 205*bR)*(JpqpG3 + JpqpG8))*l0 + 17472*(JpqpG1*cos(bR + phiS) + JpqpG2*sin(bR + phiS))))/80640.;
    dhLqM(7,7) = (Arho*l0*(l0*(112*aL*(JpqpG3 + JpqpG8) + 820*aR*(JpqpG3 + JpqpG8) + (273*JpqpG4 - 42*bL*(JpqpG3 + JpqpG8) + 211*bR*(JpqpG3 + JpqpG8))*l0) + 84*(8*(aL + 13*aR)*JpqpG2 - 3*(10*(1 + 2*eps)*JpqpG1 + (bL - 5*bR)*JpqpG2)*l0)*cos(bR + phiS) - 84*(8*aL*JpqpG1 + 104*aR*JpqpG1 + 3*(-(bL*JpqpG1) + 5*bR*JpqpG1 + 10*JpqpG2 + 20*eps*JpqpG2)*l0)*sin(bR + phiS)))/40320.;

    //dhLqJp(0,0) = 0;
    dhLqJp(0,1) = (Power(phi1p - phi2p,2)*Power(Sec((phi1 - phi2)/2.),2)*tan((phi1 - phi2)/2.))/4.;
    dhLqJp(0,2) = (Power(Sec((phi1 - phi2)/2.),4)*(144*Power(phi1p - phi2p,2)*(y1 - y2) + (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*Power(phi1p - 2*phi2p,2)*cos(phi1) + Power(phi1p,2)*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi1 - 2*phi2) - 72*Power(phi1p,2)*y1*cos(phi1 - phi2) + 144*phi1p*phi2p*y1*cos(phi1 - phi2) - 72*Power(phi2p,2)*y1*cos(phi1 - phi2) + 72*Power(phi1p,2)*y2*cos(phi1 - phi2) - 144*phi1p*phi2p*y2*cos(phi1 - phi2) + 72*Power(phi2p,2)*y2*cos(phi1 - phi2) - 256*a1*Power(phi1p,2)*cos(phi2) - 256*a2*Power(phi1p,2)*cos(phi2) + 20*l0*phi1*Power(phi1p,2)*cos(phi2) - 20*l0*Power(phi1p,2)*phi2*cos(phi2) + 512*a1*phi1p*phi2p*cos(phi2) + 512*a2*phi1p*phi2p*cos(phi2) - 40*l0*phi1*phi1p*phi2p*cos(phi2) + 40*l0*phi1p*phi2*phi2p*cos(phi2) - 128*a1*Power(phi2p,2)*cos(phi2) - 128*a2*Power(phi2p,2)*cos(phi2) + 10*l0*phi1*Power(phi2p,2)*cos(phi2) - 10*l0*phi2*Power(phi2p,2)*cos(phi2) - 128*a1p*phi1p*sin(phi1) - 128*a2p*phi1p*sin(phi1) + 15*l0*Power(phi1p,2)*sin(phi1) + 256*a1p*phi2p*sin(phi1) + 256*a2p*phi2p*sin(phi1) - 50*l0*phi1p*phi2p*sin(phi1) + 20*l0*Power(phi2p,2)*sin(phi1) - 128*a1p*phi1p*sin(phi1 - 2*phi2) - 128*a2p*phi1p*sin(phi1 - 2*phi2) + 15*l0*Power(phi1p,2)*sin(phi1 - 2*phi2) - 10*l0*phi1p*phi2p*sin(phi1 - 2*phi2) + 144*phi1p*y1p*sin(phi1 - phi2) - 144*phi2p*y1p*sin(phi1 - phi2) - 144*phi1p*y2p*sin(phi1 - phi2) + 144*phi2p*y2p*sin(phi1 - phi2) - 5*l0*Power(phi2p,2)*sin(2*phi1 - phi2) + phi2p*(256*a1p + 256*a2p - 40*l0*phi1p + 25*l0*phi2p)*sin(phi2)))/576.;
    dhLqJp(0,3) = (-2*Power(Sec((phi1 - phi2)/2.),3)*(Power(phi1p,2)*sin((phi1 - 3*phi2)/2.) - Power(phi2p,2)*sin((3*phi1 - phi2)/2.) + (Power(phi1p,2) - 4*phi1p*phi2p + Power(phi2p,2))*sin((phi1 + phi2)/2.)))/9.;
    dhLqJp(0,4) = (-2*Power(Sec((phi1 - phi2)/2.),3)*(Power(phi1p,2)*sin((phi1 - 3*phi2)/2.) - Power(phi2p,2)*sin((3*phi1 - phi2)/2.) + (Power(phi1p,2) - 4*phi1p*phi2p + Power(phi2p,2))*sin((phi1 + phi2)/2.)))/9.;
    //dhLqJp(0,5) = 0;
    dhLqJp(0,6) = -(Power(phi1p - phi2p,2)*Power(Sec((phi1 - phi2)/2.),2)*tan((phi1 - phi2)/2.))/4.;
    dhLqJp(0,7) = (Power(Sec((phi1 - phi2)/2.),4)*(144*Power(phi1p - phi2p,2)*(-y1 + y2) - 2*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*(Power(phi1p,2) - 4*phi1p*phi2p + 2*Power(phi2p,2))*cos(phi1) + 72*Power(phi1p - phi2p,2)*(y1 - y2)*cos(phi1 - phi2) + 64*a1*Power(phi2p,2)*cos(2*phi1 - phi2) + 64*a2*Power(phi2p,2)*cos(2*phi1 - phi2) - 5*l0*phi1*Power(phi2p,2)*cos(2*phi1 - phi2) + 5*l0*phi2*Power(phi2p,2)*cos(2*phi1 - phi2) + 256*a1*Power(phi1p,2)*cos(phi2) + 256*a2*Power(phi1p,2)*cos(phi2) - 20*l0*phi1*Power(phi1p,2)*cos(phi2) + 20*l0*Power(phi1p,2)*phi2*cos(phi2) - 256*a1*phi1p*phi2p*cos(phi2) - 256*a2*phi1p*phi2p*cos(phi2) + 20*l0*phi1*phi1p*phi2p*cos(phi2) - 20*l0*phi1p*phi2*phi2p*cos(phi2) + 64*a1*Power(phi2p,2)*cos(phi2) + 64*a2*Power(phi2p,2)*cos(phi2) - 5*l0*phi1*Power(phi2p,2)*cos(phi2) + 5*l0*phi2*Power(phi2p,2)*cos(phi2) + 256*a1p*phi1p*sin(phi1) + 256*a2p*phi1p*sin(phi1) - 25*l0*Power(phi1p,2)*sin(phi1) + 40*l0*phi1p*phi2p*sin(phi1) - 5*l0*Power(phi1p,2)*sin(phi1 - 2*phi2) - 144*phi1p*y1p*sin(phi1 - phi2) + 144*phi2p*y1p*sin(phi1 - phi2) + 144*phi1p*y2p*sin(phi1 - phi2) - 144*phi2p*y2p*sin(phi1 - phi2) + 128*a1p*phi2p*sin(2*phi1 - phi2) + 128*a2p*phi2p*sin(2*phi1 - phi2) - 10*l0*phi1p*phi2p*sin(2*phi1 - phi2) + 15*l0*Power(phi2p,2)*sin(2*phi1 - phi2) + (4*phi1p*(64*(a1p + a2p) - 5*l0*phi1p) - 2*(64*(a1p + a2p) - 25*l0*phi1p)*phi2p - 15*l0*Power(phi2p,2))*sin(phi2)))/576.;
    dhLqJp(1,0) = -(Power(phi1p - phi2p,2)*sin(phi1 - phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    //dhLqJp(1,1) = 0;
    dhLqJp(1,2) = -(Power(Sec((phi1 - phi2)/2.),4)*(144*Power(phi1p - phi2p,2)*(x1 - x2) + (-128*a1p*(phi1p - 2*phi2p) - 128*a2p*(phi1p - 2*phi2p) + 5*l0*(3*Power(phi1p,2) - 10*phi1p*phi2p + 4*Power(phi2p,2)))*cos(phi1) + phi1p*(128*a1p + 128*a2p + 5*l0*(-3*phi1p + 2*phi2p))*cos(phi1 - 2*phi2) - 72*Power(phi1p,2)*x1*cos(phi1 - phi2) + 144*phi1p*phi2p*x1*cos(phi1 - phi2) - 72*Power(phi2p,2)*x1*cos(phi1 - phi2) + 72*Power(phi1p,2)*x2*cos(phi1 - phi2) - 144*phi1p*phi2p*x2*cos(phi1 - phi2) + 72*Power(phi2p,2)*x2*cos(phi1 - phi2) - 5*l0*Power(phi2p,2)*cos(2*phi1 - phi2) + 256*a1p*phi2p*cos(phi2) + 256*a2p*phi2p*cos(phi2) - 40*l0*phi1p*phi2p*cos(phi2) + 25*l0*Power(phi2p,2)*cos(phi2) - 64*a1*Power(phi1p,2)*sin(phi1) - 64*a2*Power(phi1p,2)*sin(phi1) + 5*l0*phi1*Power(phi1p,2)*sin(phi1) - 5*l0*Power(phi1p,2)*phi2*sin(phi1) + 256*a1*phi1p*phi2p*sin(phi1) + 256*a2*phi1p*phi2p*sin(phi1) - 20*l0*phi1*phi1p*phi2p*sin(phi1) + 20*l0*phi1p*phi2*phi2p*sin(phi1) - 256*a1*Power(phi2p,2)*sin(phi1) - 256*a2*Power(phi2p,2)*sin(phi1) + 20*l0*phi1*Power(phi2p,2)*sin(phi1) - 20*l0*phi2*Power(phi2p,2)*sin(phi1) + 64*a1*Power(phi1p,2)*sin(phi1 - 2*phi2) + 64*a2*Power(phi1p,2)*sin(phi1 - 2*phi2) - 5*l0*phi1*Power(phi1p,2)*sin(phi1 - 2*phi2) + 5*l0*Power(phi1p,2)*phi2*sin(phi1 - 2*phi2) + 144*phi1p*x1p*sin(phi1 - phi2) - 144*phi2p*x1p*sin(phi1 - phi2) - 144*phi1p*x2p*sin(phi1 - phi2) + 144*phi2p*x2p*sin(phi1 - phi2) + 2*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*(2*Power(phi1p,2) - 4*phi1p*phi2p + Power(phi2p,2))*sin(phi2)))/576.;
    dhLqJp(1,3) = (4*(phi1p*(phi1p - 4*phi2p)*cos(phi1) - phi1p*(phi1p*cos(phi1 - 2*phi2) + 4*phi2p*cos(phi2)) + 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2)))/(9.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(1,4) = (4*(phi1p*(phi1p - 4*phi2p)*cos(phi1) - phi1p*(phi1p*cos(phi1 - 2*phi2) + 4*phi2p*cos(phi2)) + 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2)))/(9.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(1,5) = (Power(phi1p - phi2p,2)*sin(phi1 - phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    //dhLqJp(1,6) = 0;
    dhLqJp(1,7) = -(Power(Sec((phi1 - phi2)/2.),4)*(144*Power(phi1p - phi2p,2)*(-x1 + x2) + phi1p*(256*a1p + 256*a2p - 25*l0*phi1p + 40*l0*phi2p)*cos(phi1) + 5*l0*Power(phi1p,2)*cos(phi1 - 2*phi2) + 72*Power(phi1p,2)*x1*cos(phi1 - phi2) - 144*phi1p*phi2p*x1*cos(phi1 - phi2) + 72*Power(phi2p,2)*x1*cos(phi1 - phi2) - 72*Power(phi1p,2)*x2*cos(phi1 - phi2) + 144*phi1p*phi2p*x2*cos(phi1 - phi2) - 72*Power(phi2p,2)*x2*cos(phi1 - phi2) + 128*a1p*phi2p*cos(2*phi1 - phi2) + 128*a2p*phi2p*cos(2*phi1 - phi2) - 10*l0*phi1p*phi2p*cos(2*phi1 - phi2) + 15*l0*Power(phi2p,2)*cos(2*phi1 - phi2) + 256*a1p*phi1p*cos(phi2) + 256*a2p*phi1p*cos(phi2) - 20*l0*Power(phi1p,2)*cos(phi2) - 128*a1p*phi2p*cos(phi2) - 128*a2p*phi2p*cos(phi2) + 50*l0*phi1p*phi2p*cos(phi2) - 15*l0*Power(phi2p,2)*cos(phi2) + 128*a1*Power(phi1p,2)*sin(phi1) + 128*a2*Power(phi1p,2)*sin(phi1) - 10*l0*phi1*Power(phi1p,2)*sin(phi1) + 10*l0*Power(phi1p,2)*phi2*sin(phi1) - 512*a1*phi1p*phi2p*sin(phi1) - 512*a2*phi1p*phi2p*sin(phi1) + 40*l0*phi1*phi1p*phi2p*sin(phi1) - 40*l0*phi1p*phi2*phi2p*sin(phi1) + 256*a1*Power(phi2p,2)*sin(phi1) + 256*a2*Power(phi2p,2)*sin(phi1) - 20*l0*phi1*Power(phi2p,2)*sin(phi1) + 20*l0*phi2*Power(phi2p,2)*sin(phi1) - 144*phi1p*x1p*sin(phi1 - phi2) + 144*phi2p*x1p*sin(phi1 - phi2) + 144*phi1p*x2p*sin(phi1 - phi2) - 144*phi2p*x2p*sin(phi1 - phi2) - 64*a1*Power(phi2p,2)*sin(2*phi1 - phi2) - 64*a2*Power(phi2p,2)*sin(2*phi1 - phi2) + 5*l0*phi1*Power(phi2p,2)*sin(2*phi1 - phi2) - 5*l0*phi2*Power(phi2p,2)*sin(2*phi1 - phi2) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*Power(-2*phi1p + phi2p,2)*sin(phi2)))/576.;
    dhLqJp(2,0) = (11*(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2))))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(2,1) = (-11*(phi1p*(phi1p - 4*phi2p)*cos(phi1) - phi1p*(phi1p*cos(phi1 - 2*phi2) + 4*phi2p*cos(phi2)) + 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2)))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(2,2) = (-11*Power(Sec((phi1 - phi2)/2.),4)*(2*(-2*Power(phi1p,2)*(x1 - x2) + 4*phi1p*phi2p*(x1 - x2) + phi2p*(phi2p*(-x1 + x2) - 2*y1p + 2*y2p))*cos(phi2) + (phi1p - 2*phi2p)*(((phi1p - 2*phi2p)*(x1 - x2) + 2*(y1p - y2p))*cos(phi1) + (-2*x1p + 2*x2p + (phi1p - 2*phi2p)*(y1 - y2))*sin(phi1)) + phi1p*cos(2*phi2)*((phi1p*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi1) + (-2*x1p + 2*x2p - phi1p*y1 + phi1p*y2)*sin(phi1)) + 2*(2*Power(phi1p,2)*(-y1 + y2) + Power(phi2p,2)*(-y1 + y2) + 2*phi2p*(x1p - x2p + 2*phi1p*y1 - 2*phi1p*y2))*sin(phi2) + phi1p*((2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - 2*y1p + 2*y2p)*sin(phi1))*sin(2*phi2)))/(32.*l0);
    //dhLqJp(2,3) = 0;
    //dhLqJp(2,4) = 0;
    dhLqJp(2,5) = (-11*(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2))))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(2,6) = (-11*(2*phi1p*phi2p*cos(phi1) + 2*phi1p*phi2p*cos(phi2) + sin(phi1 - phi2)*(-(Power(phi2p,2)*sin(phi1)) + Power(phi1p,2)*sin(phi2))))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(2,7) = (11*Power(Sec((phi1 - phi2)/2.),4)*(2*(Power(phi1p,2)*(x1 - x2) + 2*Power(phi2p,2)*(x1 - x2) - 2*phi1p*(2*phi2p*x1 - 2*phi2p*x2 - y1p + y2p))*cos(phi1) + phi2p*(-(phi2p*x1) + phi2p*x2 + 2*y1p - 2*y2p)*cos(2*phi1 - phi2) + 2*(Power(phi1p,2)*(y1 - y2) + 2*Power(phi2p,2)*(y1 - y2) - 2*phi1p*(x1p - x2p + 2*phi2p*y1 - 2*phi2p*y2))*sin(phi1) - phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(phi2)*sin(2*phi1) + phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1)*sin(phi2) + (2*phi1p - phi2p)*((-2*phi1p*x1 + phi2p*x1 + 2*phi1p*x2 - phi2p*x2 + 2*y1p - 2*y2p)*cos(phi2) + (-2*x1p + 2*x2p - 2*phi1p*y1 + phi2p*y1 + 2*phi1p*y2 - phi2p*y2)*sin(phi2))))/(32.*l0);
    dhLqJp(3,0) = (-(phi1p*(phi1p - 4*phi2p)*cos(phi1)) + Power(phi1p,2)*cos(phi1 - 2*phi2) + 4*phi1p*phi2p*cos(phi2) - 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2))/(2.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(3,1) = -(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2)))/(2.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(3,2) = (Power(Sec((phi1 - phi2)/2.),4)*(2*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*Power(phi1p - phi2p,2) + 18*phi1p*cos(2*phi2)*((2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - 2*y1p + 2*y2p)*sin(phi1)) + 18*(phi1p - 2*phi2p)*((-2*x1p + 2*x2p + (phi1p - 2*phi2p)*(y1 - y2))*cos(phi1) + (-((phi1p - 2*phi2p)*(x1 - x2)) - 2*y1p + 2*y2p)*sin(phi1)) + cos(phi2)*(-36*(2*Power(phi1p,2)*(y1 - y2) + Power(phi2p,2)*(y1 - y2) - 2*phi2p*(x1p - x2p + 2*phi1p*y1 - 2*phi1p*y2)) + (phi1p - phi2p)*(128*(a1p + a2p)*sin(phi1) - (phi1p - phi2p)*((64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi1) + 15*l0*sin(phi1)))) + (36*(2*Power(phi1p,2)*(x1 - x2) + 4*phi1p*phi2p*(-x1 + x2) + phi2p*(phi2p*x1 - phi2p*x2 + 2*y1p - 2*y2p)) + (-128*a1p - 128*a2p + 15*l0*(phi1p - phi2p))*(phi1p - phi2p)*cos(phi1) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*Power(phi1p - phi2p,2)*sin(phi1))*sin(phi2) + 18*phi1p*((-(phi1p*x1) + phi1p*x2 + 2*y1p - 2*y2p)*cos(phi1) + (2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*sin(phi1))*sin(2*phi2)))/(144.*l0);
    dhLqJp(3,3) = (16*Power(phi1p - phi2p,2)*sin(phi1 - phi2))/(9.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(3,4) = (16*Power(phi1p - phi2p,2)*sin(phi1 - phi2))/(9.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(3,5) = (phi1p*(phi1p - 4*phi2p)*cos(phi1) - phi1p*(phi1p*cos(phi1 - 2*phi2) + 4*phi2p*cos(phi2)) + 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2))/(2.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(3,6) = (phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2)))/(2.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(3,7) = (Power(Sec((phi1 - phi2)/2.),4)*(-2*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*Power(phi1p - phi2p,2) - 36*(Power(phi1p,2)*(y1 - y2) + 2*Power(phi2p,2)*(y1 - y2) - 2*phi1p*(x1p - x2p + 2*phi2p*y1 - 2*phi2p*y2))*cos(phi1) + (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*Power(phi1p - phi2p,2)*cos(phi1 - phi2) + 36*phi2p*x1p*cos(2*phi1 - phi2) - 36*phi2p*x2p*cos(2*phi1 - phi2) + 18*Power(phi2p,2)*y1*cos(2*phi1 - phi2) - 18*Power(phi2p,2)*y2*cos(2*phi1 - phi2) + 72*phi1p*x1p*cos(phi2) - 36*phi2p*x1p*cos(phi2) - 72*phi1p*x2p*cos(phi2) + 36*phi2p*x2p*cos(phi2) + 72*Power(phi1p,2)*y1*cos(phi2) - 72*phi1p*phi2p*y1*cos(phi2) + 18*Power(phi2p,2)*y1*cos(phi2) - 72*Power(phi1p,2)*y2*cos(phi2) + 72*phi1p*phi2p*y2*cos(phi2) - 18*Power(phi2p,2)*y2*cos(phi2) + 36*Power(phi1p,2)*x1*sin(phi1) - 144*phi1p*phi2p*x1*sin(phi1) + 72*Power(phi2p,2)*x1*sin(phi1) - 36*Power(phi1p,2)*x2*sin(phi1) + 144*phi1p*phi2p*x2*sin(phi1) - 72*Power(phi2p,2)*x2*sin(phi1) + 72*phi1p*y1p*sin(phi1) - 72*phi1p*y2p*sin(phi1) - 128*a1p*phi1p*sin(phi1 - phi2) - 128*a2p*phi1p*sin(phi1 - phi2) + 15*l0*Power(phi1p,2)*sin(phi1 - phi2) + 128*a1p*phi2p*sin(phi1 - phi2) + 128*a2p*phi2p*sin(phi1 - phi2) - 30*l0*phi1p*phi2p*sin(phi1 - phi2) + 15*l0*Power(phi2p,2)*sin(phi1 - phi2) - 18*Power(phi2p,2)*x1*sin(2*phi1 - phi2) + 18*Power(phi2p,2)*x2*sin(2*phi1 - phi2) + 36*phi2p*y1p*sin(2*phi1 - phi2) - 36*phi2p*y2p*sin(2*phi1 - phi2) - 18*(-2*phi1p + phi2p)*(-2*phi1p*x1 + phi2p*x1 + 2*phi1p*x2 - phi2p*x2 + 2*y1p - 2*y2p)*sin(phi2)))/(144.*l0);
    dhLqJp(4,0) = (phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2)))/(4.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(4,1) = (-(phi1p*(phi1p - 4*phi2p)*cos(phi1)) + Power(phi1p,2)*cos(phi1 - 2*phi2) + 4*phi1p*phi2p*cos(phi2) - 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2))/(4.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(4,2) = -(Power(Sec((phi1 - phi2)/2.),4)*(2*(-2*Power(phi1p,2)*(x1 - x2) + 4*phi1p*phi2p*(x1 - x2) + phi2p*(phi2p*(-x1 + x2) - 2*y1p + 2*y2p))*cos(phi2) + (phi1p - 2*phi2p)*(((phi1p - 2*phi2p)*(x1 - x2) + 2*(y1p - y2p))*cos(phi1) + (-2*x1p + 2*x2p + (phi1p - 2*phi2p)*(y1 - y2))*sin(phi1)) + phi1p*cos(2*phi2)*((phi1p*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi1) + (-2*x1p + 2*x2p - phi1p*y1 + phi1p*y2)*sin(phi1)) + 2*(2*Power(phi1p,2)*(-y1 + y2) + Power(phi2p,2)*(-y1 + y2) + 2*phi2p*(x1p - x2p + 2*phi1p*y1 - 2*phi1p*y2))*sin(phi2) + phi1p*((2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - 2*y1p + 2*y2p)*sin(phi1))*sin(2*phi2)))/16.;
    //dhLqJp(4,3) = 0;
    //dhLqJp(4,4) = 0;
    dhLqJp(4,5) = -(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2)))/(4.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(4,6) = -(phi1p*phi2p*cos(phi1)*(2 + cos(phi1 - phi2)) + phi1p*phi2p*cos(phi2) + sin(phi1 - phi2)*((phi1p - phi2p)*phi2p*sin(phi1) + Power(phi1p,2)*sin(phi2)))/(2.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(4,7) = (Power(Sec((phi1 - phi2)/2.),4)*(2*(Power(phi1p,2)*(x1 - x2) + 2*Power(phi2p,2)*(x1 - x2) - 2*phi1p*(2*phi2p*x1 - 2*phi2p*x2 - y1p + y2p))*cos(phi1) + phi2p*(-(phi2p*x1) + phi2p*x2 + 2*y1p - 2*y2p)*cos(2*phi1 - phi2) + 2*(Power(phi1p,2)*(y1 - y2) + 2*Power(phi2p,2)*(y1 - y2) - 2*phi1p*(x1p - x2p + 2*phi2p*y1 - 2*phi2p*y2))*sin(phi1) - phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(phi2)*sin(2*phi1) + phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1)*sin(phi2) + (2*phi1p - phi2p)*((-2*phi1p*x1 + phi2p*x1 + 2*phi1p*x2 - phi2p*x2 + 2*y1p - 2*y2p)*cos(phi2) + (-2*x1p + 2*x2p - 2*phi1p*y1 + phi2p*y1 + 2*phi1p*y2 - phi2p*y2)*sin(phi2))))/16.;
    dhLqJp(5,0) = (11*(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2))))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(5,1) = (-11*(phi1p*(phi1p - 4*phi2p)*cos(phi1) - phi1p*(phi1p*cos(phi1 - 2*phi2) + 4*phi2p*cos(phi2)) + 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2)))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(5,2) = (-11*Power(Sec((phi1 - phi2)/2.),4)*(2*(-2*Power(phi1p,2)*(x1 - x2) + 4*phi1p*phi2p*(x1 - x2) + phi2p*(phi2p*(-x1 + x2) - 2*y1p + 2*y2p))*cos(phi2) + (phi1p - 2*phi2p)*(((phi1p - 2*phi2p)*(x1 - x2) + 2*(y1p - y2p))*cos(phi1) + (-2*x1p + 2*x2p + (phi1p - 2*phi2p)*(y1 - y2))*sin(phi1)) + phi1p*cos(2*phi2)*((phi1p*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi1) + (-2*x1p + 2*x2p - phi1p*y1 + phi1p*y2)*sin(phi1)) + 2*(2*Power(phi1p,2)*(-y1 + y2) + Power(phi2p,2)*(-y1 + y2) + 2*phi2p*(x1p - x2p + 2*phi1p*y1 - 2*phi1p*y2))*sin(phi2) + phi1p*((2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - 2*y1p + 2*y2p)*sin(phi1))*sin(2*phi2)))/(32.*l0);
    //dhLqJp(5,3) = 0;
    //dhLqJp(5,4) = 0;
    dhLqJp(5,5) = (-11*(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2))))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(5,6) = (-11*(2*phi1p*phi2p*cos(phi1) + 2*phi1p*phi2p*cos(phi2) + sin(phi1 - phi2)*(-(Power(phi2p,2)*sin(phi1)) + Power(phi1p,2)*sin(phi2))))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(5,7) = (11*Power(Sec((phi1 - phi2)/2.),4)*(2*(Power(phi1p,2)*(x1 - x2) + 2*Power(phi2p,2)*(x1 - x2) - 2*phi1p*(2*phi2p*x1 - 2*phi2p*x2 - y1p + y2p))*cos(phi1) + phi2p*(-(phi2p*x1) + phi2p*x2 + 2*y1p - 2*y2p)*cos(2*phi1 - phi2) + 2*(Power(phi1p,2)*(y1 - y2) + 2*Power(phi2p,2)*(y1 - y2) - 2*phi1p*(x1p - x2p + 2*phi2p*y1 - 2*phi2p*y2))*sin(phi1) - phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(phi2)*sin(2*phi1) + phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1)*sin(phi2) + (2*phi1p - phi2p)*((-2*phi1p*x1 + phi2p*x1 + 2*phi1p*x2 - phi2p*x2 + 2*y1p - 2*y2p)*cos(phi2) + (-2*x1p + 2*x2p - 2*phi1p*y1 + phi2p*y1 + 2*phi1p*y2 - phi2p*y2)*sin(phi2))))/(32.*l0);
    dhLqJp(6,0) = -(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2)))/(4.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(6,1) = -(phi1p*phi2p*cos(phi1)*(2 + cos(phi1 - phi2)) + phi1p*phi2p*cos(phi2) + sin(phi1 - phi2)*((phi1p - phi2p)*phi2p*sin(phi1) + Power(phi1p,2)*sin(phi2)))/(2.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(6,2) = (Power(Sec((phi1 - phi2)/2.),4)*(2*(-2*Power(phi1p,2)*(x1 - x2) + 4*phi1p*phi2p*(x1 - x2) + phi2p*(phi2p*(-x1 + x2) - 2*y1p + 2*y2p))*cos(phi2) + (phi1p - 2*phi2p)*(((phi1p - 2*phi2p)*(x1 - x2) + 2*(y1p - y2p))*cos(phi1) + (-2*x1p + 2*x2p + (phi1p - 2*phi2p)*(y1 - y2))*sin(phi1)) + phi1p*cos(2*phi2)*((phi1p*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi1) + (-2*x1p + 2*x2p - phi1p*y1 + phi1p*y2)*sin(phi1)) + 2*(2*Power(phi1p,2)*(-y1 + y2) + Power(phi2p,2)*(-y1 + y2) + 2*phi2p*(x1p - x2p + 2*phi1p*y1 - 2*phi1p*y2))*sin(phi2) + phi1p*((2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - 2*y1p + 2*y2p)*sin(phi1))*sin(2*phi2)))/16.;
    //dhLqJp(6,3) = 0;
    //dhLqJp(6,4) = 0;
    dhLqJp(6,5) = (phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2)))/(4.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(6,6) = (-(phi1p*(phi1p - 4*phi2p)*cos(phi1)) + Power(phi1p,2)*cos(phi1 - 2*phi2) + 4*phi1p*phi2p*cos(phi2) - 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2))/(4.*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(6,7) = (Power(Sec((phi1 - phi2)/2.),4)*((-2*Power(phi1p,2)*(x1 - x2) + 4*Power(phi2p,2)*(-x1 + x2) + 4*phi1p*(2*phi2p*x1 - 2*phi2p*x2 - y1p + y2p))*cos(phi1) + phi2p*(phi2p*(x1 - x2) - 2*y1p + 2*y2p)*cos(2*phi1 - phi2) + 2*(Power(phi1p,2)*(-y1 + y2) + 2*Power(phi2p,2)*(-y1 + y2) + 2*phi1p*(x1p - x2p + 2*phi2p*y1 - 2*phi2p*y2))*sin(phi1) + phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(phi2)*sin(2*phi1) - phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1)*sin(phi2) + (2*phi1p - phi2p)*(((2*phi1p - phi2p)*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi2) + (2*x1p - 2*x2p + (2*phi1p - phi2p)*(y1 - y2))*sin(phi2))))/16.;
    dhLqJp(7,0) = (-11*(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2))))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(7,1) = (11*(phi1p*(phi1p - 4*phi2p)*cos(phi1) - phi1p*(phi1p*cos(phi1 - 2*phi2) + 4*phi2p*cos(phi2)) + 2*Power(phi2p,2)*sin(phi1)*sin(phi1 - phi2)))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(7,2) = (11*Power(Sec((phi1 - phi2)/2.),4)*(2*(-2*Power(phi1p,2)*(x1 - x2) + 4*phi1p*phi2p*(x1 - x2) + phi2p*(phi2p*(-x1 + x2) - 2*y1p + 2*y2p))*cos(phi2) + (phi1p - 2*phi2p)*(((phi1p - 2*phi2p)*(x1 - x2) + 2*(y1p - y2p))*cos(phi1) + (-2*x1p + 2*x2p + (phi1p - 2*phi2p)*(y1 - y2))*sin(phi1)) + phi1p*cos(2*phi2)*((phi1p*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi1) + (-2*x1p + 2*x2p - phi1p*y1 + phi1p*y2)*sin(phi1)) + 2*(2*Power(phi1p,2)*(-y1 + y2) + Power(phi2p,2)*(-y1 + y2) + 2*phi2p*(x1p - x2p + 2*phi1p*y1 - 2*phi1p*y2))*sin(phi2) + phi1p*((2*x1p - 2*x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (phi1p*(x1 - x2) - 2*y1p + 2*y2p)*sin(phi1))*sin(2*phi2)))/(32.*l0);
    //dhLqJp(7,3) = 0;
    //dhLqJp(7,4) = 0;
    dhLqJp(7,5) = (11*(phi1p*(phi1p - 4*phi2p)*sin(phi1) + Power(phi1p,2)*sin(phi1 - 2*phi2) - 2*phi2p*(phi2p*cos(phi1)*sin(phi1 - phi2) + 2*phi1p*sin(phi2))))/(8.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(7,6) = (11*(2*phi1p*phi2p*cos(phi1) + 2*phi1p*phi2p*cos(phi2) + sin(phi1 - phi2)*(-(Power(phi2p,2)*sin(phi1)) + Power(phi1p,2)*sin(phi2))))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqJp(7,7) = (11*Power(Sec((phi1 - phi2)/2.),4)*((-2*Power(phi1p,2)*(x1 - x2) + 4*Power(phi2p,2)*(-x1 + x2) + 4*phi1p*(2*phi2p*x1 - 2*phi2p*x2 - y1p + y2p))*cos(phi1) + phi2p*(phi2p*(x1 - x2) - 2*y1p + 2*y2p)*cos(2*phi1 - phi2) + 2*(Power(phi1p,2)*(-y1 + y2) + 2*Power(phi2p,2)*(-y1 + y2) + 2*phi1p*(x1p - x2p + 2*phi2p*y1 - 2*phi2p*y2))*sin(phi1) + phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(phi2)*sin(2*phi1) - phi2p*(2*x1p - 2*x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1)*sin(phi2) + (2*phi1p - phi2p)*(((2*phi1p - phi2p)*(x1 - x2) - 2*y1p + 2*y2p)*cos(phi2) + (2*x1p - 2*x2p + (2*phi1p - phi2p)*(y1 - y2))*sin(phi2))))/(32.*l0);


    //Abl. h nach qpLokal
    //dhLqp(0,0) = 0;
    //dhLqp(0,1) = 0;
    dhLqp(0,2) = (Arho*l0*((-104*aLp - 8*aRp + 3*l0*(bRp + 5*bLp*(3 + 4*eps) - 20*(1 + eps)*phiSp))*cos(bL - phiS) + (-8*aLp - 104*aRp + 3*l0*(bLp + 5*bRp*(3 + 4*eps) + 20*(1 + eps)*phiSp))*cos(bR + phiS) + (3*l0*(20*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*sin(bL - phiS) + (8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-20*epsp + (bL - 5*bR)*(bRp + phiSp)))*sin(bR + phiS)))/240.;
    dhLqp(0,3) = (Arho*l0h2*((-bLp + phiSp)*sin(bL - phiS) + (bRp + phiSp)*sin(bR + phiS)))/4.;
    dhLqp(0,4) = (Arho*l0*(13*(bLp - phiSp)*cos(bL - phiS) - (bRp + phiSp)*cos(bR + phiS)))/30.;
    dhLqp(0,5) = (Arho*l0*((104*aLp + 8*aRp - 3*l0*(10*bLp + bRp + 20*bLp*eps - 5*(3 + 4*eps)*phiSp))*cos(bL - phiS) + 3*l0*(bRp + phiSp)*cos(bR + phiS) + (-8*aR*bLp - 3*l0*(20*epsp + (5*bL - bR)*(bLp - phiSp)) - 104*aL*(bLp - phiSp) + 8*aR*phiSp)*sin(bL - phiS)))/240.;
    dhLqp(0,6) = (Arho*l0*((bLp - phiSp)*cos(bL - phiS) - 13*(bRp + phiSp)*cos(bR + phiS)))/30.;
    dhLqp(0,7) = (Arho*l0*(3*l0*(-bLp + phiSp)*cos(bL - phiS) + (-8*aLp - 104*aRp + 3*l0*(bLp + 10*bRp + 20*bRp*eps + 5*(3 + 4*eps)*phiSp))*cos(bR + phiS) + (8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-20*epsp + (bL - 5*bR)*(bRp + phiSp)))*sin(bR + phiS)))/240.;
    //dhLqp(1,0) = 0;
    //dhLqp(1,1) = 0;
    dhLqp(1,2) = (Arho*l0*((3*l0*(20*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*cos(bL - phiS) - (8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-20*epsp + (bL - 5*bR)*(bRp + phiSp)))*cos(bR + phiS) + (104*aLp + 8*aRp - 3*(bRp + 5*bLp*(3 + 4*eps))*l0 + 60*(1 + eps)*l0*phiSp)*sin(bL - phiS) + (-8*aLp - 104*aRp + 3*l0*(bLp + 5*bRp*(3 + 4*eps) + 20*(1 + eps)*phiSp))*sin(bR + phiS)))/240.;
    dhLqp(1,3) = -(Arho*l0h2*((bLp - phiSp)*cos(bL - phiS) + (bRp + phiSp)*cos(bR + phiS)))/4.;
    dhLqp(1,4) = -(Arho*l0*(13*(bLp - phiSp)*sin(bL - phiS) + (bRp + phiSp)*sin(bR + phiS)))/30.;
    dhLqp(1,5) = -(Arho*l0*((3*l0*(20*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*cos(bL - phiS) + (104*aLp + 8*aRp - 3*l0*(10*bLp + bRp + 20*bLp*eps - 5*(3 + 4*eps)*phiSp))*sin(bL - phiS) - 3*l0*(bRp + phiSp)*sin(bR + phiS)))/240.;
    dhLqp(1,6) = -(Arho*l0*((bLp - phiSp)*sin(bL - phiS) + 13*(bRp + phiSp)*sin(bR + phiS)))/30.;
    dhLqp(1,7) = (Arho*l0*(-((8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-20*epsp + (bL - 5*bR)*(bRp + phiSp)))*cos(bR + phiS)) + 3*l0*(bLp - phiSp)*sin(bL - phiS) + (-8*aLp - 104*aRp + 3*l0*(bLp + 10*bRp + 20*bRp*eps + 5*(3 + 4*eps)*phiSp))*sin(bR + phiS)))/240.;
    //dhLqp(2,0) = 0;
    //dhLqp(2,1) = 0;
    dhLqp(2,2) = (Arho*l0*(-16*(197*aL*aLp + 17*aLp*aR + 17*aL*aRp + 197*aR*aRp) - (199*aLp*bL - 23*aRp*bL + 199*aL*bLp - 23*aR*bLp - 23*aLp*bR + 199*aRp*bR - 23*aL*bRp + 199*aR*bRp)*l0 - (55*bL*bLp - 21*bLp*bR - 21*bL*bRp + 55*bR*bRp + 1680*(1 + eps)*epsp)*l0h2))/10080.;
    dhLqp(2,3) = (Arho*(1 + eps)*l0h3*(bLp - bRp - 2*phiSp))/12.;
    dhLqp(2,4) = (Arho*l0*(16*aL*(196*bLp - bRp - 197*phiSp) + 136*aR*(bLp - bRp - 2*phiSp) + l0*(205*bL*bLp - 51*bLp*bR + 6*bL*bRp - 28*bR*bRp - 199*bL*phiSp + 23*bR*phiSp)))/10080.;
    dhLqp(2,5) = (Arho*l0*(64*aR*aRp + 112*aRp*bL*l0 + 224*aR*bLp*l0 - 24*aRp*bR*l0 + 180*aR*bRp*l0 + 422*bL*bLp*l0h2 - 84*bLp*bR*l0h2 - 51*bL*bRp*l0h2 + 51*bR*bRp*l0h2 + 3360*epsp*l0h2 + 3360*eps*epsp*l0h2 + 4*aLp*(136*aR + 205*bL*l0 - 51*bR*l0) + 4*l0*(23*aR - 55*bL*l0 + 21*bR*l0)*phiSp + 4*aL*(3136*aLp + 136*aRp + 410*bLp*l0 - 45*bRp*l0 - 199*l0*phiSp)))/40320.;
    dhLqp(2,6) = (Arho*l0*(16*aR*(bLp - 196*bRp - 197*phiSp) + 136*aL*(bLp - bRp - 2*phiSp) + l0*(bL*(28*bLp + 51*bRp + 23*phiSp) - bR*(6*bLp + 205*bRp + 199*phiSp))))/10080.;
    dhLqp(2,7) = (Arho*l0*(-32*(2*aL*aLp + 17*aLp*aR + 17*aL*aRp + 392*aR*aRp) + 4*(6*aLp*bL + 51*aRp*bL - 45*aL*bLp + 45*aR*bLp - 28*aLp*bR - 205*aRp*bR - 56*aL*bRp - 410*aR*bRp)*l0 - (51*bL*bLp - 51*bLp*bR - 84*bL*bRp + 422*bR*bRp + 3360*(1 + eps)*epsp)*l0h2 + 4*l0*(23*aL - 199*aR + 21*bL*l0 - 55*bR*l0)*phiSp))/40320.;
    //dhLqp(3,0) = 0;
    //dhLqp(3,1) = 0;
    dhLqp(3,2) = (Arho*l0h2*(9*aLp - 9*aRp - (bLp - bRp)*(4 + 5*eps)*l0 + 10*(1 + eps)*l0*phiSp))/60.;
    //dhLqp(3,3) = 0;
    dhLqp(3,4) = -(Arho*l0h2*(19*bLp + bRp - 18*phiSp))/120.;
    dhLqp(3,5) = (Arho*l0h2*(-76*aLp - 4*aRp + l0*(27*bLp + 3*bRp + 40*bLp*eps - 8*(4 + 5*eps)*phiSp)))/480.;
    dhLqp(3,6) = -(Arho*l0h2*(bLp + 19*bRp + 18*phiSp))/120.;
    dhLqp(3,7) = (Arho*l0h2*(-4*aLp - 76*aRp + l0*(3*bLp + 27*bRp + 40*bRp*eps + 8*(4 + 5*eps)*phiSp)))/480.;
    //dhLqp(4,0) = 0;
    //dhLqp(4,1) = 0;
    dhLqp(4,2) = (Arho*l0*(-136*aR*(bLp - bRp - 2*phiSp) + 16*aL*(-196*bLp + bRp + 197*phiSp) + l0*(-205*bL*bLp + 51*bLp*bR - 6*bL*bRp + 28*bR*bRp - 1512*epsp + 199*bL*phiSp - 23*bR*phiSp)))/10080.;
    dhLqp(4,3) = (Arho*l0h2*(19*bLp + bRp - 18*phiSp))/120.;
    //dhLqp(4,4) = 0;
    dhLqp(4,5) = (Arho*l0*(l0*(1596*epsp + (205*bL - 51*bR)*(bLp - phiSp)) + 3136*aL*(bLp - phiSp) + 136*aR*(bLp - phiSp)))/10080.;
    //dhLqp(4,6) = 0;
    dhLqp(4,7) = (Arho*l0*(8*aL*(bRp + phiSp) + 68*aR*(bRp + phiSp) + l0*(42*epsp - (3*bL - 14*bR)*(bRp + phiSp))))/5040.;
    //dhLqp(5,0) = 0;
    //dhLqp(5,1) = 0;
    dhLqp(5,2) = (Arho*l0*(64*aR*aRp + 112*aRp*bL*l0 - 24*aRp*bR*l0 - 228*aR*bRp*l0 - 33*bL*bRp*l0h2 - 33*bR*bRp*l0h2 + 2688*epsp*l0h2 + 3360*eps*epsp*l0h2 + 4*aLp*(136*aR + 205*bL*l0 - 51*bR*l0) + 4*l0*(-23*aR + 55*bL*l0 - 21*bR*l0)*phiSp + 4*aL*(3136*aLp + 136*aRp - 57*bRp*l0 + 199*l0*phiSp)))/40320.;
    dhLqp(5,3) = -(Arho*l0h3*(3*bRp + bLp*(67 + 80*eps) - 16*(4 + 5*eps)*phiSp))/960.;
    dhLqp(5,4) = -(Arho*l0*(3136*aL + 136*aR + 205*bL*l0 - 51*bR*l0)*(bLp - phiSp))/10080.;
    dhLqp(5,5) = (Arho*l0*(-32*(392*aL*aLp + 17*aLp*aR + 17*aL*aRp + 2*aR*aRp) - 4*(205*aLp*bL + 28*aRp*bL + 205*aL*bLp + 28*aR*bLp - 51*aLp*bR - 6*aRp*bR - 51*aL*bRp - 6*aR*bRp)*l0 - (211*bL*bLp - 42*bLp*bR - 42*bL*bRp + 9*bR*bRp + 42*(67 + 80*eps)*epsp)*l0h2))/40320.;
    dhLqp(5,6) = -(Arho*l0*(68*aL + 8*aR + 14*bL*l0 - 3*bR*l0)*(bLp - phiSp))/5040.;
    dhLqp(5,7) = -(Arho*l0h2*(aL*(-68*bLp + 8*bRp + 76*phiSp) + aR*(-8*bLp + 68*bRp + 76*phiSp) + l0*(-14*bL*bLp + 3*bLp*bR - 3*bL*bRp + 14*bR*bRp + 42*epsp + 11*(bL + bR)*phiSp)))/13440.;
    //dhLqp(6,0) = 0;
    //dhLqp(6,1) = 0;
    dhLqp(6,2) = (Arho*l0*(-16*aR*(bLp - 196*bRp - 197*phiSp) - 136*aL*(bLp - bRp - 2*phiSp) + l0*(6*bLp*bR + 205*bR*bRp + 1512*epsp + 199*bR*phiSp - bL*(28*bLp + 51*bRp + 23*phiSp))))/10080.;
    dhLqp(6,3) = (Arho*l0h2*(bLp + 19*bRp + 18*phiSp))/120.;
    //dhLqp(6,4) = 0;
    dhLqp(6,5) = (Arho*l0*(l0*(42*epsp + (14*bL - 3*bR)*(bLp - phiSp)) + 68*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp)))/5040.;
    //dhLqp(6,6) = 0;
    dhLqp(6,7) = (Arho*l0*(136*aL*(bRp + phiSp) + 3136*aR*(bRp + phiSp) + l0*(1596*epsp - 51*bL*(bRp + phiSp) + 205*bR*(bRp + phiSp))))/10080.;
    //dhLqp(7,0) = 0;
    //dhLqp(7,1) = 0;
    dhLqp(7,2) = (Arho*l0*(-12544*aR*aRp + 204*aRp*bL*l0 + 228*aR*bLp*l0 - 820*aRp*bR*l0 + 33*bL*bLp*l0h2 + 33*bLp*bR*l0h2 - 2688*epsp*l0h2 - 3360*eps*epsp*l0h2 - 8*aLp*(68*aR - 3*bL*l0 + 14*bR*l0) + 4*l0*(199*aR - 21*bL*l0 + 55*bR*l0)*phiSp - 4*aL*(16*aLp + 136*aRp - 57*bLp*l0 + 23*l0*phiSp)))/40320.;
    dhLqp(7,3) = -(Arho*l0h3*(3*bLp + bRp*(67 + 80*eps) + 16*(4 + 5*eps)*phiSp))/960.;
    dhLqp(7,4) = (Arho*l0*(-8*aL - 68*aR + 3*bL*l0 - 14*bR*l0)*(bRp + phiSp))/5040.;
    dhLqp(7,5) = (Arho*l0h2*(aL*(-68*bLp + 8*bRp + 76*phiSp) + aR*(-8*bLp + 68*bRp + 76*phiSp) + l0*(-14*bL*bLp + 3*bLp*bR - 3*bL*bRp + 14*bR*bRp - 42*epsp + 11*(bL + bR)*phiSp)))/13440.;
    dhLqp(7,6) = (Arho*l0*(-136*aL - 3136*aR + 51*bL*l0 - 205*bR*l0)*(bRp + phiSp))/10080.;
    dhLqp(7,7) = (Arho*l0*(-32*(2*aL*aLp + 17*aLp*aR + 17*aL*aRp + 392*aR*aRp) + 4*(6*aLp*bL + 51*aRp*bL + 6*aL*bLp + 51*aR*bLp - 28*aLp*bR - 205*aRp*bR - 28*aL*bRp - 205*aR*bRp)*l0 - (9*bL*bLp - 42*bLp*bR - 42*bL*bRp + 211*bR*bRp + 42*(67 + 80*eps)*epsp)*l0h2))/40320.;

    //dhLqpJp(0,0) = 0;
    dhLqpJp(0,1) = (phi1p - phi2p)/one_p_cos_dphi;
    dhLqpJp(0,2) = ((Power(Sec((phi1 - phi2)/2.),3)*(5*l0*(phi1p - 2*phi2p)*cos((phi1 - 3*phi2)/2.) + 10*l0*phi2p*cos((3*phi1 - phi2)/2.) - 64*(a1 + a2)*phi2p*sin((phi1 - 3*phi2)/2.) + 5*l0*(phi1p*cos((phi1 + phi2)/2.) + (phi1 - phi2)*phi2p*sin((phi1 - 3*phi2)/2.)) + 3*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*phi2p*sin((phi1 + phi2)/2.)))/2. + Power(Sec((phi1 - phi2)/2.),2)*(72*(y1p - y2p) + (-128*(a1p + a2p) + 15*l0*phi1p)*cos(phi2) + (72*(phi1p - phi2p)*(y1 - y2) - (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*(2*phi1p - phi2p)*cos(phi2))*tan((phi1 - phi2)/2.)))/144.;
    dhLqpJp(0,3) = (-16*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(9.*one_p_cos_dphi);
    dhLqpJp(0,4) = (-16*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(9.*one_p_cos_dphi);
    //dhLqpJp(0,5) = 0;
    dhLqpJp(0,6) = (-phi1p + phi2p)/one_p_cos_dphi;
    dhLqpJp(0,7) = (-4*Power(cos((phi1 - phi2)/2.),2)*(36*(y1p - y2p) + (64*a1p + 64*a2p - 5*l0*phi1p + 10*l0*phi2p)*cos(phi1) + 5*l0*phi1p*cos(phi2)) + 2*phi1p*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*sin(phi1) - 72*(phi1p - phi2p)*(y1 - y2)*sin(phi1 - phi2) + (64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*phi2p*sin(2*phi1 - phi2) + (64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*(2*phi1p - phi2p)*sin(phi2))/(72.*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(1,0) = (-phi1p + phi2p)/one_p_cos_dphi;
    //dhLqpJp(1,1) = 0;
    dhLqpJp(1,2) = -(-((64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*(phi1p - 2*phi2p)*cos(phi1)) + phi1p*(64*a1 + 64*a2 + 5*l0*(-phi1 + phi2))*cos(phi1 - 2*phi2) + 128*a1*phi2p*cos(phi2) + 128*a2*phi2p*cos(phi2) - 10*l0*phi1*phi2p*cos(phi2) + 10*l0*phi2*phi2p*cos(phi2) - 10*l0*phi1p*sin(phi1) - 5*l0*phi2p*sin(phi1) + 10*l0*phi1p*sin(phi1 - 2*phi2) - 5*l0*phi2p*sin(phi1 - 2*phi2) + 72*phi1p*x1*sin(phi1 - phi2) - 72*phi2p*x1*sin(phi1 - phi2) - 72*phi1p*x2*sin(phi1 - phi2) + 72*phi2p*x2*sin(phi1 - phi2) - 5*l0*phi2p*sin(2*phi1 - phi2) + 5*l0*(-4*phi1p + phi2p)*sin(phi2) + 16*Power(cos((phi1 - phi2)/2.),2)*(9*(x1p - x2p) + 16*(a1p + a2p)*sin(phi2)))/(72.*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(1,3) = (-16*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(9.*one_p_cos_dphi);
    dhLqpJp(1,4) = (-16*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(9.*one_p_cos_dphi);
    dhLqpJp(1,5) = (phi1p - phi2p)/one_p_cos_dphi;
    //dhLqpJp(1,6) = 0;
    dhLqpJp(1,7) = -(72*(-x1p + x2p) + 2*phi1p*(64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*cos(phi1) + (64*a1 + 64*a2 - 5*l0*phi1 + 5*l0*phi2)*phi2p*cos(2*phi1 - phi2) + 128*a1*phi1p*cos(phi2) + 128*a2*phi1p*cos(phi2) - 10*l0*phi1*phi1p*cos(phi2) + 10*l0*phi1p*phi2*cos(phi2) - 64*a1*phi2p*cos(phi2) - 64*a2*phi2p*cos(phi2) + 5*l0*phi1*phi2p*cos(phi2) - 5*l0*phi2*phi2p*cos(phi2) + 128*(a1p + a2p)*sin(phi1) - 5*l0*phi1p*sin(phi1) + 20*l0*phi2p*sin(phi1) + 8*cos(phi1 - phi2)*(-9*x1p + 9*x2p + 16*(a1p + a2p)*sin(phi1)) - 5*l0*phi1p*sin(phi1 - 2*phi2) - 72*phi1p*x1*sin(phi1 - phi2) + 72*phi2p*x1*sin(phi1 - phi2) + 72*phi1p*x2*sin(phi1 - phi2) - 72*phi2p*x2*sin(phi1 - phi2) - 5*l0*phi1p*sin(2*phi1 - phi2) + 10*l0*phi2p*sin(2*phi1 - phi2) + 5*l0*(phi1p + 2*phi2p)*sin(phi2))/(72.*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(2,0) = (11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(2,1) = (11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(2,2) = (11*((x1p - x2p - (phi1p - 2*phi2p)*(y1 - y2))*cos(phi1) + (x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1 - 2*phi2) + 2*x1p*cos(phi2) - 2*x2p*cos(phi2) + 2*phi2p*y1*cos(phi2) - 2*phi2p*y2*cos(phi2) + phi1p*x1*sin(phi1) - 2*phi2p*x1*sin(phi1) - phi1p*x2*sin(phi1) + 2*phi2p*x2*sin(phi1) + y1p*sin(phi1) - y2p*sin(phi1) + phi1p*x1*sin(phi1 - 2*phi2) - phi1p*x2*sin(phi1 - 2*phi2) - y1p*sin(phi1 - 2*phi2) + y2p*sin(phi1 - 2*phi2) + 2*(-(phi2p*x1) + phi2p*x2 + y1p - y2p)*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    //dhLqpJp(2,3) = 0;
    //dhLqpJp(2,4) = 0;
    dhLqpJp(2,5) = (-11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(2,6) = (-11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(2,7) = (11*(2*(x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (x1p - x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1 - phi2) + x1p*cos(phi2) - x2p*cos(phi2) + 2*phi1p*y1*cos(phi2) - phi2p*y1*cos(phi2) - 2*phi1p*y2*cos(phi2) + phi2p*y2*cos(phi2) + 2*(-(phi1p*x1) + phi1p*x2 + y1p - y2p)*sin(phi1) - (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(phi2)*sin(2*phi1) - 2*phi1p*x1*sin(phi2) + phi2p*x1*sin(phi2) + 2*phi1p*x2*sin(phi2) - phi2p*x2*sin(phi2) + y1p*sin(phi2) - y2p*sin(phi2) + (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(2*phi1)*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(3,0) = (2*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(l0*one_p_cos_dphi);
    dhLqpJp(3,1) = (-2*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(l0*one_p_cos_dphi);
    dhLqpJp(3,2) = (-18*((phi1p - 2*phi2p)*(x1 - x2) + y1p - y2p)*cos(phi1) + 18*(phi1p*(x1 - x2) - y1p + y2p)*cos(phi1 - 2*phi2) - 10*l0*phi1p*cos(phi1 - phi2) + 10*l0*phi2p*cos(phi1 - phi2) + 36*phi2p*x1*cos(phi2) - 36*phi2p*x2*cos(phi2) - 36*y1p*cos(phi2) + 36*y2p*cos(phi2) - 18*phi1p*y1*sin(phi1) + 36*phi2p*y1*sin(phi1) + 18*phi1p*y2*sin(phi1) - 36*phi2p*y2*sin(phi1) - 18*phi1p*y1*sin(phi1 - 2*phi2) + 18*phi1p*y2*sin(phi1 - 2*phi2) + 64*a1*phi1p*sin(phi1 - phi2) + 64*a2*phi1p*sin(phi1 - phi2) - 5*l0*phi1*phi1p*sin(phi1 - phi2) + 5*l0*phi1p*phi2*sin(phi1 - phi2) - 64*a1*phi2p*sin(phi1 - phi2) - 64*a2*phi2p*sin(phi1 - phi2) + 5*l0*phi1*phi2p*sin(phi1 - phi2) - 5*l0*phi2*phi2p*sin(phi1 - phi2) + 36*phi2p*(y1 - y2)*sin(phi2) + 2*(32*a1p + 32*a2p - 5*l0*phi1p + 5*l0*phi2p + 32*(a1p + a2p)*cos(phi1 - phi2) + 9*(x1p - x2p)*(sin(phi1) - sin(phi1 - 2*phi2) + 2*sin(phi2))))/(18.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(3,3) = (32*(phi1p - phi2p))/(9.*l0*one_p_cos_dphi);
    dhLqpJp(3,4) = (32*(phi1p - phi2p))/(9.*l0*one_p_cos_dphi);
    dhLqpJp(3,5) = (-2*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(l0*one_p_cos_dphi);
    dhLqpJp(3,6) = (2*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(l0*one_p_cos_dphi);
    dhLqpJp(3,7) = (36*(phi1p*(x1 - x2) - y1p + y2p)*cos(phi1) - 2*(32*a1p + 32*a2p + 5*l0*(-phi1p + phi2p))*cos(phi1 - phi2) + 18*phi2p*x1*cos(2*phi1 - phi2) - 18*phi2p*x2*cos(2*phi1 - phi2) - 18*y1p*cos(2*phi1 - phi2) + 18*y2p*cos(2*phi1 - phi2) + 36*phi1p*x1*cos(phi2) - 18*phi2p*x1*cos(phi2) - 36*phi1p*x2*cos(phi2) + 18*phi2p*x2*cos(phi2) - 18*y1p*cos(phi2) + 18*y2p*cos(phi2) + 36*phi1p*y1*sin(phi1) - 36*phi1p*y2*sin(phi1) + 2*(-32*a1p - 32*a2p + 5*l0*phi1p - 5*l0*phi2p + 18*(x1p - x2p)*one_p_cos_dphi*sin(phi1)) - 64*a1*phi1p*sin(phi1 - phi2) - 64*a2*phi1p*sin(phi1 - phi2) + 5*l0*phi1*phi1p*sin(phi1 - phi2) - 5*l0*phi1p*phi2*sin(phi1 - phi2) + 64*a1*phi2p*sin(phi1 - phi2) + 64*a2*phi2p*sin(phi1 - phi2) - 5*l0*phi1*phi2p*sin(phi1 - phi2) + 5*l0*phi2*phi2p*sin(phi1 - phi2) + 18*phi2p*y1*sin(2*phi1 - phi2) - 18*phi2p*y2*sin(2*phi1 - phi2) - 18*(-2*phi1p + phi2p)*(y1 - y2)*sin(phi2))/(18.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(4,0) = (phi2p*cos(phi1) + phi1p*cos(phi2))/one_p_cos_dphi;
    dhLqpJp(4,1) = (phi2p*sin(phi1) + phi1p*sin(phi2))/one_p_cos_dphi;
    dhLqpJp(4,2) = ((x1p - x2p - (phi1p - 2*phi2p)*(y1 - y2))*cos(phi1) + (x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1 - 2*phi2) + 2*x1p*cos(phi2) - 2*x2p*cos(phi2) + 2*phi2p*y1*cos(phi2) - 2*phi2p*y2*cos(phi2) + phi1p*x1*sin(phi1) - 2*phi2p*x1*sin(phi1) - phi1p*x2*sin(phi1) + 2*phi2p*x2*sin(phi1) + y1p*sin(phi1) - y2p*sin(phi1) + phi1p*x1*sin(phi1 - 2*phi2) - phi1p*x2*sin(phi1 - 2*phi2) - y1p*sin(phi1 - 2*phi2) + y2p*sin(phi1 - 2*phi2) + 2*(-(phi2p*x1) + phi2p*x2 + y1p - y2p)*sin(phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    //dhLqpJp(4,3) = 0;
    //dhLqpJp(4,4) = 0;
    dhLqpJp(4,5) = -((phi2p*cos(phi1) + phi1p*cos(phi2))/one_p_cos_dphi);
    dhLqpJp(4,6) = -((phi2p*sin(phi1) + phi1p*sin(phi2))/one_p_cos_dphi);
    dhLqpJp(4,7) = (2*(x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (x1p - x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1 - phi2) + x1p*cos(phi2) - x2p*cos(phi2) + 2*phi1p*y1*cos(phi2) - phi2p*y1*cos(phi2) - 2*phi1p*y2*cos(phi2) + phi2p*y2*cos(phi2) + 2*(-(phi1p*x1) + phi1p*x2 + y1p - y2p)*sin(phi1) - (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(phi2)*sin(2*phi1) - 2*phi1p*x1*sin(phi2) + phi2p*x1*sin(phi2) + 2*phi1p*x2*sin(phi2) - phi2p*x2*sin(phi2) + y1p*sin(phi2) - y2p*sin(phi2) + (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(2*phi1)*sin(phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(5,0) = (11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(5,1) = (11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(5,2) = (11*((x1p - x2p - (phi1p - 2*phi2p)*(y1 - y2))*cos(phi1) + (x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1 - 2*phi2) + 2*x1p*cos(phi2) - 2*x2p*cos(phi2) + 2*phi2p*y1*cos(phi2) - 2*phi2p*y2*cos(phi2) + phi1p*x1*sin(phi1) - 2*phi2p*x1*sin(phi1) - phi1p*x2*sin(phi1) + 2*phi2p*x2*sin(phi1) + y1p*sin(phi1) - y2p*sin(phi1) + phi1p*x1*sin(phi1 - 2*phi2) - phi1p*x2*sin(phi1 - 2*phi2) - y1p*sin(phi1 - 2*phi2) + y2p*sin(phi1 - 2*phi2) + 2*(-(phi2p*x1) + phi2p*x2 + y1p - y2p)*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    //dhLqpJp(5,3) = 0;
    //dhLqpJp(5,4) = 0;
    dhLqpJp(5,5) = (-11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(5,6) = (-11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(5,7) = (11*(2*(x1p - x2p + phi1p*y1 - phi1p*y2)*cos(phi1) + (x1p - x2p + phi2p*y1 - phi2p*y2)*cos(2*phi1 - phi2) + x1p*cos(phi2) - x2p*cos(phi2) + 2*phi1p*y1*cos(phi2) - phi2p*y1*cos(phi2) - 2*phi1p*y2*cos(phi2) + phi2p*y2*cos(phi2) + 2*(-(phi1p*x1) + phi1p*x2 + y1p - y2p)*sin(phi1) - (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(phi2)*sin(2*phi1) - 2*phi1p*x1*sin(phi2) + phi2p*x1*sin(phi2) + 2*phi1p*x2*sin(phi2) - phi2p*x2*sin(phi2) + y1p*sin(phi2) - y2p*sin(phi2) + (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(2*phi1)*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(6,0) = -((phi2p*cos(phi1) + phi1p*cos(phi2))/one_p_cos_dphi);
    dhLqpJp(6,1) = -((phi2p*sin(phi1) + phi1p*sin(phi2))/one_p_cos_dphi);
    dhLqpJp(6,2) = ((-x1p + x2p + (phi1p - 2*phi2p)*(y1 - y2))*cos(phi1) + (-x1p + x2p - phi1p*y1 + phi1p*y2)*cos(phi1 - 2*phi2) - 2*x1p*cos(phi2) + 2*x2p*cos(phi2) - 2*phi2p*y1*cos(phi2) + 2*phi2p*y2*cos(phi2) - phi1p*x1*sin(phi1) + 2*phi2p*x1*sin(phi1) + phi1p*x2*sin(phi1) - 2*phi2p*x2*sin(phi1) - y1p*sin(phi1) + y2p*sin(phi1) - phi1p*x1*sin(phi1 - 2*phi2) + phi1p*x2*sin(phi1 - 2*phi2) + y1p*sin(phi1 - 2*phi2) - y2p*sin(phi1 - 2*phi2) + 2*(phi2p*x1 - phi2p*x2 - y1p + y2p)*sin(phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    //dhLqpJp(6,3) = 0;
    //dhLqpJp(6,4) = 0;
    dhLqpJp(6,5) = (phi2p*cos(phi1) + phi1p*cos(phi2))/one_p_cos_dphi;
    dhLqpJp(6,6) = (phi2p*sin(phi1) + phi1p*sin(phi2))/one_p_cos_dphi;
    dhLqpJp(6,7) = (2*(-x1p + x2p - phi1p*y1 + phi1p*y2)*cos(phi1) + (-x1p + x2p - phi2p*y1 + phi2p*y2)*cos(2*phi1 - phi2) - x1p*cos(phi2) + x2p*cos(phi2) - 2*phi1p*y1*cos(phi2) + phi2p*y1*cos(phi2) + 2*phi1p*y2*cos(phi2) - phi2p*y2*cos(phi2) + 2*(phi1p*x1 - phi1p*x2 - y1p + y2p)*sin(phi1) + (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(phi2)*sin(2*phi1) + 2*phi1p*x1*sin(phi2) - phi2p*x1*sin(phi2) - 2*phi1p*x2*sin(phi2) + phi2p*x2*sin(phi2) - y1p*sin(phi2) + y2p*sin(phi2) - (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(2*phi1)*sin(phi2))/(2.*Power(1 + cos(phi1 - phi2),2));
    dhLqpJp(7,0) = (-11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(7,1) = (-11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(7,2) = (11*((-x1p + x2p + (phi1p - 2*phi2p)*(y1 - y2))*cos(phi1) + (-x1p + x2p - phi1p*y1 + phi1p*y2)*cos(phi1 - 2*phi2) - 2*x1p*cos(phi2) + 2*x2p*cos(phi2) - 2*phi2p*y1*cos(phi2) + 2*phi2p*y2*cos(phi2) - phi1p*x1*sin(phi1) + 2*phi2p*x1*sin(phi1) + phi1p*x2*sin(phi1) - 2*phi2p*x2*sin(phi1) - y1p*sin(phi1) + y2p*sin(phi1) - phi1p*x1*sin(phi1 - 2*phi2) + phi1p*x2*sin(phi1 - 2*phi2) + y1p*sin(phi1 - 2*phi2) - y2p*sin(phi1 - 2*phi2) + 2*(phi2p*x1 - phi2p*x2 - y1p + y2p)*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));
    //dhLqpJp(7,3) = 0;
    //dhLqpJp(7,4) = 0;
    dhLqpJp(7,5) = (11*(phi2p*cos(phi1) + phi1p*cos(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(7,6) = (11*(phi2p*sin(phi1) + phi1p*sin(phi2)))/(2.*l0*one_p_cos_dphi);
    dhLqpJp(7,7) = (11*(2*(-x1p + x2p - phi1p*y1 + phi1p*y2)*cos(phi1) + (-x1p + x2p - phi2p*y1 + phi2p*y2)*cos(2*phi1 - phi2) - x1p*cos(phi2) + x2p*cos(phi2) - 2*phi1p*y1*cos(phi2) + phi2p*y1*cos(phi2) + 2*phi1p*y2*cos(phi2) - phi2p*y2*cos(phi2) + 2*(phi1p*x1 - phi1p*x2 - y1p + y2p)*sin(phi1) + (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(phi2)*sin(2*phi1) + 2*phi1p*x1*sin(phi2) - phi2p*x1*sin(phi2) - 2*phi1p*x2*sin(phi2) + phi2p*x2*sin(phi2) - y1p*sin(phi2) + y2p*sin(phi2) - (phi2p*x1 - phi2p*x2 - y1p + y2p)*cos(2*phi1)*sin(phi2)))/(4.*l0*Power(1 + cos(phi1 - phi2),2));

    // bis hier: aus der Herleitungsroutine

    Dhz(0,0, 7,7) << dhqJ + trans(Jeg)*( dhLq *Jeg - dhLqM*Jeg - MLokal*dhLqJp  );
    Dhz(8,0,15,7) <<        trans(Jeg)*( dhLqp*Jeg             - MLokal*dhLqpJp );

    return Dhz;
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo

  // Mechanik des Elements mit diesen Koordinaten/Geschwindigkeiten 
  Vec FiniteElement1s21RCM::ElementData(Vec qElement, Vec qpElement)
  {
    // Rueckgabedaten
    Vec Data(8,fmatvec::INIT,0.0);
    // 0:  eps
    // 1:  epsp
    // 2:  xS
    // 3:  yS
    // 4:  xSp
    // 5:  ySp
    // 6:  delta_phi      = bL  + bR
    // 7:  delta_phip     = bLp + bRp

    Vec qLokal(8,fmatvec::INIT,0.0), qpLokal(8,fmatvec::INIT,0.0);
    SqrMat Jeg(8,fmatvec::INIT,0.0), Jegp(8,fmatvec::INIT,0.0);

    //--- lokale  Koordinaten, Geschwingigkeiten
    double &xS     = qLokal(0);      double &yS     = qLokal(1);
    //    double &phiS   = qLokal(2);  unused
    double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);
    //
    double &xSp    = qpLokal(0);     double &ySp   = qpLokal(1);
    //    double &phiSp  = qpLokal(2); unused
    double &epsp  = qpLokal(3);
    double &aLp    = qpLokal(4);     double &bLp   = qpLokal(5);
    double &aRp    = qpLokal(6);     double &bRp   = qpLokal(7);


    //lokale Koordinate-----------------------------------------------------------
    BuildqLokal(qElement,qLokal);

    //JacobiMatrizen--------------------------------------------------------------
    BuildJacobi(qElement,qpElement,Jeg,Jegp);

    qpLokal << Jeg * qpElement;
    Data(0) = (32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2)/(420.*l0h2);
    Data(1) = (64*(17*aL*aLp - aLp*aR - aL*aRp + 17*aR*aRp)*(1 + eps) + 32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*epsp - 12*((8*aLp*bL - 3*aRp*bL + 8*aL*bLp - 3*aR*bLp - 3*aLp*bR + 8*aRp*bR - 3*aL*bRp + 8*aR*bRp)*(1 + eps) + (8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*epsp)*l0 + 3*(6*(3*bL*bLp - bLp*bR - bL*bRp + 3*bR*bRp)*(1 + eps) + (140 + 9*Power(bL,2) - 6*bL*bR + 9*Power(bR,2))*epsp)*l0h2)/ (420.*l0h2);
    Data(2) = xS;
    Data(3) = yS;
    Data(4) = xSp;
    Data(5) = ySp;
    Data(6) = bL  + bR;
    Data(7) = bLp + bRp;

    return Data;
  }

  //Energien
  double FiniteElement1s21RCM::computeKineticEnergy(const Vec& qElement, const Vec& qpElement) {
    Vec    qLokal(8,fmatvec::NONINIT), qpLokal(8,fmatvec::NONINIT);

    //--- lokale  Koordinaten, Geschwingigkeiten
    //    double &xS     = qLokal(0);    // unused  
    //    double &yS     = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);
    //
    double &xSp    = qpLokal(0);     double &ySp   = qpLokal(1);
    double &phiSp  = qpLokal(2);     double &epsp  = qpLokal(3);
    double &aLp    = qpLokal(4);     double &bLp   = qpLokal(5);
    double &aRp    = qpLokal(6);     double &bRp   = qpLokal(7);

    //lokale Koordinate-----------------------------------------------------------
    BuildqLokal(qElement,qLokal);

    //JacobiMatrizen--------------------------------------------------------------
    SqrMat Jeg   (8,fmatvec::INIT,0.0), Jegp   (8,fmatvec::INIT,0.0);
    BuildJacobi(qElement,qpElement,Jeg,Jegp);

    // Lokale Geschwindigkeit-----------------------------------------------------
    qpLokal << Jeg * qpElement;

    return (Arho*l0*(12608*Power(aLp,2) + 12608*Power(aRp,2) - 856*aRp*bLp*l0 + 1640*aL*bL*Power(bLp,2)*l0 + 224*aR*bL*Power(bLp,2)*l0 - 408*aL*Power(bLp,2)*bR*l0 - 48*aR*Power(bLp,2)*bR*l0 - 11176*aRp*bRp*l0 - 48*aL*bL*Power(bRp,2)*l0 - 408*aR*bL*Power(bRp,2)*l0 + 224*aL*bR*Power(bRp,2)*l0 + 1640*aR*bR*Power(bRp,2)*l0 - 672*aRp*bLp*eps*l0 - 12768*aRp*bRp*eps*l0 + 12768*aL*bLp*epsp*l0 + 672*aR*bLp*epsp*l0 + 672*aL*bRp*epsp*l0 + 12768*aR*bRp*epsp*l0 + 2488*Power(bLp,2)*l0h2 + 211*Power(bL,2)*Power(bLp,2)*l0h2 - 84*bL*Power(bLp,2)*bR*l0h2 + 9*Power(bLp,2)*Power(bR,2)*l0h2 + 336*bLp*bRp*l0h2 + 2488*Power(bRp,2)*l0h2 + 9*Power(bL,2)*Power(bRp,2)*l0h2 - 84*bL*bR*Power(bRp,2)*l0h2 + 211*Power(bR,2)*Power(bRp,2)*l0h2 + 5628*Power(bLp,2)*eps*l0h2 + 504*bLp*bRp*eps*l0h2 + 5628*Power(bRp,2)*eps*l0h2 + 3360*Power(bLp,2)*Power(eps,2)*l0h2 + 3360*Power(bRp,2)*Power(eps,2)*l0h2 + 1092*bL*bLp*epsp*l0h2 - 252*bLp*bR*epsp*l0h2 - 252*bL*bRp*epsp*l0h2 + 1092*bR*bRp*epsp*l0h2 + 6720*Power(epsp,2)*l0h2 - 2*(64*(196*Power(aL,2) + 17*aL*aR + Power(aR,2))*bLp - 64*(Power(aL,2) + 17*aL*aR + 196*Power(aR,2))*bRp + 8*(756*aRp*(1 + eps) + aR*(28*bL*bLp - 6*bLp*bR + 51*bL*bRp - 205*bR*bRp - 756*epsp) + aL*(205*bL*bLp - 51*bLp*bR + 6*bL*bRp - 28*bR*bRp + 756*epsp))*l0 + (bLp*(211*Power(bL,2) - 84*bL*bR + 9*Power(bR,2) + 672*(1 + eps)*(4 + 5*eps)) - bRp*(9*Power(bL,2) - 84*bL*bR + 211*Power(bR,2) + 672*(1 + eps)*(4 + 5*eps)) + 672*(bL - bR)*epsp)*l0h2)*phiSp + 4*(3152*Power(aL,2) + 3152*Power(aR,2) - 46*aR*bL*l0 + 398*aR*bR*l0 + (55*Power(bL,2) - 42*bL*bR + 55*Power(bR,2) + 1680*Power(1 + eps,2))*l0h2 + aL*(544*aR + 398*bL*l0 - 46*bR*l0))*Power(phiSp,2) + 8*aLp*(272*aRp + l0*(-1397*bLp - 107*bRp - 84*(19*bLp + bRp)*eps + 1512*(1 + eps)*phiSp)) + 64*((196*Power(aL,2) + 17*aL*aR + Power(aR,2))*Power(bLp,2) + (Power(aL,2) + 17*aL*aR + 196*Power(aR,2))*Power(bRp,2) + 1260*(Power(xSp,2) + Power(ySp,2))) - 336*(3*l0*(20*epsp + (5*bL - bR)*(bLp - phiSp))*xSp + 104*aL*(bLp - phiSp)*xSp + 8*aR*(bLp - phiSp)*xSp + 8*(13*aLp + aRp)*ySp - 3*l0*(bRp + 5*bLp*(3 + 4*eps) - 20*(1 + eps)*phiSp)*ySp)*cos(bL - phiS) + 336*((8*aL*(bRp + phiSp)*xSp + 104*aR*(bRp + phiSp)*xSp - 3*l0*(-20*epsp + (bL - 5*bR)*(bRp + phiSp))*xSp - 8*(aLp + 13*aRp)*ySp + 3*l0*(bLp + 5*bRp*(3 + 4*eps) + 20*(1 + eps)*phiSp)*ySp)*cos(bR + phiS) + (-((104*aLp + 8*aRp - 3*(bRp + 5*bLp*(3 + 4*eps))*l0 + 60*(1 + eps)*l0*phiSp)*xSp) + (3*l0*(20*epsp + (5*bL - bR)*(bLp - phiSp)) + 104*aL*(bLp - phiSp) + 8*aR*(bLp - phiSp))*ySp)*sin(bL - phiS) + ((8*aLp + 104*aRp - 3*l0*(bLp + 5*bRp*(3 + 4*eps) + 20*(1 + eps)*phiSp))*xSp + (8*aL*(bRp + phiSp) + 104*aR*(bRp + phiSp) - 3*l0*(-20*epsp + (bL - 5*bR)*(bRp + phiSp)))*ySp)*sin(bR + phiS))))/161280.;
  }

  double FiniteElement1s21RCM::computeV(const Vec& qElement) {
    Vec    qLokal(8,fmatvec::NONINIT);

    //--- lokale  Koordinaten, Geschwingigkeiten
    double &xS     = qLokal(0);    // unused  
    double &yS     = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

    // Gravitation
    double gx = g(0);
    double gy = g(1);

    //lokale Koordinate-----------------------------------------------------------
    BuildqLokal(qElement,qLokal);

    return (EA*Power(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2,2) + 35280*EI*(640*Power(aL,2) + 640*Power(aR,2) + 8*aR*(11*bL - 43*bR)*l0 - 8*aL*(32*aR + 43*bL*l0 - 11*bR*l0) + l0h2*(57*Power(bL,2) - 30*bL*bR + 57*Power(bR,2) - 10*(bL + bR)*l0*wss0 + 5*l0h2*Power(wss0,2))))/(352800.*l0h3) - (Arho*l0*(480*(gx*xS + gy*yS) - (60*(1 + eps)*gx*l0 + gy*(104*aL + 8*aR + 15*bL*l0 - 3*bR*l0))*cos(bL - phiS) + (-8*(aL + 13*aR)*gy + 60*(1 + eps)*gx*l0 + 3*(bL - 5*bR)*gy*l0)*cos(bR + phiS) + (-104*aL*gx - 8*aR*gx + 3*((-5*bL + bR)*gx + 20*(1 + eps)*gy)*l0)*sin(bL - phiS) + (8*aL*gx + 104*aR*gx + 3*(-(bL*gx) + 5*bR*gx + 20*(1 + eps)*gy)*l0)*sin(bR + phiS)))/480.;
  }

  double FiniteElement1s21RCM::computeElasticEnergy(const Vec& qElement) {
    Vec    qLokal(8,fmatvec::NONINIT);

    //--- lokale  Koordinaten, Geschwingigkeiten
    double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

    //lokale Koordinate-----------------------------------------------------------
    BuildqLokal(qElement,qLokal);

    return (EA*Power(32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2,2) + 35280*EI*(640*Power(aL,2) + 640*Power(aR,2) + 8*aR*(11*bL - 43*bR)*l0 - 8*aL*(32*aR + 43*bL*l0 - 11*bR*l0) + l0h2*(57*Power(bL,2) - 30*bL*bR + 57*Power(bR,2) - 10*(bL + bR)*l0*wss0 + 5*l0h2*Power(wss0,2))))/(352800.*l0h3);
  }

  double FiniteElement1s21RCM::computeGravitationalEnergy(const Vec& qElement) {
    Vec    qLokal(8,fmatvec::NONINIT);

    //--- lokale  Koordinaten, Geschwingigkeiten
    double &xS     = qLokal(0);
    double &yS     = qLokal(1);
    double &phiS   = qLokal(2);      double &eps   = qLokal(3);
    double &aL     = qLokal(4);      double &bL    = qLokal(5);
    double &aR     = qLokal(6);      double &bR    = qLokal(7);

    // Gravitation
    double gx = g(0);
    double gy = g(1);

    //lokale Koordinate-----------------------------------------------------------
    BuildqLokal(qElement,qLokal);

    return - (Arho*l0*(480*(gx*xS + gy*yS) - (60*(1 + eps)*gx*l0 + gy*(104*aL + 8*aR + 15*bL*l0 - 3*bR*l0))*cos(bL - phiS) + (-8*(aL + 13*aR)*gy + 60*(1 + eps)*gx*l0 + 3*(bL - 5*bR)*gy*l0)*cos(bR + phiS) + (-104*aL*gx - 8*aR*gx + 3*((-5*bL + bR)*gx + 20*(1 + eps)*gy)*l0)*sin(bL - phiS) + (8*aL*gx + 104*aR*gx + 3*(-(bL*gx) + 5*bR*gx + 20*(1 + eps)*gy)*l0)*sin(bR + phiS)))/480.;
  }

}

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------
