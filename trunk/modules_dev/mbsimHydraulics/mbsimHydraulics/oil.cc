/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: schneidm@users.berlios.de
 */

#include "oil.h"
#include <math.h>
#include <iostream>

using namespace std;

//Oil::Oil():HydFluid(857,821.7e6,0,4./3.), eta(0), m(0), Tm(0), Wm(0), ubbelohde(false), rho0(0), T0(0), dVdT(0), dRhodT(0) {
//} 
//
//Oil::Oil(OilType _type, double _T):HydFluid(857,821.7e6,0,4./3.), eta(0), m(0), Tm(0), Wm(0), ubbelohde(false), rho0(0), T0(0), dVdT(0), dRhodT(0) {
//  setType(_type);
//  init(_T);
//  T=_T;
//}
//
//void Oil::setType(OilType _type) {
//  double nu1_,T1_,T2_,nu2_,rho0_,T0_,dRhodT_;
//  if(_type==CastrolLongLife2TopUp_0W30) {
//    T1_ =     40;
//    T2_ =     100;
//    nu1_=     55.3*1e-6; //in m^2/s    cSt=mm^2/s=1e-6m^2/s
//    nu2_=     10.1*1e-6;
//    rho0_=    842.7;
//    T0_ =      15.;
//    dRhodT_= -0.7;
//  }
//  else if(_type==CastrolLongLife3TopUp_5W30) {
//    T1_ =    40;
//    T2_ =    100;
//    nu1_=    73.4*1e-6;
//    nu2_=    11.9*1e-6;
//    rho0_ =    850.;
//    T0_ =      15.;
//    dRhodT_=  -0.7; //geraten
//  }
//  else {
//    cout << "oil.cc: Unknown Oil!"; 
//    throw 50;
//  }
//  setdRhodT(dRhodT_,rho0_,T0_);
//  setUbbelohde(T1_,nu1_,T2_,nu2_);
//}
//
//void Oil::setUbbelohde(double T1,double nu1, double T2,double nu2) {
//  ubbelohde=true; 
//  Tm=T1+273.16;
//  Wm=log10(log10(nu1*1e6+0.8));  //Umrechnung in cSt
//  T2=T2+273.16;
//  double W2=log10(log10(nu2*1e6+0.8));
//  m=(Wm-W2)/(log10(T2)-log10(Tm));
//}
//
//void Oil::setdVdT(double dVdT_,double rho0_,double T0_) {
//  dVdT=dVdT_; 
//  rho0=rho0_; 
//  rho=rho0_; 
//  T0=T0_; 
//}
//
//void Oil::setdRhodT(double dRhodT_,double rho0_,double T0_) {
//  dRhodT=dRhodT_; 
//  rho0=rho0_; 
//  rho=rho0_; 
//  T0=T0_; 
//}
//
//void Oil::init(double T_) {
//  T=T_;
//  nu=getNu(T_);
//  rho=getRho(T_);
//  eta=getEta(T_);
//  cout << endl;
//  cout << "=====================================" << endl;
//  cout << "Initialisiere Oel mit T=" << T_ << " [degC]" << endl;
//  cout << "                     nu=" << nu*1e6 << " [mm^2/s]" << endl;
//  cout << "                    rho=" << rho << " [kg/m^3]" << endl;
//  cout << "                    eta=" << eta << " [Pa*s]" << endl;
//  cout << "                  kappa=" << kappa << " [-]" << endl;
//  cout << "=====================================\n\n" << endl;
//}
//
//double Oil::getNu(double T_) {
//  if(ubbelohde) {
//    double Tx=T_+273.16;
//    double Wx=m*(log10(Tm)-log10(Tx))+Wm;
//    return (pow(10,pow(10,Wx))-0.8)*1e-6; //Umrechnung zu m^2/s
//  }
//  else
//    return nu;
//}
//
//double Oil::getEta(double _T) {
//  return getNu(_T)*getRho(_T);
//}
//
//double Oil::getRho(double _T) {
//  if(dRhodT)
//    return rho0  + dRhodT*(_T- T0 );
//  else if(dVdT)
//    return rho0*(1./(1+dVdT*(_T-T0)));
//  else
//    return rho;
//}
