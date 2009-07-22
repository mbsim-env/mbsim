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

#include "hydfluid.h"
#include "mbsim/utils/eps.h"
#include <iostream>
#include <math.h>

using namespace std;

namespace MBSim {

  HydFluid::HydFluid() : rho(0), E(0), nu(0), kappa(0), T(0) {
  }

  HydFluid::HydFluid(double rho_, double E_, double nu_, double kappa_) {
    setProperties(rho_, E_, nu_, kappa_);
  }

  void HydFluid::setProperties(double rho_, double E_, double nu_, double kappa_) {
    rho=rho_;
    E = E_;
    nu = nu_;
    kappa=kappa_;

    f1=pow(1.e5, 1./kappa)*E/kappa;
    f2=-(1.+1./kappa);
  }

  double HydFluid::getE(double p, double fracAir) {
    if(p<=0.1) {
      cout << "HydFluid: pressure near zero! Continuing anyway, using p=0.1 Pa" << endl;
      p=0.1;
    }
    // Umdruck zur Vorlesung
    // Grundlagen der Oelhydraulik
    // W.Backe
    // H.Murrenhoff
    // 10. Auflage 1994
    // Formel (3-11), S. 103
    return E * ((fracAir<epsroot()) ?
      1. :
      //(1.+fracAir) / (1.+pow((1.e5/p),(1./kappa))*fracAir*E/(kappa*p)));
      // rechentechnisch optimiert:
      (1.+fracAir) / (1. + f1*fracAir*pow(p, f2)));
  }

  double HydFluid::getRho(double T_) {
    cout << "Caution! No dependy on temperature considered." << endl;
    return getRho();
  }

  double HydFluid::getNu(double T_) {
    cout << "Caution! No dependy on temperature considered." << endl;
    return getNu();
  }

  double HydFluid::getEta(double T_) {
    cout << "Caution! No dependy on temperature considered." << endl;
    return getEta(); 
  }

  double HydFluid::getTemperature() {
    if (T==0)
      cout << "Caution! No temperature set." << endl;
    return T;
  }

}
