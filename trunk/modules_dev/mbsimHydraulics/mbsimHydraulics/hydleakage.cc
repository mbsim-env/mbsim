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

#include "hydleakage.h"
#include "mbsim/dynamic_system_solver.h"
#include "environment.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void HydLeakage::setGeometry(double lGap_, double hGap_, double wGap_) {
    lGap=lGap_;
    hGap=hGap_;
    wGap=wGap_;
  }

  void HydLeakage::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      double d=sqrt(4.*hGap*wGap/M_PI);
      double l=lGap;
      //    double dmin=2e-3;
      //    double lmin=10e-3;
      //    d=(d<dmin)?dmin:d;
      //    l=(l<lmin)?lmin:l;
      cout << name << ": gapLength=" << lGap*1e3 << "[mm]  equivalent diamter=" << d*1e3 << "[mm]" << endl;
      HydLine::setDiameter(d);
      HydLine::setLength(l);
      HydLine::init(stage);
    }
    if (stage==MBSim::unknownStage) {
      HydLine::init(stage);
      for (unsigned int i=0; i<pd.size(); i++)
        if (dynamic_cast<LeakagePressureLoss*>(pd[i]))
          static_cast<LeakagePressureLoss*>(pd[i])->transferLeakageGapData(lGap, hGap, wGap);
    }
    else
      HydLine::init(stage);
  }



  LeakagePressureLoss::LeakagePressureLoss(const string &name) : PressureLoss(name) {
  }

  LeakagePressureLossHagenPoiseuille::LeakagePressureLossHagenPoiseuille(const string &name) : LeakagePressureLoss(name) {
  }

  void LeakagePressureLossHagenPoiseuille::transferLeakageGapData(double lGap, double hGap, double wGap) {
    lossFactor=12.*HydraulicEnvironment::getInstance()->getDynamicViscosity()*lGap/wGap/hGap/hGap/hGap;
    cout << "lossFactor=" << lossFactor << endl;
  }

  double LeakagePressureLossHagenPoiseuille::operator()(double Q){
    return lossFactor*Q;
  }

}

