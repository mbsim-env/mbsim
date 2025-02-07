/* Copyright (C) 2004-2025  Martin Förg
 
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
 *   martin.o.foerg@googlemail.com
 *
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include "dae_integrator.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

  void setZero(Mat &A, const RangeV &rows, const RangeV &cols) {
    for(int i=rows.start(); i<=rows.end(); i++) {
      for(int j=cols.start(); j<=cols.end(); j++)
        A(i,j) = 0;
    }
  }

  double ImplicitIntegrator::delta(int i, double z) const {
    return sqrt(macheps*max(1.e-5,abs(z)));
  }

  void ImplicitIntegrator::par_ud_xd_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->updateud();
      system->updatexd();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::par_zd_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->updatezd();
      for(int i=0; i<system->getzSize(); i++)
        J(i,j)=(zd(i)-zd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::par_ud_xd_par_u_x(Mat &J, bool updla) {
    const Vec &zd = system->getzd(false);
    for(int j=system->getqSize(); j<system->getzSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(updla);
      system->updateud();
      system->updatexd();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::par_zd_par_z(Mat &J, bool updla) {
    const Vec &zd = system->getzd(false);
    for(int j=0; j<system->getzSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->updatezd();
      for(int i=qMove; i<neq; i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::calcSize() {
    neq = system->getzSize();
  }

  void ImplicitIntegrator::init() {
    calcSize();
    qMove = reduced ? system->getqSize() : 0;
    Rq = RangeV(0,system->getqSize()-1);
    Ru = RangeV(system->getqSize(),system->getqSize()+system->getuSize()-1);
    Rx = RangeV(system->getqSize()+system->getuSize(),system->getzSize()-1);
    Rz = RangeV(0,system->getzSize()-1);
    RuMove = RangeV(Ru.start()-qMove, Ru.end()-qMove);
    RxMove = RangeV(Rx.start()-qMove, Rx.end()-qMove);
    zd0.resize(system->getzSize(),NONINIT);
  }

}
