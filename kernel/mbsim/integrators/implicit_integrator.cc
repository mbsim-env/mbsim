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
        A.e(i,j) = 0;
    }
  }

  void ImplicitIntegrator::par_ud_xd_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->updateud();
      system->updatexd();
      for(int i=RuMove.start(),k=0; i<=RuMove.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=RxMove.start(),k=0; i<=RxMove.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::par_zd_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->updatezd();
      for(int i=Rq.start(); i<=Rq.end(); i++)
        J(i,j)=(system->getqd(false)(i)-qd0(i))/delta;
      for(int i=Ru.start(),k=0; i<=Ru.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=Rx.start(),k=0; i<=Rx.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::par_ud_xd_par_u_x(Mat &J, bool updla) {
    for(int j=system->getqSize(); j<system->getzSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(updla);
      system->updateud();
      system->updatexd();
      for(int i=RuMove.start(),k=0; i<=RuMove.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=RxMove.start(),k=0; i<=RxMove.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  void ImplicitIntegrator::calcSize() {
    neq = system->getzSize();
  }

  void ImplicitIntegrator::init() {
    calcSize();
    rowMove = reduced ? system->getqSize() : 0;
    Rq = RangeV(0,system->getqSize()-1);
    Ru = RangeV(system->getqSize(),system->getqSize()+system->getuSize()-1);
    Rx = RangeV(system->getqSize()+system->getuSize(),system->getzSize()-1);
    Rz = RangeV(0,system->getzSize()-1);
    RuMove = RangeV(Ru.start()-rowMove, Ru.end()-rowMove);
    RxMove = RangeV(Rx.start()-rowMove, Rx.end()-rowMove);
    res0.resize(neq,NONINIT);
    qd0.ref(res0,Rq);
    ud0.ref(res0,Ru);
    xd0.ref(res0,Rx);
  }

}
